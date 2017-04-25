
from collections import deque
from threading import Semaphore, Lock
from struct import Struct
from errno import errorcode, ETIMEDOUT
from uuid import UUID
from time import time, sleep
import logging
import msgpack

import usb.core
import usb.util

from fluxclient.utils.version import StrictVersion
from fluxclient import __version__

logger = logging.getLogger(__name__)
HEAD_PACKER = Struct("<HB")
PING_PACKER = Struct("<HBB")
DEVST_PACKER = Struct("<dif16s32s")
ID_VENDOR = 0xffff
ID_PRODUCT = 0xfd00
USBTIMEOUT = 1.5


def match_direction(direction):
    def wrapper(ep):
        return usb.util.endpoint_direction(ep.bEndpointAddress) == direction
    return wrapper


class USBProtocol(object):
    channels = None
    running = False
    session = None
    uuid = None
    device_status = None
    timestamp = 0
    _flag = 0
    _buf = b""

    _usbdev = None
    _enable_ping = False
    _enable_padding = False
    _wait_ping = False

    @classmethod
    def get_interfaces(cls):
        return list(usb.core.find(idVendor=ID_VENDOR, idProduct=ID_PRODUCT,
                                  find_all=True))

    def __init__(self, usbdev):
        # from usb.backend import libusb1
        # libusb1._lib.libusb_set_debug(usbdev.backend.ctx, 4)
        self._usbdev = dev = usbdev
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except NotImplementedError:
            pass

        dev.set_configuration()
        cfg = dev.get_active_configuration()
        intf = cfg[(0, 0)]

        try:
            self._rx = usb.util.find_descriptor(
                intf, bmAttributes=0x2,
                custom_match=match_direction(usb.util.ENDPOINT_IN))

            self._tx = usb.util.find_descriptor(
                intf, bmAttributes=0x2,
                custom_match=match_direction(usb.util.ENDPOINT_OUT))
            logger.info("Host2Host USB device opened")

            import IPython
            IPython.embed()
            self.tx_mutex = Lock()
            self.chl_semaphore = Semaphore(0)
            self.chl_open_mutex = Lock()
            self.channels = {}
            self.device_status = {}
            self.addr = usbdev.address

            self.do_handshake()

        except Exception:
            self.close()
            raise

    def __del__(self):
        try:
            self.close()
        except Exception as e:
            logger.error("%s", e)

    def _send(self, buf):
        # Low level send
        try:
            with self.tx_mutex:
                ret = self._tx.write(buf)
                while ret < len(buf):
                    ret += self._tx.write(buf)

        except usb.core.USBError as e:
            self._close_usbdev()
            if e.errno == ETIMEDOUT:
                raise FluxUSBError(*e.args, symbol=("TIMEOUT", ))
            else:
                raise FluxUSBError(*e.args,
                                   symbol=("UNKNOWN_ERROR",
                                           errorcode.get(e.errno, e.errno)))

    def _send_binary_ack(self, channel_idx):
        self._send(HEAD_PACKER.pack(4, channel_idx) + b"\x80")

    def _recv(self, length, timeout):
        # Low level recv
        try:
            # note: thread will dead lock if tiemout is 0
            b = self._rx.read(length, timeout).tobytes()
            self.timestamp = time()
            return b
        except usb.core.USBError as e:
            if e.errno == ETIMEDOUT or e.backend_error_code == -116:
                return b""
            else:
                self._close_usbdev()
                raise FluxUSBError(*e.args)

    def _feed_buffer(self, timeout=50):
        self._buf += self._recv(1024, timeout)

    def _unpack_buffer(self):
        l = len(self._buf)
        if l > 3:
            size, channel_idx = HEAD_PACKER.unpack(self._buf[:3])
            if size < 3:
                if size == 0:
                    self._buf = self._buf[2:]
                    return -1, None, None
                else:
                    raise FluxUSBError("Recv bad usb message size: %i" % size)
            elif l >= size:
                fin = self._buf[size - 1]
                buf = self._buf[3:size - 1]
                self._buf = self._buf[size:]
                return channel_idx, buf, fin
        return None, None, None

    def _handle_handshake(self, buf):
        data = msgpack.unpackb(buf, use_list=False, encoding="utf8",
                               unicode_errors="ignore")
        session = data.pop("session", "?")
        if self.session is None:
            logger.info("Get handshake session: %s", session)
        else:
            logger.info("Replace handshake session: %s", session)

        self.endpoint_profile = data
        self.session = session
        self.send_object(0xfe, {"session": self.session,
                                "protocol_level": 1,
                                "client": "fluxclient-%s" % __version__})

    def _final_handshake(self, buf):
        data = msgpack.unpackb(buf, use_list=False, encoding="utf8",
                               unicode_errors="ignore")
        if data["session"] == self.session:
            self.uuid = UUID(hex=self.endpoint_profile["uuid"])
            self.serial = self.endpoint_profile["serial"]
            self.version = StrictVersion(self.endpoint_profile["version"])
            self.model_id = self.endpoint_profile["model"]
            self.nickname = self.endpoint_profile["nickname"]

            self._apply_protocol_compatibility(data)
            self._flag |= 1
            logger.info("Host2Host USB Connected")
            logger.debug("Serial: %s {%s}\nModel: %s\nName: %s\n",
                         self.serial, self.uuid, self.model_id, self.nickname)
            return True
        else:
            logger.info("USB final handshake error with wrong session "
                        "recv=%i, except=%i", data["session"], self.session)
            self.session = None
            return False

    def _apply_protocol_compatibility(self, resp):
        self._enable_ping = self.version >= StrictVersion("1.6.3")

        pl = resp.get("protocol_level", 0)
        if pl > 0:
            self._enable_padding = True

        logger.debug("Apply usb protocol enable_ping=%s, enable_padding=%s",
                     self._enable_ping, self._enable_padding)

    def do_handshake(self):
        ttl = 3
        self._usbdev.ctrl_transfer(0x40, 0xFD, 0, 0)

        self._send(b"\x00" * 1024)
        self.send_object(0xfc, None)  # Request handshake
        while ttl:
            bl = -1
            self._buf = b""
            while len(self._buf) != bl:
                bl = len(self._buf)
                self._feed_buffer(timeout=300)

            data = None
            while True:
                d = self._unpack_buffer()
                if d[0] is None:
                    break
                else:
                    data = d

            if data and data[0] is not None:
                channel_idx, buf, fin = data
                if channel_idx == 0xff and fin == 0xf0:
                    self._handle_handshake(buf)
                    continue
                elif channel_idx == 0xfd and fin == 0xf0:
                    if self.session is not None:
                        if self._final_handshake(buf):
                            return True
                    else:
                        logger.warning("Recv unexcept final handshake")
                elif channel_idx == -1:
                    logger.warning("Recv 0")
                    continue
                else:
                    logger.warning("Recv unexcept channel idx %r and fin "
                                   "%r in handshake", channel_idx, fin)
            else:
                logger.info("Handshake timeout, retry")

            self.send_object(0xfc, None)  # Request handshake
            sleep(1.0)
            ttl -= 1
        raise FluxUSBError("Handshake failed.", symbol=("TIMEOUT", ))

    def run_once(self):
        self._feed_buffer()
        channel_idx, buf, fin = self._unpack_buffer()
        if channel_idx is None:
            return
        elif channel_idx == -1:
            raise FluxUSBError("USB protocol broken, got zero data length.")
        elif channel_idx < 0x80:
            channel = self.channels.get(channel_idx)
            if channel is None:
                raise FluxUSBError("Recv bad channel idx 0x%02x" % channel_idx)
            if fin == 0xf0:
                channel.on_object(msgpack.unpackb(buf, encoding="utf8",
                                  unicode_errors="ignore"))
            elif fin == 0xff:
                self._send_binary_ack(channel_idx)
                channel.on_binary(buf)
            elif fin == 0xc0:
                channel.on_binary_ack()
            else:
                raise FluxUSBError("Recv bad fin 0x%02x" % fin)
        elif channel_idx == 0xa0 and fin == 0xff:
            logger.debug("Recv padding")
        elif channel_idx == 0xf1:
            if fin != 0xf0:
                raise FluxUSBError("Recv bad fin 0x%02x" % fin)
            self._on_channel_ctrl_response(msgpack.unpackb(buf))
        elif channel_idx == 0xfb:
            self._on_pong(buf)
        else:
            self.stop()
            self.close()
            raise FluxUSBError("Recv bad control channel 0x%02x" % channel_idx)

    def run(self):
        try:
            self._flag |= 2
            while self._flag == 3:
                self.run_once()
                if time() - self.timestamp > USBTIMEOUT:
                    self.ping()
        except FluxUSBError as e:
            logger.error("USB Error: %s", e)
            self._flag = 0
            self.close()
        except Exception:
            logger.exception("Unknown error")
            self._flag = 0
            self.close()
            raise

    def stop(self):
        self._flag &= ~2

    def _close_usbdev(self):
        if self._usbdev:
            if self.channels:
                for idx, channel in self.channels.items():
                    channel.close(directly=True)
            usb.util.dispose_resources(self._usbdev)
            self._usbdev = self._tx = self._rx = None
            logger.info("Host2Host dev closed")

    def close(self):
        if self._usbdev:
            self.send_object(0xfc, None)
            self._close_usbdev()

    def ping(self):
        if self._enable_ping is False:
            return

        if self._wait_ping:
            raise FluxUSBError("PING/PONG timeout")
        else:
            self._wait_ping = True
            self._send(PING_PACKER.pack(4, 0xfa, 0x00))

    def send_object(self, chl_idx, obj):
        data = msgpack.packb(obj)
        l = len(data) + 4
        if l < 508 and self._enable_padding and self.uuid:
            padding = 512 - l
            buf = b"".join((
                HEAD_PACKER.pack(l, chl_idx), data, b"\xb0",
                HEAD_PACKER.pack(padding, 0xa0), b"\x00" * (padding - 4), b"\xff"))  # noqa
        else:
            buf = b"".join((HEAD_PACKER.pack(l, chl_idx), data, b"\xb0"))

        # buf = HEAD_PACKER.pack(len(data) + 4, chl_idx) + data + b"\xb0"
        self._send(buf)

    def send_binary(self, chl_idx, data):
        l = len(data) + 4
        # buf = HEAD_PACKER.pack(len(data) + 4, chl_idx) + data + b"\xbf"
        if l < 508 and self._enable_padding and self._proto_handshake:
            padding = 512 - l
            buf = b"".join((
                HEAD_PACKER.pack(l, chl_idx), data, b"\xbf",
                HEAD_PACKER.pack(padding, 0xa0), b"\x00" * (padding - 4), b"\xff"))  # noqa
        else:
            buf = b"".join((HEAD_PACKER.pack(l, chl_idx), data, b"\xbf"))
        self._send(buf)

    def _on_pong(self, buf):
        if self._wait_ping:
            self._wait_ping = False
            if len(buf) != 64:
                logger.error("PONG payload length: %i (should be 64)",
                             len(buf))
            else:
                st_ts, st_id, st_prog, st_th, st_er = DEVST_PACKER.unpack(buf)
                self.device_status = {
                    "timestamp": st_ts, "st_id": st_id, "st_prog": st_prog,
                    "st_head": st_th.decode("ascii").rstrip("\x00"),
                    "st_err": st_er.decode("ascii").rstrip("\x00")}
        else:
            logger.debug("Recv pong but did not ping")
            raise FluxUSBError("Protocol error")

    def _on_channel_ctrl_response(self, obj):
        index = obj.get(b"channel")
        status = obj.get(b"status")
        action = obj.get(b"action")
        if action == b"open":
            if status == b"ok":
                self.channels[index] = Channel(self, index)
                self.chl_semaphore.release()
                logger.info("Channel %i opened", index)
            else:
                logger.error("Channel %i open failed", index)
        elif action == b"close":
            if status == b"ok":
                self.channels.pop(index)
                logger.info("Channel %i closed", index)
            else:
                logger.error("Channel %i close failed", index)
        else:
            logger.error("Unknown channel action: %r", action)

    def _close_channel(self, channel):
        logger.info("Close channel %i", channel.index)
        self.send_object(0xf0, {"channel": channel.index, "action": "close"})

    def open_channel(self, channel_type="robot", timeout=10.0):
        # Send request
        with self.chl_open_mutex:
            idx = None
            for i in range(len(self.channels) + 1):
                if self.channels.get(i) is None:
                    idx = i
            logger.info("Request channel %i with type %s", idx, channel_type)
            self.send_object(0xf0, {"channel": idx, "action": "open",
                                    "type": channel_type})

            self.chl_semaphore.acquire(timeout=timeout)
            channel = self.channels.get(idx)
            if channel:
                return self.channels[idx]
            else:
                raise FluxUSBError("Channel creation failed")


class Channel(object):
    binary_stream = None

    def __init__(self, usbprotocol, index):
        self.index = index
        self.usbprotocol = usbprotocol
        self.obj_semaphore = Semaphore(0)
        self.buf_semaphore = Semaphore(0)
        self.ack_semaphore = Semaphore(0)
        self.objq = deque()
        self.bufq = deque()

        self.__opened = True

    def __del__(self):
        self.close()

    @property
    def alive(self):
        return self.__opened

    def close(self, directly=False):
        if self.__opened:
            self.__opened = False
            if directly is False:
                self.usbprotocol._close_channel(self)

    def on_object(self, obj):
        self.objq.append(obj)
        self.obj_semaphore.release()

    def on_binary(self, buf):
        if self.binary_stream:
            try:
                self.binary_stream.send(buf)
            except (OSError, IOError) as e:
                self.close()
                logger.warning("Send USB binary data to %s error (%s), "
                               "close channel directly (channel=%i)",
                               self.binary_stream, e, self.index)
        else:
            self.bufq.append(buf)
            self.buf_semaphore.release()

    def get_buffer(self, timeout=60.0):
        if self.buf_semaphore.acquire(timeout=timeout) is False:
            raise FluxUSBError("Operation timeout", symbol=("TIMEOUT", ))
        return self.bufq.popleft()

    def on_binary_ack(self):
        self.ack_semaphore.release()

    def get_object(self, timeout=10.0):
        if self.obj_semaphore.acquire(timeout=timeout) is False:
            raise FluxUSBError("Operation timeout", symbol=("TIMEOUT", ))
        return self.objq.popleft()

    def send_object(self, obj):
        if self.__opened:
            self.usbprotocol.send_object(self.index, obj)
        else:
            raise FluxUSBError("Device is closed",
                               symbol=("DEVICE_ERROR", ))

    def send_binary(self, buf, timeout=10.0):
        if self.__opened:
            self.usbprotocol.send_binary(self.index, buf)
            if self.ack_semaphore.acquire(timeout=timeout) is False:
                raise FluxUSBError("Operation timeout", symbol=("TIMEOUT", ))
        else:
            raise FluxUSBError("Device is closed",
                               symbol=("DEVICE_ERROR", ))


class FluxUSBError(Exception):
    def __init__(self, *args, **kw):
        self.symbol = kw.get("symbol", ("UNKNOWN_ERROR", ))
        super().__init__(*args)
