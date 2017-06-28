
from collections import deque
from threading import Semaphore, Lock
from struct import Struct
from errno import errorcode, ETIMEDOUT, ENODEV
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
HEAD_V2_PACKER = Struct("<HHBB")
CTRL_PACKER = Struct("<HBB")
BYTE_PACKER = Struct("<B")
DEVST_PACKER = Struct("<dif16s32s")
ID_VENDOR = 0xffff
ID_PRODUCT = 0xfd00
USBTIMEOUT = 1.5


def match_direction(direction):
    def wrapper(ep):
        return usb.util.endpoint_direction(ep.bEndpointAddress) == direction
    return wrapper


class USBProtocol(object):
    @classmethod
    def get_interfaces(cls):
        return list(usb.core.find(idVendor=ID_VENDOR, idProduct=ID_PRODUCT,
                                  find_all=True))

    @classmethod
    def connect(cls, dev):
        # from usb.backend import libusb1
        # libusb1._lib.libusb_set_debug(usbdev.backend.ctx, 4)

        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except NotImplementedError:
            pass

        try:
            dev.set_configuration()
            cfg = dev.get_active_configuration()
            intf = cfg[(0, 0)]
        except usb.core.USBError as e:
            if e.errno == ENODEV:
                raise FluxUSBError("USB not available.",
                                   symbol=("UNAVAILABLE", ))
            else:
                raise

        try:
            tx = usb.util.find_descriptor(
                intf, bmAttributes=0x2,
                custom_match=match_direction(usb.util.ENDPOINT_OUT))

            rx = usb.util.find_descriptor(
                intf, bmAttributes=0x2,
                custom_match=match_direction(usb.util.ENDPOINT_IN))

            logger.debug("Host2Host USB device opened")

            profile = cls._do_handshake(dev, tx, rx)
            level = profile["final"].get("protocol_level", 0)

            if level <= 1:
                logger.debug("Use USBProtocol1")
                return USBProtocol1(dev, tx, rx, profile)
            elif level == 2:
                logger.debug("Use USBProtocol2")
                return USBProtocol2(dev, tx, rx, profile)
            else:
                raise FluxUSBError("USB protocol not support (v%i??)" % level,
                                   symbol=("NOT_SUPPORT", ))
        except Exception:
            usb.util.dispose_resources(dev)
            raise

    @classmethod
    def _handle_handshake(cls, handshake_state, buf, tx):
        data = msgpack.unpackb(buf, use_list=False, encoding="utf8",
                               unicode_errors="ignore")
        session = data.pop("session", "?")
        if handshake_state.get("session") is None:
            logger.info("Get handshake session: %s", session)
        else:
            logger.info("Replace handshake session: %s", session)

        handshake_state["session"] = session
        handshake_state["endpoint"] = data

        resp_body = {"session": session, "protocol_level": 2,
                     "client": "fluxclient-%s" % __version__}
        cls._write(tx, cls.build_object(0xfe, resp_body, 0xb0))

    @classmethod
    def _final_handshake(cls, handshake_state, buf, tx):
        if "session" not in handshake_state:
            logger.warning("Recv unexcept final handshake")
            return None

        data = msgpack.unpackb(buf, use_list=False, encoding="utf8",
                               unicode_errors="ignore")

        if data.get("session") == handshake_state["session"]:
            endpoint_profile = handshake_state["endpoint"]
            endpoint_profile["final"] = data
            logger.info("Host2Host USB Connected")
            logger.debug("Serial: %(serial)s {%(uuid)s}\nModel: %(model)s\n"
                         "Name: %(nickname)s\n", endpoint_profile)
            return endpoint_profile
        else:
            logger.info("USB final handshake error with wrong session "
                        "recv=%i, except=%i", data["session"],
                        endpoint_profile["session"])
            return None

    @classmethod
    def _do_handshake(cls, dev, tx, rx):
        ttl = 3
        dev.ctrl_transfer(0x40, 0xFD, 0, 0)

        req_handshake_payload = cls.build_object(0xfc, None, 0xb0)
        handshake_state = {}
        cls._write(tx, b"\x00" * 1024)
        cls._write(tx, req_handshake_payload)

        while ttl:
            bl = -1
            buf = b""
            while len(buf) != bl:
                bl = len(buf)
                buf += cls._recv(rx, 512, timeout=300)

            data = None
            while len(buf) > 3:
                size, channel_idx = HEAD_PACKER.unpack(buf[:3])
                if size % 256 == 0:
                    buf = buf[1:]
                    continue
                elif len(buf) == size and channel_idx != 0xa0:
                    data = channel_idx, buf[3:-1], buf[-1]
                    break
                elif len(buf) > size:
                    buf = buf[size:]
                else:
                    logger.error("Handshake payload size error")
                    data = None
                    buf = b""

            if data:
                channel_idx, buf, fin = data
                if channel_idx == 0xff and fin == 0xf0:
                    cls._handle_handshake(handshake_state, buf, tx)
                    sleep(0.1)
                    continue
                elif channel_idx == 0xfd and fin == 0xf0:
                    pf = cls._final_handshake(handshake_state, buf, tx)
                    if pf:
                        return pf
                else:
                    logger.warning("Recv unexcept channel idx %r and fin "
                                   "%r in handshake", channel_idx, fin)
            else:
                logger.info("Handshake timeout, retry")

            handshake_state = {}
            cls._write(tx, req_handshake_payload)
            sleep(1.0)
            ttl -= 1
        raise FluxUSBError("Handshake failed.", symbol=("TIMEOUT", ))

    @classmethod
    def _write(cls, tx, buf):
        # Low level send
        try:
            l = len(buf)
            ret = tx.write(buf[:512])
            while ret < l:
                ret += tx.write(buf[ret:ret + 512])

        except usb.core.USBError as e:
            if e.errno == ETIMEDOUT or e.backend_error_code == -116:
                raise FluxUSBError(*e.args, symbol=("TIMEOUT", ))
            else:
                logger.error("unhandle libusb error: %s", e)
                raise FluxUSBError(*e.args,
                                   symbol=("UNKNOWN_ERROR",
                                           errorcode.get(e.errno, e.errno)))

    @classmethod
    def _recv(cls, rx, length, timeout=50):
        # Low level recv
        try:
            # note: thread will dead lock if tiemout is 0
            b = rx.read(length, timeout).tobytes()
            return b
        except usb.core.USBError as e:
            if e.errno == ETIMEDOUT or e.backend_error_code == -116:
                return b""
            else:
                logger.error("unhandle libusb error: %s", e)
                raise FluxUSBError(*e.args)

    @classmethod
    def build_object(cls, chl_idx, obj, fin):
        data = msgpack.packb(obj)
        l = len(data) + 4
        return b"".join((HEAD_PACKER.pack(l, chl_idx), data, BYTE_PACKER.pack(fin)))

    _dev = None
    _tx = _rx = None
    uuid = None
    serial = None
    version = None
    model_id = None
    nickname = None
    endpoint_profile = None

    def __init__(self, dev, tx, rx, endpoint_profile):
        self._dev = dev
        self._tx = tx
        self._rx = rx

        self.uuid = UUID(hex=endpoint_profile["uuid"])
        self.serial = endpoint_profile["serial"]
        self.version = StrictVersion(endpoint_profile["version"])
        self.model_id = endpoint_profile["model"]
        self.nickname = endpoint_profile["nickname"]
        self.endpoint_profile = endpoint_profile
        self.addr = dev.address

    def close(self):
        if self._dev:
            usb.util.dispose_resources(self._dev)
            self._dev = self._tx = self._rx = None

    def __del__(self):
        self.close()


class USBProtocol1(USBProtocol):
    channels = None
    running = False
    session = None
    device_status = None
    timestamp = 0
    _flag = 0
    _buf = b""

    _enable_ping = False
    _wait_ping = False

    def __init__(self, dev, tx, rx, endpoint_profile):
        super(USBProtocol1, self).__init__(dev, tx, rx, endpoint_profile)

        self._enable_ping = self.version >= StrictVersion("1.6.3")
        self.tx_mutex = Lock()
        self.chl_semaphore = Semaphore(0)
        self.chl_open_mutex = Lock()
        self.channels = {}
        self.device_status = {}
        self._flag = 1

    def _send(self, buf):
        # Low level send
        try:
            l = len(buf)
            with self.tx_mutex:
                ret = self._tx.write(buf[:512])
                while ret < l:
                    ret += self._tx.write(buf[ret:ret + 512])

        except usb.core.USBError as e:
            self.close()
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
                self.close()
                raise FluxUSBError(*e.args)

    def _feed_buffer(self, timeout=50):
        self._buf += self._recv(512, timeout)

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

    def stop(self):
        self._flag &= ~2

    def close(self):
        self._flag = 0
        while self.channels:
            idx, channel = self.channels.popitem()
            channel.close(directly=True)
        super(USBProtocol1, self).close()

    def ping(self):
        if self._enable_ping is False:
            return

        if self._wait_ping:
            raise FluxUSBError("PING/PONG timeout")
        else:
            self.timestamp = time()
            self._wait_ping = True
            self._send(CTRL_PACKER.pack(4, 0xfa, 0x00))

    def send_object(self, chl_idx, obj):
        data = msgpack.packb(obj)
        l = len(data) + 4
        buf = b"".join((HEAD_PACKER.pack(l, chl_idx), data, b"\xb0"))
        self._send(buf)

    def send_binary(self, chl_idx, data):
        l = len(data) + 4
        buf = b"".join((HEAD_PACKER.pack(l, chl_idx), data, b"\xbf"))
        self._send(buf)

    def _on_pong(self, buf):
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


class USBProtocol2(USBProtocol1):
    def __init__(self, dev, tx, rx, endpoint_profile):
        super(USBProtocol2, self).__init__(dev, tx, rx, endpoint_profile)
        self.tx_semaphore = Semaphore(8)
        self.tx_mutex2 = Lock()
        self._local_queue = deque()
        self._local_idx, self._remote_idx = 0, 0

    def _unpack_buffer(self):
        l = len(self._buf)
        if l > 6:
            size, seq, chl_idx, fin = HEAD_V2_PACKER.unpack(self._buf[:6])
            if size < 6:
                raise FluxUSBError("Recv bad usb message size: %i" % size)
            elif l >= size:
                buf = self._buf[6:size]
                self._buf = self._buf[size:]
                return seq, chl_idx, fin, buf
        return None, None, None, None

    def run(self):
        last_ping = -1
        try:
            self._flag |= 2
            while self._flag == 3:
                self.run_once()
                if time() - last_ping > 3.0:
                    last_ping = time()
                    self.ping()

                if time() - self.timestamp > 1.4 and self._local_queue:
                    logger.warning("Resend usb data")
                    for idx, ack, buf in self._local_queue:
                        self._send(buf)
                    self.timestamp = time()

        except FluxUSBError as e:
            logger.error("USB Error: %s", e)
            self._flag = 0
            self.close()
        except Exception:
            logger.exception("Unknown error")
            self._flag = 0
            self.close()

    def run_once(self):
        self._feed_buffer()
        seq, channel_idx, fin, buf = self._unpack_buffer()

        if seq is None:
            return
        elif channel_idx == 0xf2:
            # Remote send ack
            while self._local_queue:
                if self._local_queue[0][0] <= seq or \
                        seq < 10000 and self._local_queue[0][0] > 50000:
                    _, ack, _ = self._local_queue.popleft()
                    if ack is not None:
                        channel = self.channels.get(ack)
                        if channel:
                            channel.on_binary_ack()
                    self.tx_semaphore.release()
                else:
                    break
            return

        if seq != self._remote_idx:  # index not match
            logger.debug("Remote seq error, Drop. (%s != %s)", seq, self._remote_idx)
            if seq > self._remote_idx or (seq > 65000 and self._remote_idx < 100):
                self._send_ack()
            return
        else:
            self._send(HEAD_V2_PACKER.pack(6, self._remote_idx, 0xf2, 0))
            self._remote_idx = (self._remote_idx + 1) % 65536

        if channel_idx < 0x80:
            channel = self.channels.get(channel_idx)
            if channel is None:
                raise FluxUSBError("Recv bad channel idx 0x%02x" % channel_idx)
            if fin == 0:
                channel.on_object(msgpack.unpackb(buf, encoding="utf8",
                                  unicode_errors="ignore"))
            elif fin == 1:
                channel.on_binary(buf)
            else:
                raise FluxUSBError("Recv bad fin 0x%02x" % (fin))
        elif channel_idx == 0xf1:
            if fin == 0:
                self._on_channel_ctrl_response(msgpack.unpackb(buf))
            else:
                raise FluxUSBError("Recv bad fin 0x%02x" % (fin))
        elif channel_idx == 0xfb:
            self._on_pong(buf)
        else:
            raise FluxUSBError("Recv bad control channel 0x%02x" % channel_idx)

    def _send_ack(self):
        if self._remote_idx == 0:
            self._send(HEAD_V2_PACKER.pack(6, 65535, 0xf2, 0))
        else:
            self._send(HEAD_V2_PACKER.pack(6, self._remote_idx - 1, 0xf2, 0))

    def send_object(self, chl_idx, obj):
        data = msgpack.packb(obj)
        l = len(data) + 6
        if l > 512:
            raise RuntimeError("Payload size overlimit")
        with self.tx_mutex2:
            buf = HEAD_V2_PACKER.pack(l, self._local_idx, chl_idx, 0) + data
            if self.tx_semaphore.acquire(timeout=15):
                if not self._local_queue:
                    self.timestamp = time()
                self._local_queue.append((self._local_idx, None, buf))
                self._local_idx = (self._local_idx + 1) % 65535
                self._send(buf)
            else:
                raise FluxUSBError("Transmit timeout")

    def send_binary(self, chl_idx, data):
        l = len(data) + 6
        if l > 512:
            raise RuntimeError("Payload size overlimit")
        with self.tx_mutex2:
            buf = HEAD_V2_PACKER.pack(l, self._local_idx, chl_idx, 1) + data
            if self.tx_semaphore.acquire(timeout=15):
                if not self._local_queue:
                    self.timestamp = time()
                self._local_queue.append((self._local_idx, chl_idx, buf))
                self._local_idx = (self._local_idx + 1) % 65535
                self._send(buf)
            else:
                raise FluxUSBError("Transmit timeout")

    def ping(self):
        if self.tx_mutex2.acquire(False):
            try:
                buf = HEAD_V2_PACKER.pack(6, self._local_idx, 0xfa, 1)
                if self.tx_semaphore.acquire(blocking=0):
                    if not self._local_queue:
                        self.timestamp = time()
                    self._local_queue.append((self._local_idx, None, buf))
                    self._local_idx = (self._local_idx + 1) % 65535
                    self._send(buf)
            finally:
                self.tx_mutex2.release()


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

    def get_buffer(self, timeout=20.0):
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
