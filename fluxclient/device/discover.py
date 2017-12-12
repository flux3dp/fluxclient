
"""
To find flux devices in the network, `fluxclient.device.DeviceDiscover` class \
provide interface to discover and collect informations continuously.

Basic usage example::

    from fluxclient.device import DeviceDiscover

    def my_callback(discover, device, **kw):
        print("Device '%s' found at %s" % (device.name, device.ipaddr))

        # We find only one printer in this example
        discover.stop()

    d = DeviceDiscover()
    d.discover(my_callback)

In the example, `my_callback` will be called one a device is found or recive \
a device status update. A callback contains two positional arguments, first \
is `DeviceDiscover` instance and second is \
:class:`fluxclient.device.device.Device` instance which it found or been \
updated.
"""

from weakref import proxy
from select import select
from uuid import UUID
from time import time
from io import BytesIO
import platform
import logging
import socket
import struct

from fluxclient.utils.version import StrictVersion
from fluxclient.encryptor import KeyObject
from .device import Device
from .misc import validate_identify

logger = logging.getLogger(__name__)


CODE_DISCOVER = 0x00
CODE_RESPONSE_DISCOVER = CODE_DISCOVER + 1
MULTICAST_IPADDR = "239.255.255.250"
MULTICAST_PORT = 1901


class DeviceDiscover(object):
    """The uuid and device_ipaddr param can limit DeviceDiscover to find \
device with specified uuid or IP address. These params usually be used when \
you want recive specified status continuously.

    :param uuid.UUID uuid: Discover specified uuid of device only
    :param string device_ipaddr: Discover device from specified IP address \
only.
    """

    _break = True

    @staticmethod
    def create_sockets(mcst_ipaddr, mcst_port):
        mreq = struct.pack("4sl", socket.inet_aton(mcst_ipaddr),
                           socket.INADDR_ANY)

        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                          socket.IPPROTO_UDP)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        if platform.system() == "Windows":
            s.bind(("", mcst_port))
            return (s, )

        else:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            s.bind((mcst_ipaddr, mcst_port))

            bsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                                  socket.IPPROTO_UDP)
            bsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            bsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            bsock.bind(("", 1901))

            return (s, bsock)

    def __init__(self, uuid=None, device_ipaddr=None,
                 mcst_ipaddr=MULTICAST_IPADDR, mcst_port=MULTICAST_PORT):
        self.devices = {}

        self.uuid = uuid
        self.device_ipaddr = device_ipaddr

        self.handlers = (BroadcastHelper(self), Version1Helper(self),
                         Version2Helper(self))

        self.socks = self.create_sockets(mcst_ipaddr, mcst_port) + tuple(
            (h.sock for h in self.handlers if hasattr(h, "sock")))

    def poke(self, ipaddr, version=None):
        """
        Sends a special message to destination IP address. And the destination\
 device will send a unicast UDP package back.
        """
        if version:
            self.handlers[version].poke(ipaddr)
        else:
            self.handlers[-1].poke(ipaddr)

    def source_filter(self, uuid, endpoint):
        if self.uuid and self.uuid != uuid:
            return False
        elif self.device_ipaddr and self.device_ipaddr != endpoint[0]:
            return False
        else:
            return True

    def discover(self, callback, lookup_callback=None, timeout=float("INF")):
        """
        Executes discovering task. The callback function has a \
minimal definition::

            def callback(device_discover_instance, device, **kw):
                pass

        * `device_discover_instance` is the instance which calls this method.
        * `device` a `fluxclient.device.device.Device` instance

        :param callable callback: This method will be invoked when a device \
has been found or the computer recived a new status from a device.
        :param float timeout: Maximum waiting time.
        """

        self._break = False
        timeout_at = time() + timeout
        poke_timer = 0

        while not self._break:
            # Poke device to prevent discover not work while device is in
            # different subnet.
            if self.device_ipaddr and time() - poke_timer > 3:
                self.poke(self.device_ipaddr)
                poke_timer = time()

            wait_time = min(timeout_at - time(), 0.5)
            if wait_time < 0.05:
                self.stop()
                break

            self.try_recive(self.socks, callback, wait_time)

            if lookup_callback:
                lookup_callback(self)

    def stop(self):
        """Invoke this function to break discover task

        .. note:: Discover method may still invoke a callback even if user \
        called this method, because the data already in local socket buffer."""

        self._break = True

    def try_recive(self, socks, callback, timeout=1.5):
        for sock in select(socks, (), (), timeout)[0]:
            uuid = self.on_recive(sock)
            if uuid:
                device = self.devices[uuid]
                dataset = device.to_old_dict()
                dataset["device"] = device
                callback(self, **dataset)

    def on_recive(self, sock):
        buf, endpoint = sock.recvfrom(4096)

        if len(buf) < 8:
            # Message too short to be process
            return

        try:
            magic_num, proto_ver, action_id = struct.unpack("4sBB", buf[:6])

            if magic_num != b"FLUX":
                # Bad magic number
                return

            if proto_ver > 2:
                logger.debug("Protocol %i not support", proto_ver)
                return

            ret = self.handlers[proto_ver].handle_message(endpoint, action_id,
                                                          buf[6:])
            return ret
        except struct.error:
            logger.warning("Payload error: %s", repr(buf))
        except Exception:
            logger.exception("Error during process discover payload")

    def add_master_key(self, uuid, serial, master_key, disc_ver):
        if uuid in self.devices:
            device = self.devices[uuid]
            if device.master_key != master_key:
                raise Exception("Device %s got conflict master keys" % device,
                                device.master_key, master_key)
            if device.serial != serial:
                raise Exception("Device %s got conflict master keys" % device,
                                device.serial, serial)
            return device
        else:
            d = Device(uuid, serial, master_key, disc_ver)
            d.update_status()
            self.devices[uuid] = d
            return d

    def get_master_key(self, uuid):
        return self.devices[uuid].master_key


class BroadcastHelper(object):
    def __init__(self, server):
        self.server = proxy(server)

    def handle_message(self, endpoint, mcst_ver, payload):
        if len(payload) == 16:
            uuid = UUID(bytes=payload)

            # if uuid.int == 0, its might be a discover request from other
            # client.
            if uuid.int > 0 and self._need_poke(endpoint[0], uuid):
                self.server.poke(endpoint[0], version=mcst_ver)
        else:
            logger.debug("Broadcast helper can not parse payload: %s",
                         payload)

    def _need_poke(self, ipaddr, uuid):
        device = self.server.devices.get(uuid)
        if device:
            return time() - device.last_update > 3.15
        else:
            return True


class Helper(object):
    def __init__(self, server):
        self.server = proxy(server)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                                  socket.IPPROTO_UDP)
        self.sock.bind(('', 0))

    def fileno(self):
        return self.sock.fileno()

    def poke(self, ipaddr):
        payload = struct.pack("<4sBB16s", b"FLUX", 1, 0,
                              UUID(int=0).bytes)
        self.sock.sendto(payload, (ipaddr, MULTICAST_PORT))


class Version1Helper(Helper):
    def _need_touch(self, uuid, slave_timestamp):
        device = self.server.devices.get(uuid)
        if device and device.slave_timestamp is not None:
            return slave_timestamp > device.slave_timestamp
        else:
            return True

    def handle_message(self, endpoint, action_id, payload):
        if action_id == 0:
            return self._handle_discover(endpoint, payload)
        elif action_id == 3:
            return self._handle_touch(endpoint, payload)
        else:
            logger.error("Can not handle proto_ver=1, action_id=%s", action_id)

    def _handle_discover(self, endpoint, payload):
        args = struct.unpack("<16s10sfHH", payload[:34])
        uuid_bytes, bsn, master_ts = args[:3]
        l_master_pkey, l_signuture = args[3:]
        sn = bsn.decode("ascii")

        uuid = UUID(bytes=uuid_bytes)
        if not self.server.source_filter(uuid, endpoint):
            return

        f = BytesIO(payload[34:])
        masterkey_doc = f.read(l_master_pkey)
        signuture = f.read(l_signuture)
        if not validate_identify(uuid, signuture, serial=sn,
                                 masterkey_doc=masterkey_doc):
            logger.error("Validate identify failed (uuid=%s)", uuid)
            return

        master_pkey = KeyObject.load_keyobj(masterkey_doc)
        uuid = UUID(bytes=uuid_bytes)

        if self._need_touch(uuid, master_ts):
            self.server.add_master_key(uuid, sn, master_pkey, 1)
            payload = struct.pack("<4sBB16s", b"FLUX", 1, 2, uuid.bytes)
            try:
                self.sock.sendto(payload, endpoint)
            except Exception:
                logger.exception("Error while poke %s", endpoint)
        else:
            try:
                stbuf = f.read(64)
                st_ts, st_id, st_prog, st_head, st_err = \
                    struct.unpack("dif16s32s", stbuf)

                head_module = st_head.decode("ascii", "ignore").strip("\x00")
                error_label = st_err.decode("ascii", "ignore").strip("\x00")
                device = self.server.devices[uuid]
                device.update_status(st_id=st_id, st_ts=st_ts, st_prog=st_prog,
                                     head_module=head_module,
                                     error_label=error_label)
                device.discover_endpoint = endpoint
                device.ipaddr = endpoint[0]

                return uuid
            except Exception:
                basic_info = self.server.devices[uuid]
                if basic_info.version > StrictVersion("0.13a"):
                    logger.exception("Unpack status failed")

    def _handle_touch(self, endpoint, payload):
        f = BytesIO(payload)

        buuid, master_ts, l1, l2 = struct.unpack("<16sfHH", f.read(24))
        uuid = UUID(bytes=buuid)

        if not self.server.source_filter(uuid, endpoint):
            # Ingore this uuid
            return

        device = self.server.devices[uuid]

        slavekey_str = f.read(l1)
        slavekey_signuture = f.read(l2)
        temp_pkey = KeyObject.load_keyobj(slavekey_str)

        bmeta = f.read(struct.unpack("<H", f.read(2))[0])
        smeta = bmeta.decode("utf8")
        rawdata = {}
        for item in smeta.split("\x00"):
            if "=" in item:
                k, v = item.split("=", 1)
                rawdata[k] = v

        doc_signuture = f.read()
        master_key = self.server.get_master_key(uuid)

        if master_key.verify(payload[16:20] + slavekey_str,
                             slavekey_signuture):
            if temp_pkey.verify(bmeta, doc_signuture):
                device.slave_timestamp = master_ts
                device.slave_key = temp_pkey
                device.has_password = rawdata.get("pwd") == "T"
                device.timestamp = float(rawdata.get("time", 0))
                device.timedelta = device.timestamp - time()

                device.model_id = rawdata.get("model", "UNKNOW_MODEL")
                device.version = StrictVersion(rawdata["ver"])
                device.name = rawdata.get("name", "NONAME")

                device.discover_endpoint = endpoint
                device.ipaddr = endpoint[0]

                return uuid
            else:
                logger.error("Slave key signuture error (V1)")
        else:
            logger.error("Master key signuture error (V1)")


class Version2Helper(Helper):
    session_cache = None
    session_swap = None

    def __init__(self, server):
        super(Version2Helper, self).__init__(server)
        self.session_cache = {}
        self.session_swap = {}

    def _need_touch(self, uuid, session):
        device = self.server.devices.get(uuid)
        if device and uuid in self.session_cache and \
                self.session_cache[uuid] == session:
            return False
        else:
            return True

    def handle_message(self, endpoint, action_id, payload):
        if action_id == 0:
            return self._handle_discover(endpoint, payload)
        elif action_id == 3:
            return self._handle_touch(endpoint, payload)
        else:
            logger.error("Can not handle proto_ver=1, action_id=%s", action_id)

    def _handle_discover(self, endpoint, payload):
        args = struct.unpack("<16s8s", payload[:24])
        uuid_bytes, session = args[:2]

        uuid = UUID(bytes=uuid_bytes)
        if not self.server.source_filter(uuid, endpoint):
            return

        if self._need_touch(uuid, session):
            payload = struct.pack("<4sBB16s", b"FLUX", 2, 2, uuid.bytes)
            try:
                self.session_swap[uuid] = session
                self.sock.sendto(payload, endpoint)
            except Exception:
                logger.exception("Error while poke %s", endpoint)
        else:
            try:
                st_ts, st_id, st_prog, st_head, st_err = \
                    struct.unpack("dif16s32s", payload[24:88])

                head_module = st_head.decode("ascii", "ignore").strip("\x00")
                error_label = st_err.decode("ascii", "ignore").strip("\x00")
                device = self.server.devices[uuid]
                device.update_status(st_id=st_id, st_ts=st_ts, st_prog=st_prog,
                                     head_module=head_module,
                                     error_label=error_label)
                device.discover_endpoint = endpoint
                device.ipaddr = endpoint[0]

                return uuid
            except Exception:
                basic_info = self.server.devices[uuid]
                if basic_info.version > StrictVersion("0.13a"):
                    logger.exception("Unpack status failed")

    def _handle_touch(self, endpoint, payload):
        f = BytesIO(payload)

        buuid, l1, l2 = struct.unpack("<16sHH", f.read(20))
        uuid = UUID(bytes=buuid)

        if not self.server.source_filter(uuid, endpoint):
            # Ingore this uuid
            return

        pubkey_der = f.read(l1)
        pubkey_signuture = f.read(l2)
        dev_pubkey = KeyObject.load_keyobj(pubkey_der)

        bmeta = f.read(struct.unpack("<H", f.read(2))[0])
        smeta = bmeta.decode("utf8")
        rawdata = {}
        for item in smeta.split("\x00"):
            if "=" in item:
                k, v = item.split("=", 1)
                rawdata[k] = v

        sn = rawdata.get("serial", None)
        if sn and uuid in self.session_swap and \
                validate_identify(uuid, pubkey_signuture, serial=sn, masterkey_doc=pubkey_der):
            device = self.server.add_master_key(uuid, sn, dev_pubkey, 2)
            device.model_id = rawdata.get("model", "UNKNOW_MODEL")
            device.has_password = rawdata.get("pwd") == "T"
            device.version = StrictVersion(rawdata["ver"])
            device.name = rawdata.get("name", "NONAME")
            device.discover_endpoint = endpoint
            device.ipaddr = endpoint[0]

            self.session_cache[uuid] = self.session_swap.pop(uuid)
            return uuid
        else:
            logger.error("Validate identify failed (uuid=%s, serial=%s)",
                         uuid, sn)
