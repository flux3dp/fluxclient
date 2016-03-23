
from io import BytesIO
from uuid import UUID
from time import time
import platform
import logging
import select
import socket
import struct

from .misc import DEFAULT_IPADDR, DEFAULT_PORT, validate_identify
from fluxclient.encryptor import KeyObject

logger = logging.getLogger(__name__)


CODE_DISCOVER = 0x00
CODE_RESPONSE_DISCOVER = CODE_DISCOVER + 1
MULTICAST_VERSION = 1


"""Discover Flux 3D Printer

Here is a simple example:

from fluxclient.upnp_discover import UpnpDiscover

def my_callback(discover, serial, model_id, timestemp, version,
                has_passwd, ipaddrs):
    print("Find Printer at: " + ipaddrs)

    # We find only one printer in this example
    discover.stop()


d = UpnpDiscover()
d.discover(my_callback)
"""

INIT_PING_FREQ = 0.5
PING_RREQ_RATIO = 1.3
MAX_PING_FREQ = 3.0


class UpnpDiscover(object):
    _break = True
    _last_sent = 0
    _send_freq = INIT_PING_FREQ

    def __init__(self, uuid=None, ipaddr=DEFAULT_IPADDR, port=DEFAULT_PORT):
        self.history = {}

        self.ipaddr = ipaddr
        self.port = port
        self.uuid = uuid

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                                  socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        mreq = struct.pack("4sl", socket.inet_aton(DEFAULT_IPADDR),
                           socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                             mreq)

        if platform.system() == "Windows":
            self.sock.bind(("", self.port))
        else:
            self.sock.bind((DEFAULT_IPADDR, self.port))

        self.handlers = (None, Version1Helper(self))

        self.socks = (self.sock, ) + tuple(
            (h.sock for h in self.handlers if hasattr(h, "sock")))

    def poke(self, ipaddr):
        # TODO
        self.handlers[-1].poke(ipaddr)

    def limited_uuid(self, uuid):
        if self.uuid:
            return self.uuid == uuid
        else:
            return True

    def discover(self, callback, lookup_callback=None, timeout=float("INF")):
        """
        Call this method to execute discover task

        @callback: when find a flux printer, it will invoke
        `callback(instance, serial, model_id, timestemp, version,
                     has_passwd, ipaddrs)` where ipaddrs is a list.
        """
        self._break = False
        timeout_at = time() + timeout

        while not self._break:
            wait_time = min(timeout_at - time(), 0.5)
            if wait_time < 0.05:
                self.stop()
                break

            self.try_recive(self.socks, callback, wait_time)

            if lookup_callback:
                lookup_callback(self)

    def stop(self):
        """Call this function to break discover task"""
        self._break = True

    def try_recive(self, socks, callback, timeout=1.5):
        timeout_at = time() + timeout

        while timeout > 0:
            for sock in select.select(socks, (), (), timeout)[0]:
                uuid = self.on_recive(sock)
                if uuid:
                    data = self.history[uuid]
                    callback(self, uuid=uuid, **data)

            timeout = timeout_at - time()

    def on_recive(self, sock):
        buf, endpoint = sock.recvfrom(4096)
        if len(buf) < 8:
            # Message too short to be process
            return

        magic_num, proto_ver, action_id = struct.unpack("4sBB", buf[:6])

        if magic_num != b"FLUX":
            # Bad magic number
            return

        # TODO: err handle
        ret = self.handlers[proto_ver].handle_message(endpoint, action_id,
                                                      buf[6:])
        return ret

    def in_history(self, uuid, master_ts):
        if uuid in self.history:
            return master_ts <= self.history[uuid].get("master_ts", 0)
        else:
            return False

    def update_history(self, uuid, dataset):
        if "master_key" in dataset:
            raise RuntimeError("Can not update master key")
        self.history[uuid].update(dataset)

    def add_master_key(self, uuid, sn, master_key):
        if uuid in self.history:
            logger.warning("Master already in list: %s", uuid)
        else:
            self.history[uuid] = {"master_key": master_key, "serial": sn}

    def get_master_key(self, uuid):
        return self.history[uuid]["master_key"]

    def get_serial(self, uuid):
        return self.history[uuid]["serial"]


class Version1Helper(object):
    def __init__(self, server):
        self.server = server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                                  socket.IPPROTO_UDP)
        self.sock.bind(('', 0))

    def fileno(self):
        return self.sock.fileno()

    def poke(self, ipaddr):
        payload = struct.pack("<4sBB16s", b"FLUX", MULTICAST_VERSION, 0,
                              UUID(int=0).bytes)
        self.sock.sendto(payload, (ipaddr, DEFAULT_PORT))

    def handle_message(self, endpoint, action_id, payload):
        if action_id == 0:
            return self.handle_discover(endpoint, payload)
        elif action_id == 3:
            return self.handle_touch(endpoint, payload)
        else:
            logger.error("Can not handle proto_ver=1, action_id=%s", action_id)

    def handle_discover(self, endpoint, payload):
        args = struct.unpack("<16s10sfHH", payload[:34])
        uuid_bytes, sn, master_ts = args[:3]
        l_master_pkey, l_signuture = args[3:]

        uuid = UUID(bytes=uuid_bytes)
        if not self.server.limited_uuid(uuid):
            return

        f = BytesIO(payload[34:])
        masterkey_doc = f.read(l_master_pkey)
        signuture = f.read(l_signuture)
        if not validate_identify(uuid, signuture, serial=sn.decode("ascii"),
                                 masterkey_doc=masterkey_doc):
            logger.error("Validate identify failed (uuid=%s)", uuid)
            return

        master_pkey = KeyObject.load_keyobj(masterkey_doc)
        uuid = UUID(bytes=uuid_bytes)

        if self.server.limited_uuid(uuid):
            if self.server.in_history(uuid, master_ts):
                try:
                    stbuf = f.read(64)
                    st_ts, st_id, st_prog, st_head, st_err = \
                        struct.unpack("dif16s32s", stbuf)

                    head_module = st_head.decode("ascii",
                                                 "ignore").strip("\x00")
                    error_label = st_err.decode("ascii",
                                                "ignore").strip("\x00")
                    dataset = self.server.history[uuid]
                    dataset.update({
                        "st_id": st_id, "st_ts": st_ts, "st_prog": st_prog,
                        "st_ts": st_ts, "head_module": head_module,
                        "error_label": error_label})
                    return uuid
                except Exception:
                    basic_info = self.server.history[uuid]
                    if basic_info["version"] > "0.13a":
                        logger.exception("Unpack status failed")
            else:
                self.server.add_master_key(uuid, sn.decode("ascii"),
                                           master_pkey)
                payload = struct.pack("<4sBB16s", b"FLUX", MULTICAST_VERSION,
                                      2, uuid.bytes)
                self.sock.sendto(payload, endpoint)

    def handle_touch(self, endpoint, payload):
        f = BytesIO(payload)

        buuid, master_ts, l1, l2 = struct.unpack("<16sfHH", f.read(24))
        uuid = UUID(bytes=buuid)

        if not self.server.limited_uuid(uuid):
            # Ingore this uuid
            return

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
                data = {}
                data["slave_key"] = temp_pkey
                data["master_ts"] = master_ts

                data["model_id"] = rawdata.get("model", "UNKNOW")
                data["version"] = rawdata.get("ver")
                raw_has_password = rawdata.get("pwd", "F")

                data["timestemp"] = float(rawdata.get("time", 0))
                data["timedelta"] = data["timestemp"] - time()

                data["name"] = rawdata.get("name", "NONAME")
                data["has_password"] = raw_has_password == "T"
                data["ipaddr"] = endpoint
                self.server.update_history(uuid, data)

                return uuid
            else:
                logger.error("Slave key signuture error (V1)")
        else:
            logger.error("Master key signuture error (V1)")
