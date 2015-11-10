
from time import time, sleep
from io import BytesIO
from uuid import UUID
import logging
import select
import socket
import struct
import json

logger = logging.getLogger(__name__)


CODE_DISCOVER = 0x00
CODE_RESPONSE_DISCOVER = CODE_DISCOVER + 1
MULTICAST_VERSION = 1

from fluxclient import encryptor as E
from .misc import DEFAULT_IPADDR, DEFAULT_PORT


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

        self.disc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                                       socket.IPPROTO_UDP)
        self.disc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.disc_sock.bind((DEFAULT_IPADDR, self.port))
        mreq = struct.pack("4sl", socket.inet_aton(DEFAULT_IPADDR),
                           socket.INADDR_ANY)
        self.disc_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                  mreq)

        self.touch_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                                        socket.IPPROTO_UDP)
        self.touch_sock.bind(('', 0))

    def __del__(self):
        self.disc_sock.close()
        self.disc_sock = None
        self.touch_sock.close()
        self.touch_sock = None

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

            self.try_recive((self.disc_sock, self.touch_sock), callback,
                             wait_time)

            if lookup_callback:
                lookup_callback(self)

    def stop(self):
        """Call this function to break discover task"""
        self._break = True

    def try_recive(self, socks, callback, timeout=1.5):
        timeout_at = time() + timeout

        while timeout > 0:
            for sock in select.select(socks, (), (), timeout)[0]:
                data = self.on_recive(sock)
                if data:
                    callback(self, **data)

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

        if proto_ver == 1:
            if action_id == 0:
                self.unpack_v1_discover(buf[6:], endpoint)
            elif action_id == 3:
                return self.process_v1_touch(buf[6:], endpoint)
        else:
            # Can not handle protocol version
            return

    def add_master_key(self, uuid, sn, master_key):
        if uuid in self.history:
            self.history[uuid]["master_key"] = master_key
            self.history[uuid]["serial"] = sn
        else:
            self.history[uuid] = {"master_key": master_key,
                                  "serial": sn}

    def get_master_key(self, uuid):
        return self.history[uuid]["master_key"]

    def get_serial(self, uuid):
        return self.history[uuid]["serial"]

    def in_history(self, uuid, ts):
        if uuid in self.history:
            return ts <= self.history[uuid].get("temp_ts", 0)
        else:
            return False

    def unpack_v1_discover(self, payload, endpoint):
        args = struct.unpack("<16s10sfHH", payload[:34])
        uuid_bytes, sn, temp_ts = args[:3]
        l_master_pkey, l_identify = args[3:]

        f = BytesIO(payload[34:])
        try:
            master_pkey = E.load_keyobj(f.read(l_master_pkey))
            identify = f.read(l_identify)

        except ValueError:
            # Data error
            return

        uuid = UUID(bytes=uuid_bytes)
        if self.limited_uuid(uuid):
            if not self.in_history(uuid, temp_ts):
                self.add_master_key(uuid, sn.decode("ascii"), master_pkey)
                self.touch_v1_device(uuid, endpoint)

    def touch_v1_device(self, uuid, endpoint):
        payload = struct.pack("<4sBB16s", b"FLUX", MULTICAST_VERSION,
            2, uuid.bytes)
        self.touch_sock.sendto(payload, endpoint)

    def process_v1_touch(self, payload, endpoint):
        f = BytesIO(payload)

        buuid, temp_ts, l1, l2 = struct.unpack("<16sfHH", f.read(24))
        uuid = UUID(bytes=buuid)

        if not self.limited_uuid(uuid):
            # Ingore this uuid
            return

        try:
            temp_pkey_str = f.read(l1)
            temp_pkey = E.load_keyobj(temp_pkey_str)
            temp_pkey_ca = f.read(l2)

            bmeta = f.read(struct.unpack("<H", f.read(2))[0])
            signuture = f.read()

            master_key = self.get_master_key(uuid)
            if E.validate_signature(master_key,
                                    payload[16:20] + temp_pkey_str,
                                    temp_pkey_ca):
                if E.validate_signature(temp_pkey, bmeta, signuture):
                    meta_str = bmeta.decode("utf8")
                    rawdata = {}
                    for item in meta_str.split("\x00"):
                        if "=" in item:
                            k, v = item.split("=", 1)
                            rawdata[k] = v

                    data = {"uuid": uuid, "serial": self.get_serial(uuid)}
                    data["model_id"] = rawdata.get("model", "UNKNOW")
                    data["name"] = rawdata.get("name", "NONAME")
                    data["timestemp"] = float(rawdata.get("time", 0))
                    data["version"] = rawdata.get("ver")
                    raw_has_password = rawdata.get("has_password", "F")
                    data["has_password"] = raw_has_password == "T"
                    data["master_key"] = master_key
                    data["slave_key"] = temp_pkey
                    data["ipaddr"] = endpoint
                    return data
                else:
                    logger.error("Slave key signuture error (V1)")
            else:
                logger.error("Master key signuture error (V1)")
        except Exception:
            logger.exception("Unhandle Error")
