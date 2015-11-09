
from time import time, sleep
from io import BytesIO
import uuid as _uuid
import logging
import select
import socket
import struct
import json

logger = logging.getLogger(__name__)


CODE_DISCOVER = 0x00
CODE_RESPONSE_DISCOVER = CODE_DISCOVER + 1

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

GLOBAL_SERIAL = _uuid.UUID(int=0)
INIT_PING_FREQ = 0.5
PING_RREQ_RATIO = 1.3
MAX_PING_FREQ = 3.0


class UpnpDiscover(object):
    _break = True
    _last_sent = 0
    _send_freq = INIT_PING_FREQ

    def __init__(self, serial=GLOBAL_SERIAL, ipaddr=DEFAULT_IPADDR,
                 port=DEFAULT_PORT):
        self.serial = serial
        self.ipaddr = ipaddr
        self.port = port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                                  socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.port))
        mreq = struct.pack("4sl", socket.inet_aton(DEFAULT_IPADDR),
                           socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    def __del__(self):
        self.sock.close()
        self.sock = None

    def fileno(self):
        return self.sock.fileno()

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

            self.try_recive(self.sock, callback, wait_time)

            if lookup_callback:
                lookup_callback(self)

    def stop(self):
        """Call this function to break discover task"""
        self._break = True

    def try_recive(self, sock, callback, timeout=1.5):
        timeout_at = time() + timeout

        while timeout > 0:
            if select.select((sock, ), (), (), timeout)[0]:
                data = self._parse_response()
                if data:
                    callback(self, **data)

            timeout = timeout_at - time()

    def _parse_response(self):
        buf, endpoint = self.sock.recvfrom(4096)
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
        else:
            # Can not handle protocol version
            return

    def unpack_v1_discover(self, payload, endpoint):
        args = struct.unpack("<16s10sfHHHH", payload[:38])
        uuid_bytes, sn, temp_ts = args[:3]
        l_master_pkey, l_identify, l_tmp_pkey, l_sign = args[3:]

        f = BytesIO(payload[38:])

        try:
            master_pkey = E.load_keyobj(f.read(l_master_pkey))
            identify = f.read(l_identify)
            temp_pkey = E.load_keyobj(f.read(l_tmp_pkey))
            signature = f.read(l_sign)

            sign_doc = struct.pack("<f", temp_ts) + temp_pkey.exportKey("DER")
            if E.validate_signature(master_pkey, sign_doc, signature):
                print(_uuid.UUID(bytes=uuid_bytes))
                self.sock.sendto(b"FLUX", endpoint)
            else:
                logging.error("signature failed")

        except ValueError:
            # Data error
            return
