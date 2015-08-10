
from collections import namedtuple
from time import time, sleep
import uuid as _uuid
import logging
import select
import socket
import struct
import json

logger = logging.getLogger(__name__)


CODE_DISCOVER = 0x00
CODE_RESPONSE_DISCOVER = CODE_DISCOVER + 1

DEFAULT_PORT = 3310


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

    def __init__(self, serial=GLOBAL_SERIAL, ipaddr="255.255.255.255",
                 port=DEFAULT_PORT):
        self.ipaddr = ipaddr
        self.serial = serial
        self.port = port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                                  socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

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

            self.ping()
            self._receiving_pong(self.sock, callback, wait_time)

            if lookup_callback:
                lookup_callback(self)


    def stop(self):
        """Call this function to break discover task"""
        self._break = True

    def ping(self):
        if time() - self._last_sent > self._send_freq:
            payload = struct.pack('<4s16sB', b"FLUX",
                                  self.serial.bytes,
                                  CODE_DISCOVER)

            self.sock.sendto(payload, (self.ipaddr, self.port))
            self._last_sent = time()

            self._send_freq = min(self._send_freq * PING_RREQ_RATIO,
                                  MAX_PING_FREQ)

    def on_recv_pong(self):
        buf, remote = self.sock.recvfrom(4096)
        data = self._parse_response(buf)
        return data

    def _receiving_pong(self, sock, callback, timeout=1.5):
        timeout_at = time() + timeout

        while timeout > 0:
            if select.select((sock, ), (), (), timeout)[0]:
                data = self.on_recv_pong()
                if data:
                    callback(self, **data)

            timeout = timeout_at - time()

    def _parse_response(self, buf):
        resp_code, status = struct.unpack("<BB", buf[:2])

        if resp_code != CODE_RESPONSE_DISCOVER:
            return None

        payload = json.loads(buf[2:-1].decode("utf8"))

        data = {
            "serial": payload.get("serial"),
            "name": payload.get("name", "My FLUX 3D Printer"),
            "version": payload.get("ver"),
            "model_id": payload.get("model"),
            "timestemp": payload.get("time"),
            "ipaddrs": payload.get("ip"),
            "has_password": payload.get("pwd")
        }

        return data
