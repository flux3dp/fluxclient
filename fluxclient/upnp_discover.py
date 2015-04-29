
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

def my_callback(discover, model, id, ipaddss):
    print("Find Printer at: " + ipaddrs)

    # We find only one printer in this example
    discover.stop()


d = UpnpDiscover()
d.discover(my_callback)
"""

GLOBAL_SERIAL = _uuid.UUID(int=0)


class UpnpDiscover(object):
    _last_sent = 0
    _break = True

    def __init__(self, serial=GLOBAL_SERIAL, ipaddr="255.255.255.255",
                 port=DEFAULT_PORT):
        self.ipaddr = ipaddr
        self.serial = serial
        self.port = port

    def discover(self, callback, timeout=3.0):
        """
        Call this method to execute discover task

        @callback: when find a flux printer, it will invoke
        `callback(instance, model, id, ipaddrs)` where ipaddrs is a list.
        """
        self._break = False
        timeout_at = time() + timeout

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                             socket.IPPROTO_UDP)

        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

            while not self._break:
                self._send_request(sock)
                self._recv_response(sock, callback)
                self._sleep_or_quit(timeout_at)

        finally:
            sock.close()

    def stop(self):
        """Call this function to break discover task"""
        self._break = True

    def _send_request(self, sock):
        now = time()

        if now - self._last_sent > 0.1:
            payload = struct.pack('<4s16sB', b"FLUX",
                                  self.serial.bytes,
                                  CODE_DISCOVER)

            sock.sendto(payload, (self.ipaddr, self.port))
            self._last_sent = time()

    def _recv_response(self, sock, callback):
        while self._has_response(sock):
            if self._break:
                return

            buf, remote = sock.recvfrom(4096)
            resp_code, status = struct.unpack("<BB", buf[:2])

            if resp_code != CODE_RESPONSE_DISCOVER:
                continue

            payload = json.loads(buf[2:-1].decode("utf8"))

            serial = payload.get("serial")
            version = payload.get("ver")
            model_id = payload.get("model")
            timestemp = payload.get("time")
            ipaddrs = payload.get("ip")
            has_passwd = payload.get("pwd")

            callback(self, serial, model_id, timestemp, version,
                     has_passwd, ipaddrs)

    def _sleep_or_quit(self, timeout_at):
        time_left = timeout_at - time()
        if time_left > 0:
            sleep(min(time_left, 0.3))
        else:
            self.stop()

    def _has_response(self, sock):
        if select.select((sock, ), (), (), 0)[0]:
            return True
        else:
            return False
