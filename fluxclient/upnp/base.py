
from random import randint
from select import select
from time import time
import struct
import socket
import json

from fluxclient.utils.version import StrictVersion
from fluxclient.upnp.discover import UpnpDiscover
from fluxclient.encryptor import KeyObject


class UpnpBase(object):
    _access_id = None
    remote_addr = "239.255.255.250"

    def __init__(self, uuid, client_key=None,
                 remote_profile=None, lookup_callback=None,
                 lookup_timeout=float("INF")):
        self.uuid = uuid
        self.keyobj = client_key if client_key else \
            KeyObject.get_or_create_keyobj()

        if remote_profile:
            self.update_remote_profile(**remote_profile)
        else:
            self.reload_remote_profile(lookup_callback, lookup_timeout)

        if self.remote_version < StrictVersion("1.0b5"):
            raise RuntimeError("FLUXMONITOR_VERSION_IS_TOO_OLD")
        elif self.remote_version >= StrictVersion("2.0a0"):
            raise RuntimeError("FLUXMONITOR_VERSION_IS_TOO_NEW")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                                  socket.IPPROTO_UDP)

    def reload_remote_profile(self, lookup_callback=None,
                              lookup_timeout=float("INF")):
        self._inited = False

        d = UpnpDiscover(uuid=self.uuid)
        d.discover(self._load_profile, lookup_callback, lookup_timeout)

        if not self._inited:
            raise RuntimeError("Can not find device")

    def _load_profile(self, discover_instance, **kw):
        self.update_remote_profile(**kw)
        discover_instance.stop()

    def update_remote_profile(self, name, serial, model_id, timedelta, version,
                              has_password, ipaddr, master_key, slave_key,
                              **kw):
        self.name = name
        self.serial = serial
        self.model_id = model_id
        self.timedelta = timedelta
        self.remote_version = StrictVersion(version)
        self.has_password = has_password
        self.endpoint = ipaddr
        self.master_key = master_key
        self.slave_key = slave_key
        self._inited = True

    @property
    def publickey_der(self):
        return self.keyobj.public_key_der

    @property
    def access_id(self):
        if not self._access_id:
            self._access_id = self.keyobj.get_access_id(binary=True)
        return self._access_id

    def create_timestemp(self):
        return time() + self.timedelta

    def make_request(self, req_code, resp_code, message, encrypt=True,
                     timeout=1.2):
        if message and encrypt:
            message = self.slave_key.encrypt(message)

        payload = struct.pack("<4sBB16s", b"FLUX", 1, req_code,
                              self.uuid.bytes) + message
        self.sock.sendto(payload, (self.endpoint))

        while select((self.sock, ), (), (), timeout)[0]:
            resp = self._parse_response(self.sock.recv(4096), resp_code)
            if resp:
                return resp

    def sign_request(self, body):
        salt = ("%i" % randint(1000, 9999)).encode()
        ts = self.create_timestemp()
        message = struct.pack("<20sd4s", self.access_id, ts, salt) + body
        signature = self.keyobj.sign(self.uuid.bytes + message)
        return message + signature

    def _parse_response(self, buf, resp_code):
        if len(buf) < 24:
            return

        mn, proto_ver, verb, buuid, l = struct.unpack("<4sBB16sH", buf[:24])
        if mn != b"FLUX":
            return

        if proto_ver != 1:
            return

        if verb != resp_code:
            return

        if buuid != self.uuid.bytes:
            return

        body = buf[24:24 + l]
        signature = buf[24 + l:]

        if self.slave_key.verify(body, signature):
            message = body.decode("utf8")
            if message[0] == "E":
                raise RuntimeError(message[1:])
            else:
                return json.loads(message)
