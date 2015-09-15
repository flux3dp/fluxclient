
from select import select
from random import randint
from time import time
import uuid as _uuid
import struct
import socket
import json

from fluxclient.utils.version import StrictVersion
from fluxclient.upnp.discover import UpnpDiscover
from fluxclient.upnp import misc
from fluxclient import encryptor


class UpnpBase(object):
    remote_addr = "255.255.255.255"

    def __init__(self, serial, ipaddr=None, pubkey=None, lookup_callback=None,
                 port=misc.DEFAULT_PORT, forcus_broadcast=False,
                 lookup_timeout=float("INF")):
        self.port = port

        if len(serial) == 25:
            self.serial = _uuid.UUID(hex=misc.short_to_uuid(serial))
        else:
            self.serial = _uuid.UUID(hex=serial)

        self.keyobj = encryptor.get_or_create_keyobj()
        self._inited = False

        if ipaddr:
            d = UpnpDiscover(serial=self.serial, ipaddr=ipaddr)
        else:
            d = UpnpDiscover(serial=self.serial)

        d.discover(self._load_profile, lookup_callback, lookup_timeout)
        if not self._inited:
            raise RuntimeError("Can not find device")

        if not forcus_broadcast:
            for ipaddr in self.remote_addrs:
                d.ipaddr = ipaddr[0]
                d.discover(self._ensure_remote_ipaddr, timeout=1.5)

        if self.remote_version < StrictVersion("0.10a1"):
            raise RuntimeError("fluxmonitor version is too old")
        elif self.remote_version >= StrictVersion("0.12"):
            raise RuntimeError("fluxmonitor version is too new")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                                  socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        if not pubkey:
            pubkey = self.fetch_publickey()
        self.pubkey = pubkey
        self.remote_keyobj = encryptor.load_keyobj(pubkey)

    def create_timestemp(self):
        return time() + self.timedelta

    @property
    def publickey_der(self):
        return encryptor.get_public_key_der(self.keyobj)

    def _load_profile(self, discover_instance, serial, model_id, timestemp,
                      version, name, has_password, ipaddrs):
        if serial == self.serial.hex:
            self.name = name
            self.model_id = model_id
            self.timedelta = timestemp - time()
            self.remote_version = StrictVersion(version)
            self.has_password = has_password
            self.remote_addrs = ipaddrs
            self._inited = True
            discover_instance.stop()

    def _ensure_remote_ipaddr(self, discover_instance, serial, model_id,
                              timestemp, version, has_password,
                              ipaddrs, **kw):
        if serial == self.serial.hex:
            self.remote_addr = discover_instance.ipaddr
            discover_instance.stop()

    def fetch_publickey(self, retry=3):
        resp = self.make_request(misc.CODE_RSA_KEY,
                                 misc.CODE_RESPONSE_RSA_KEY, b"")
        if resp:
            return resp
        else:
            if retry > 0:
                return self.fetch_publickey(retry - 1)
            else:
                raise RuntimeError("Remote did not return public key")

    def make_request(self, req_code, resp_code, message, encrypt=True,
                     timeout=1.2):
        if message and encrypt:
            message = encryptor.rsa_encrypt(self.remote_keyobj, message)

        payload = struct.pack('<4s16sB', b"FLUX", self.serial.bytes,
                              req_code) + message

        self.sock.sendto(payload, (self.remote_addr, self.port))

        while select((self.sock, ), (), (), timeout)[0]:
            resp = self._parse_response(self.sock.recv(4096), resp_code)
            if resp:
                return resp

    def sign_request(self, body):
        salt = ("%i" % randint(1000, 9999)).encode()
        ts = self.create_timestemp()
        message = struct.pack("<20sd4s", self.access_id, ts, salt) + body
        signature = encryptor.sign(self.keyobj,
                                   self.serial.bytes + message)

        return message + signature

    def _parse_response(self, buf, resp_code):
        payload, signature = buf[2:].split(b"\x00", 1)

        code, status = struct.unpack("<BB", buf[:2])
        if code != resp_code:
            return

        if status != 0:
            raise RuntimeError(payload.decode("utf8"))

        resp = json.loads(payload.decode("utf8"))
        if resp_code == misc.CODE_RESPONSE_RSA_KEY:
            remote_keyobj = encryptor.load_keyobj(resp)
            if encryptor.validate_signature(remote_keyobj, payload,
                                            signature):
                return resp
        else:
            if encryptor.validate_signature(self.remote_keyobj, payload,
                                            signature):
                return resp
