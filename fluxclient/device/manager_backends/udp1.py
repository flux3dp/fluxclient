
from random import randint
from select import select
from time import time
import socket
import struct
import json

from fluxclient.utils.version import StrictVersion
from .base import (ManagerAbstractBackend, TimeoutError, AuthError,
                   NotSupportError, ManagerError)

__all__ = ["Udp1Backend"]


CODE_NOPWD_ACCESS = 0x04
CODE_RESPONSE_NOPWD_ACCESS = 0x05

CODE_PWD_ACCESS = 0x06
CODE_RESPONSE_PWD_ACCESS = 0x07

CODE_CHANGE_PWD = 0xa0
CODE_RESPONSE_CHANGE_PWD = 0xa1

CODE_SET_NETWORK = 0xa2
CODE_RESPONSE_SET_NETWORK = 0xa3

SUPPORT_VERSION = (StrictVersion("1.1.0"), StrictVersion("1.1b1"))


class Udp1Backend(ManagerAbstractBackend):
    sock = None
    _access_id = None
    _authorized = False

    @classmethod
    def support_device(cls, model_id, version):
        return version >= SUPPORT_VERSION[0] and version < SUPPORT_VERSION[1]

    def __init__(self, client_key, device):
        # TODO: name -> nickname
        super(Udp1Backend, self).__init__(
            client_key, device.uuid, device.serial, device.model_id,
            device.version, device.name)

        self.endpoint = device.discover_endpoint
        self.timedelta = device.timedelta
        self.master_key = device.master_key
        self.slave_key = device.slave_key

    @property
    def publickey_der(self):
        return self.client_key.public_key_der

    @property
    def access_id(self):
        if not self._access_id:
            self._access_id = self.client_key.get_access_id(binary=True)
        return self._access_id

    def connect(self):
        if self.sock:
            self.close()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                                  socket.IPPROTO_UDP)
        self._authorized = False
        self._try_auth()

    @property
    def connected(self):
        return self.sock is not None

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None
            self._authorized = False

    def create_timestamp(self):
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
        ts = self.create_timestamp()
        message = struct.pack("<20sd4s", self.access_id, ts, salt) + body
        signature = self.client_key.sign(self.uuid.bytes + message)
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
                raise ManagerError(message[1:])
            else:
                return json.loads(message)

    def _try_auth(self, timeout=6):
        start_at = time()
        while timeout >= (time() - start_at):
            resp = self._auth_without_password()
            if resp:
                if resp.get("status") == "ok":
                    self._authorized = True
                    return

                elif resp.get("status") == "deny":
                    return

                elif resp.get("status") == "padding":
                    raise ManagerError(
                        "Can not auth because device does not has password",
                        err_symbol=("NOT_SUPPORT", ))

                else:
                    raise ManagerError(
                        "Unknown status '%s'" % resp.get("status"))

        raise TimeoutError()

    def _auth_without_password(self, timeout=1.2):
        der = self.publickey_der

        req_code = CODE_NOPWD_ACCESS
        resp_code = CODE_RESPONSE_NOPWD_ACCESS
        msg = struct.pack("<d%ss" % len(der), self.create_timestamp(), der)

        resp = self.make_request(req_code, resp_code, msg,
                                 encrypt=False, timeout=timeout)
        return resp

    def authorize_with_password(self, passwd, timeout=1.2):
        der = self.publickey_der

        req_code = CODE_PWD_ACCESS
        resp_code = CODE_RESPONSE_PWD_ACCESS

        buf = b"\x00".join([
            str(self.create_timestamp()).encode(),
            passwd.encode(),
            der
        ])
        resp = self.make_request(req_code, resp_code, buf, encrypt=True)
        if resp and resp.get("status") == "ok":
            self._authorized = True
        else:
            raise AuthError("Bad password")

    def add_trust(self, label, pem):
        raise NotSupportError(self.model_id, self.version)

    def list_trust(self):
        raise NotSupportError(self.model_id, self.version)

    def remove_trust(self, access_id):
        raise NotSupportError(self.model_id, self.version)

    def set_nickname(self, nickname):
        raise NotSupportError(self.model_id, self.version)

    def set_password(self, old_passwd, new_passwd, reset_acl):
        if reset_acl is False:
            raise NotSupportError("Reset ACL can not be false on this version")

        req_code = CODE_CHANGE_PWD
        resp_code = CODE_RESPONSE_CHANGE_PWD

        message = "\x00".join((new_passwd, old_passwd))
        request = self.sign_request(message.encode())

        return self.make_request(req_code, resp_code, request)

    def set_network(self, **network_options):
        req_code = CODE_SET_NETWORK
        resp_code = CODE_RESPONSE_SET_NETWORK

        message = "\x00".join(("%s=%s" % i for i in network_options.items()))
        request = self.sign_request(message.encode())

        return self.make_request(req_code, resp_code, request)

    def get_wifi_list(self):
        raise NotSupportError(self.model_id, self.version)
