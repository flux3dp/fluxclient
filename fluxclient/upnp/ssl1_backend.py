
from binascii import b2a_hex as to_hex
from struct import Struct
from hashlib import sha1
from hmac import HMAC
import socket
import json
import ssl

from fluxclient.utils.version import StrictVersion
from .abstract_backend import (UpnpAbstractBackend, AuthError,
                               NotSupportError, ConnectionBroken, UpnpError, )

__all__ = ["UpnpSSL1Backend"]

SHORT_PACKER = Struct("<H")
SUPPORT_VERSION = (StrictVersion("1.1b1"), StrictVersion("1.2b1"))


def raise_error(ret, **ref):
    if ret.startswith("error ") or ret.startswith("er "):
        errno = ret.split(" ")[1:]
        message = ref.get(errno[0])
        if not message:
            message = "Error: " + " ".join(errno)
        return UpnpError(message, err_symbol=errno)
    else:
        return UpnpError(ret, err_symbol=("UNKNOW_ERROR", ))


def ensure_pair(key, value=None):
    return (key, value)


class UpnpSSL1Backend(UpnpAbstractBackend):
    sock = None
    _access_id = None

    @classmethod
    def support_device(cls, model_id, version):
        return version >= SUPPORT_VERSION[0] and version < SUPPORT_VERSION[1]

    def __init__(self, client_key, uuid, version, model_id, ipaddr,
                 metadata=None, options={}, port=1901):
        super(UpnpSSL1Backend, self).__init__(
            client_key, uuid, version, model_id, ipaddr, metadata, options)

        self.endpoint = (ipaddr, port)
        self.timedelta = metadata["timedelta"]
        self.has_password = metadata["has_password"]
        self.master_key = metadata["master_key"]
        self.options = options

        self.connect()

    @property
    def access_id(self):
        if not self._access_id:
            self._access_id = self.client_key.get_access_id(binary=True)
        return self._access_id

    def recv_bytes(self, length):
        # TODO
        buf = b""

        while(len(buf) < length):
            b = self.sock.recv(length - len(buf))
            if len(b) == 0:
                raise ConnectionBroken()
            buf += b
        return buf

    def recv_text(self):
        size = SHORT_PACKER.unpack(self.recv_bytes(2))[0]
        buf = self.recv_bytes(size)
        return buf.decode("utf8", "ignore")

    def send_text(self, message):
        self.sock.send(SHORT_PACKER.pack(len(message) + 2))
        if isinstance(message, str):
            self.sock.send(message.encode())
        else:
            self.sock.send(message)

    def connect(self):
        if self.sock:
            self.close()

        s = socket.socket()
        s.connect(self.endpoint)

        hello = s.recv(8, socket.MSG_WAITALL)
        if hello != b"FLUX0003":
            raise NotSupportError()

        self.sock = ssl.SSLSocket(s)

        # Stage 1: Recv randbytes
        self.randbytes = self.recv_bytes(64)

        # Stage 2: Send public key
        strkey = self.client_key.public_key_pem.decode("ascii")
        self.send_text(strkey)

        # Stage 3: Get public key status
        resp = self.recv_text()

        if resp == "sign":
            # Stage 4.a: Sign
            doc = HMAC(self.uuid.bytes, self.randbytes, sha1).digest()
            signature = self.client_key.sign(doc)
            self.send_text(to_hex(signature))

        elif resp == "password":
            # Stage 4.b: Send password
            return

        elif resp.startswith("error "):
            raise raise_error(resp)
        else:
            raise NotSupportError("Auth method %s not support", resp)

        resp = self.recv_text()
        if resp == "ok":
            self._authorized = True
            return
        elif resp.startswith("error "):
            err = resp[6:]
            if err == "AUTH_ERROR":
                raise AuthError()
            else:
                raise UpnpError(err)

    @property
    def connected(self):
        return self.sock

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None
            self._authorized = False

    def authorize_with_password(self, password):
        # Stage 4.b: Send password
        self.send_text(password)
        resp = self.recv_text()
        if resp == "ok":
            self._authorized = True
        else:
            self.close()
            raise AuthError("Bad password")

    def add_trust(self, label, pem):
        self.send_text("add_trust\x00%s\x00%s" % (label, pem))
        resp = self.recv_text()
        if resp != "ok":
            raise raise_error(resp, OPERATION_ERROR="Key already in list")

    def list_trust(self):
        data = []
        self.send_text("list_trust")
        while True:
            resp = self.recv_text()
            if resp.startswith("data "):
                d = {}
                for strpair in resp[5:].split("\x00"):
                    key, value = ensure_pair(*(strpair.split("=", 1)))
                    d[key] = value
                data.append(d)

            elif resp == "ok":
                return data
            else:
                raise raise_error(resp)

    def remove_trust(self, access_id):
        self.send_text("remove_trust\x00%s" % access_id)
        resp = self.recv_text()
        if resp == "ok":
            return
        else:
            raise raise_error(resp)

    def rename(self, new_name):
        self.send_text(("\x00".join(("rename", new_name))).encode())
        resp = self.recv_text()
        if resp != "ok":
            raise raise_error(resp)

    def modify_password(self, old_password, new_password, reset_acl):
        clean_acl = "Y" if reset_acl else "F"
        cmd = "\x00".join(("passwd", old_password, new_password, clean_acl))
        self.send_text(cmd.encode())
        resp = self.recv_text()
        if resp != "ok":
            raise raise_error(resp)

    def modify_network(self, **settings):
        opts = ["network"]
        for key, value in settings.items():
            opts.append("%s=%s" % (key, value))
        cmd = "\x00".join(opts)
        self.send_text(cmd)

    def get_wifi_list(self):
        self.send_text("scan_wifi")
        l = []

        while True:
            resp = self.recv_text()
            if resp.startswith("data "):
                l.append(json.loads(resp[5:]))
            elif resp == "ok":
                return l
            else:
                raise raise_error(resp)
