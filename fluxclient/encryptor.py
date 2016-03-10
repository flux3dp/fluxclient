
from io import BytesIO
from hashlib import sha1
import os

from Crypto.Signature import PKCS1_v1_5
from Crypto.Cipher import PKCS1_OAEP
from Crypto.PublicKey import RSA
from Crypto.Hash import SHA as CryptoSHA  # noqa


class KeyObject(object):
    @classmethod
    def load_keyobj(cls, pem_or_der):
        ref = RSA.importKey(pem_or_der)
        return cls(ref)

    @classmethod
    def get_or_create_keyobj(cls, path=None):
        if path is None:
            path = os.path.expanduser("~/.fluxclient_key.pem")

        if os.path.exists(path):
            try:
                with open(path, "rb") as f:
                    buf = f.read()
                    ref = RSA.importKey(buf)
                    return cls(ref)
            except Exception:
                raise
                os.unlink(path)

        ref = RSA.generate(1024)
        with open(path, "wb") as f:
            f.write(ref.exportKey("PEM"))

        return cls(ref)

    def __init__(self, ref_keyobj):
        self._key = ref_keyobj

    def encrypt(self, message):
        chip = PKCS1_OAEP.new(self._key)
        size = ((self._key.size() + 1) // 8) - 42
        in_buf = BytesIO(message)
        out_buf = BytesIO()

        buf = in_buf.read(size)
        while buf:
            out_buf.write(chip.encrypt(buf))
            buf = in_buf.read(size)

        return out_buf.getvalue()

    def decrypt(self, buf):
        chip = PKCS1_OAEP.new(self._key)
        size = (self._key.size() + 1) // 8
        in_buf = BytesIO(buf)
        out_buf = BytesIO()

        buf = in_buf.read(size)
        while buf:
            out_buf.write(chip.decrypt(buf))
            buf = in_buf.read(size)

        return out_buf.getvalue()

    @property
    def size(self):
        return (self._key.size() + 1) // 8

    @property
    def public_key_pem(self):
        return self._key.publickey().exportKey("PEM")

    @property
    def public_key_der(self):
        return self._key.publickey().exportKey("DER")

    def sign(self, message):
        chip = PKCS1_v1_5.new(self._key)
        return chip.sign(CryptoSHA.new(message))

    def verify(self, message, signature):
        chip = PKCS1_v1_5.new(self._key)
        return chip.verify(CryptoSHA.new(message), signature)

    def get_access_id(self, binary=False):
        der = self._key.publickey().exportKey("DER")

        if binary:
            return sha1(der).digest()
        else:
            return sha1(der).hexdigest()
