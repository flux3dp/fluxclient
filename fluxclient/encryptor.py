
from io import BytesIO
from hashlib import sha1
import warnings

from Crypto.Signature import PKCS1_v1_5
from Crypto.Cipher import PKCS1_OAEP
from Crypto.PublicKey import RSA
from Crypto.Hash import SHA as CryptoSHA  # noqa


class KeyObject(object):
    _size = None

    @classmethod
    def load_keyobj(cls, pem_or_der):
        ref = RSA.importKey(pem_or_der)
        return cls(ref)

    @classmethod
    def new_keyobj(cls, size=4096):
        ref = RSA.generate(size)
        return cls(ref)

    @classmethod
    def get_or_create_keyobj(cls, path=None):
        warnings.warn("get_or_create_keyobj is deprecated", DeprecationWarning)
        from fluxclient.commands.misc import get_or_create_default_key
        return get_or_create_default_key(path)

    def __init__(self, ref_keyobj):
        self._key = ref_keyobj

    def __eq__(self, obj):
        if isinstance(obj, KeyObject):
            return self.public_key_der == obj.public_key_der
        return False

    def encrypt(self, message):
        chip = PKCS1_OAEP.new(self._key)
        size = self.size - 42
        in_buf = BytesIO(message)
        out_buf = BytesIO()

        buf = in_buf.read(size)
        while buf:
            out_buf.write(chip.encrypt(buf))
            buf = in_buf.read(size)

        return out_buf.getvalue()

    def decrypt(self, buf):
        chip = PKCS1_OAEP.new(self._key)
        size = self.size
        in_buf = BytesIO(buf)
        out_buf = BytesIO()

        buf = in_buf.read(size)
        while buf:
            out_buf.write(chip.decrypt(buf))
            buf = in_buf.read(size)

        return out_buf.getvalue()

    @property
    def size(self):
        if self._size is None:
            s = self._key.size()
            while s % 8 > 0:
                s += 1
            self._size = s // 8
        return self._size

    @property
    def public_key_pem(self):
        return self._key.publickey().exportKey("PEM")

    @property
    def public_key_der(self):
        return self._key.publickey().exportKey("DER")

    @property
    def private_key_pem(self):
        return self._key.exportKey("PEM")

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
