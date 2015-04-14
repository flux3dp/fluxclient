
from io import BytesIO
import os

from Crypto.Signature import PKCS1_v1_5
from Crypto.Cipher import PKCS1_OAEP
from Crypto.PublicKey import RSA
from Crypto.Hash import SHA as CryptoSHA


def load_keyobj(pem):
    return RSA.importKey(pem)


def get_or_create_keyobj(path=None):
    if path is None:
        path = os.path.expanduser("~/.fluxclient_key.pem")

    if os.path.exists(path):
        try:
            with open(path, "rb") as f:
                buf = f.read()
                return RSA.importKey(buf)
        except Exception:
            raise
            os.unlink(path)

    keyobj = RSA.generate(1024)
    with open(path, "wb") as f:
        f.write(keyobj.exportKey("PEM"))
    return keyobj


def encrypt(keyobj, message):
    chip = PKCS1_OAEP.new(keyobj)
    size = ((keyobj.size() + 1) / 8) - 42
    in_buf = BytesIO(message)
    out_buf = BytesIO()

    buf = in_buf.read(size)
    while buf:
        out_buf.write(chip.encrypt(buf))
        buf = in_buf.read(size)

    return out_buf.getvalue()


def get_public_key_der(keyobj):
    return keyobj.publickey().exportKey("DER")


def sign(keyobj, message):
    chip = PKCS1_v1_5.new(keyobj)
    return chip.sign(CryptoSHA.new(message))


def validate_signature(keyobj, message, signature):
    chip = PKCS1_v1_5.new(keyobj)
    return chip.verify(CryptoSHA.new(message), signature)
