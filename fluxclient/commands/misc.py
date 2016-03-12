
import os

from fluxclient.encryptor import KeyObject


def get_or_create_default_key(path=None):
    if path is None:
        path = os.path.expanduser("~/.fluxclient_key.pem")

    if os.path.exists(path):
        try:
            with open(path, "rb") as f:
                buf = f.read()
                return KeyObject.load_keyobj(buf)
        except Exception:
            raise
            os.unlink(path)

    key = KeyObject.get_or_create_keyobj(1024)
    with open(path, "wb") as f:
        f.write(key.private_key_pem)

    return key
