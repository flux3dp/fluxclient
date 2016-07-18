
from configparser import RawConfigParser
from uuid import UUID
import tempfile
import os

from fluxclient.encryptor import KeyObject
from fluxclient.upnp import discover_device

CLIENTKEY = KeyObject.new_keyobj(1024)
CLIENTKEY_FILE = tempfile.NamedTemporaryFile()
CLIENTKEY_FILE.write(CLIENTKEY.private_key_pem)
CLIENTKEY_FILE.flush()


DEFAULT_UUID = None
DEFAULT_PASSWORD = None
DEFAULT_DEVICE = None

if os.path.exists("conftest.ini"):
    parser = RawConfigParser()
    parser.read("conftest.ini")

    if "default_device" in parser.sections():
        devices = dict(parser.items("default_device"))
        DEFAULT_UUID = devices.get("uuid", None)
        DEFAULT_PASSWORD = devices.get("password", None)
        if DEFAULT_UUID:
            DEFAULT_DEVICE = discover_device(UUID(hex=DEFAULT_UUID), timeout=5)
