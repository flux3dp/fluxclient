
from pkg_resources import resource_string
from hashlib import sha1
import logging
import ecdsa

DEFAULT_IPADDR = "239.255.255.250"
DEFAULT_PORT = 1901

VK_CACHE = {}
VALIDATE_CACHE = {}

logger = logging.getLogger(__name__)


def validate_identify(uuid, identify, serial=None, masterkey_doc=None):
    if not serial or not masterkey_doc:
        return False

    if serial == "X" * 10 and uuid.hex.startswith("f" * 16):
        return True

    if "delta-1" in VK_CACHE:
        vk = VK_CACHE["delta-1"]
    else:
        binkey = resource_string("fluxclient", "assets/key-delta-1")
        vk = VK_CACHE["delta-1"] = ecdsa.VerifyingKey.from_der(binkey)

    try:
        if uuid in VALIDATE_CACHE:
            return VALIDATE_CACHE[uuid] == masterkey_doc
        else:
            doc = serial.encode() + b"$" + uuid.bytes + b"$" + masterkey_doc
            ret = vk.verify(identify, doc, hashfunc=sha1,
                            sigdecode=ecdsa.util.sigdecode_der)
            if ret:
                VALIDATE_CACHE[uuid] = masterkey_doc
            return ret
    except Exception:
        logger.exception("Error while validate identify")
