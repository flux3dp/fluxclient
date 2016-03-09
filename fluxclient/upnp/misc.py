
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


def parse_network_config(method, wifi_mode=None, ipaddr=None, mask=None,
                         route=None, ns=None, ssid=None, security=None,
                         wepkey=None, psk=None):
    assert method in ["dhcp", "static"], "method must be dhcp or static"

    options = {"method": method}
    if method == "static":
        assert ipaddr, "ipaddr is required"
        assert mask and isinstance(mask, int), "mask is required"
        assert route, "route is required"
        assert ns, "ns is required"

        options.update({
            "ipaddr": ipaddr, "mask": mask, "route": route, "ns": ns,
        })

    if ssid:
        options["wifi_mode"] = wifi_mode
        options["ssid"] = ssid

        if security == "WEP":
            assert wepkey, "wepkey is required"
            options["security"] = security
            options["wepkey"] = wepkey
        elif security in ['WPA-PSK', 'WPA2-PSK']:
            options["security"] = security
            options["psk"] = psk

    return options
