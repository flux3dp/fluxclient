
import uuid as _uuid


DEFAULT_PORT = 3310

CODE_DISCOVER = 0x00
CODE_RESPONSE_DISCOVER = 0x01

CODE_RSA_KEY = 0x02
CODE_RESPONSE_RSA_KEY = 0x03

CODE_NOPWD_ACCESS = 0x04
CODE_RESPONSE_NOPWD_ACCESS = 0x05

CODE_PWD_ACCESS = 0x06
CODE_RESPONSE_PWD_ACCESS = 0x07

CODE_CHANGE_PWD = 0x08
CODE_RESPONSE_CHANGE_PWD = 0x09

CODE_SET_NETWORK = 0x0a
CODE_RESPONSE_SET_NETWORK = 0x0b

GLOBAL_SERIAL = _uuid.UUID(int=0)
HEXMAP = "123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"


def uuid_to_short(uuid_hex, mapping=HEXMAP):
    u = _uuid.UUID(uuid_hex)
    l = len(mapping)
    n = u.int
    a_short = []
    while n > 0:
        c = mapping[n % l]
        n = n // l
        a_short.append(c)

    while len(a_short) < 25:
        a_short.append(mapping[0])

    return "".join(a_short)


def short_to_uuid(short, mapping=HEXMAP):
    n = 0
    offset = 1
    l = len(mapping)

    for c in short:
        n += mapping.index(c) * offset
        offset *= l

    return _uuid.UUID(int=n).hex


def parse_network_config(method, ipaddr=None, mask=None, route=None, ns=None,
                         ssid=None, security=None, wepkey=None, psk=None):
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
        options["ssid"] = ssid

        if security == "WEP":
            assert wepkey, "wepkey is required"
            options["security"] = security
            options["wepkey"] = wepkey
        elif security in ['WPA-PSK', 'WPA2-PSK']:
            options["security"] = security
            options["psk"] = psk

    return options
