
from uuid import UUID
import logging.config
import sys
import os


def setup_logger(name, stdout=sys.stderr, debug=False):
    level = logging.DEBUG if debug else logging.INFO

    logging.config.dictConfig({
        'version': 1,
        'disable_existing_loggers': True,
        'formatters': {
            'default': {
                'format': "%(message)s"
            }
        },
        'handlers': {
            'console': {
                'level': level,
                'formatter': 'default',
                'class': 'logging.StreamHandler',
            }
        },
        'loggers': {},
        'root': {
            'handlers': ['console'],
            'level': level,
            'propagate': True
        }
    })

    return logging.getLogger(name)


def get_or_create_default_key(path=None):
    from fluxclient.encryptor import KeyObject

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

    key = KeyObject.new_keyobj(1024)
    with open(path, "wb") as f:
        f.write(key.private_key_pem)

    return key


def parse_ipaddr(target, default_port):
    if ":" in target:
        addr, port = target.split(":")
        return (addr, int(port))
    else:
        return (target, default_port)


def discover_device(uuid, client_key, logstream=sys.stdout):
    result = []

    def lookup_callback(discover):
        logstream.write(".")
        logstream.flush()

    def found_callback(discover, **kw):
        result.append(kw)
        discover.stop()

    logstream.write("Discover...")
    logstream.flush()

    from fluxclient.upnp import UpnpDiscover

    discover = UpnpDiscover(uuid)
    discover.discover(found_callback, lookup_callback)

    logstream.write(" OK\n")
    logstream.flush()

    return result[0]


def get_device_endpoint(target, client_key, default_port,
                        logstream=sys.stdout):
    from fluxclient.robot.misc import is_uuid

    if is_uuid(target):
        metadata = discover_device(UUID(hex=target), client_key, logstream)
        ipaddr = metadata.get("ipaddr")
        if not ipaddr:
            raise RuntimeError("Can not get ipaddress from target %s" % target)

        return (ipaddr, default_port), metadata
    else:
        return parse_ipaddr(target, default_port), {}
