
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


def connect_robot_helper(target, client_key, logstream=sys.stdout):
    from fluxclient.robot.misc import is_uuid
    from fluxclient.upnp import discover_device
    from fluxclient.robot.robot import FluxRobot

    def working_callback(discover):
        logstream.write(".")
        logstream.flush()

    if is_uuid(target):
        uuid = UUID(hex=target)

        logstream.write("Discover...")
        logstream.flush()

        device = discover_device(uuid, working_callback)
        logstream.write("\nConnecting...")
        session = device.connect_robot(client_key,
                                       conn_callback=working_callback)
    else:
        device = None
        logstream.write("Connecting...")
        endpoint = parse_ipaddr(target, 23811)
        session = FluxRobot(endpoint, client_key)

    logstream.write(" OK\n")
    return session, device


def connect_camera_helper(target, client_key, logstream=sys.stdout):
    from fluxclient.robot.misc import is_uuid
    from fluxclient.upnp import discover_device
    from fluxclient.robot.camera import FluxCamera

    def working_callback(discover):
        logstream.write(".")
        logstream.flush()

    if is_uuid(target):
        uuid = UUID(hex=target)

        logstream.write("Discover...")
        logstream.flush()

        device = discover_device(uuid, working_callback)
        logstream.write("\nConnecting...")
        session = device.connect_camera(client_key,
                                        conn_callback=working_callback)
    else:
        device = None
        logstream.write("Connecting...")
        endpoint = parse_ipaddr(target, 23812)
        session = FluxCamera(endpoint, client_key)

    logstream.write(" OK\n")
    return session, device


class CharacterRenderHelper(object):
    text = None
    index = 0

    def __init__(self, text="-\\|/"):
        self.text = text

    def render(self, fileobj):
        c = self.text[self.index]
        self.index = (self.index + 1) % len(self.text)
        fileobj.write("\r%s" % c)
        fileobj.flush()
