
from uuid import UUID
import sys
import os

from fluxclient.robot.misc import is_uuid
from fluxclient.encryptor import KeyObject
from fluxclient.upnp.task import UpnpTask


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


def discover_device(target, client_key, logstream=sys.stdout):
    def lookup_callback(discover):
        logstream.write(".")
        logstream.flush()

    logstream.write("Discover...")
    logstream.flush()

    device = UpnpTask(UUID(hex=target), client_key,
                      lookup_callback=lookup_callback)
    ipaddr = device.endpoint[0]
    logstream.write(" OK\n")
    logstream.write("Name: %s\nUUID: %s\nModel: %s\nIP Addr: %s\n" %
                    (device.name, device.uuid.hex, device.model_id, ipaddr))
    logstream.flush()

    while True:
        try:
            device.require_auth()
            break
        except RuntimeError as e:
            logstream.write("Error: %s, retry...\n" % e.args[0])

    return device


def get_robot_endpoint(target, client_key, logstream=sys.stdout):
    if is_uuid(target):
        device = discover_device(target, client_key, logstream)
        return (device.endpoint[0], 23811), device
    else:
        return parse_ipaddr(target, 23811), None


def get_camera_endpoint(target, client_key, logstream=sys.stdout):
    if is_uuid(target):
        device = discover_device(target, client_key, logstream)
        return (device.endpoint[0], 23812), device
    else:
        return parse_ipaddr(target, 23812), None
