
from uuid import UUID
import sys
import re

from fluxclient.upnp.task import UpnpTask


def is_uuid(input):
    return True if re.match("[0-9a-fA-F]{32}", input) else False


def parse_ipaddr(target):
    if ":" in target:
        addr, port = target.split(":")
        return (addr, int(port))
    else:
        return (target, 23811)


def require_robot(target, client_key, logstream=sys.stdout):
    def lookup_callback(discover):
        logstream.write(".")
        logstream.flush()

    if is_uuid(target):
        logstream.write("Discover...")
        logstream.flush()

        task = UpnpTask(UUID(hex=target), client_key,
                        lookup_callback=lookup_callback)
        ipaddr = task.endpoint[0]
        logstream.write(" OK\n")
        logstream.write("Name: %s\nUUID: %s\nModel: %s\nIP Addr: %s\n" %
                        (task.name, task.uuid.hex, task.model_id, ipaddr))
        logstream.flush()

        while True:
            try:
                task.require_auth()
                break
            except RuntimeError as e:
                logstream.write("Error: %s, retry...\n" % e.args[0])

        logstream.write("Wakeup Robot: ")
        logstream.flush()

        return (ipaddr, 23811), task.slave_key

    else:
        return parse_ipaddr(target), None


def msg_waitall(sock, length):
    buf = b""

    while len(buf) < length:
        buf += sock.recv(length - len(buf))

    return buf
