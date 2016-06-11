
from select import select
from time import time
import re

from .errors import RobotSessionError


def is_uuid(input):
    return True if re.match("[0-9a-fA-F]{32}", input) else False


def msg_waitall(sock, length, timeout):
    buf = b""
    ttl = time() + timeout

    while len(buf) < length and (ttl - time() > 0):
        rl = select((sock,), (), (), ttl - time())[0]
        if rl:
            chunk = sock.recv(length - len(buf))
            if chunk:
                buf += chunk
            else:
                raise RobotSessionError("Recive message borken")

    if len(buf) != length:
        raise RobotSessionError("Recive message borken")

    return buf
