
from time import time, sleep
import logging
import socket

from .base import RobotError
from .misc import msg_waitall

logger = logging.getLogger(__name__)


def connect_robot(ipaddr, server_key, client_key, conn_callback):
    sock = _connect(ipaddr, conn_callback)
    sock.settimeout(8)

    version = msg_waitall(sock, 8)

    if version[:4] != b"FLUX":
        raise RobotError("Magic number error")
    elif version[4:] == b"0002":
        from .v0002 import FluxRobotV0002
        return FluxRobotV0002(sock, server_key)
    else:
        raise RobotError("Robot version not support")


def _connect(ipaddr, conn_callback):
    logger.info("Connecting")
    while True:
        t = time()

        try:
            s = socket.socket()
            s.connect(ipaddr)
            return s
        except ConnectionRefusedError:
            if conn_callback and conn_callback():
                t = min(0.6 - time() + t, 0.6)
                if t > 0:
                    sleep(t)
                continue
            raise
