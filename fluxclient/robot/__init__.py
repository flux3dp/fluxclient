
from time import time, sleep
import logging
import socket

from .aes_socket import AESSocket
from .ssl_socket import SSLSocket
from .errors import RobotError
from .robot import FluxRobot
from .camera import FluxCamera
from .misc import msg_waitall

logger = logging.getLogger(__name__)


def connect_camera(endpoint, server_key, client_key, conn_callback=None):
    sock = _connect(endpoint, conn_callback)
    sock.settimeout(8)

    version = msg_waitall(sock, 8)

    if version[:4] != b"FLUX":
        raise RobotError("Magic number error")
    elif version[4:] == b"0002":
        aessock = AESSocket(sock, server_key=server_key, client_key=client_key)
        while not aessock.do_handshake():
            pass
        return FluxCamera(aessock)
    elif version[4:] == b"0003":
        sslsock = SSLSocket(sock, server_key=server_key, client_key=client_key)
        while sslsock.do_handshake() != 0:
            pass
        return FluxCamera(sslsock)
    else:
        raise RobotError("Robot version not support")


def connect_robot(ipaddr, server_key, client_key=None, conn_callback=None):
    # TODO: argument will be change after next fluxghost release
    if not client_key:
        from fluxclient.commands.misc import get_or_create_default_key
        client_key = get_or_create_default_key()
    sock = _connect(ipaddr, conn_callback)
    sock.settimeout(8)

    version = msg_waitall(sock, 8)

    if version[:4] != b"FLUX":
        raise RobotError("Magic number error")
    elif version[4:] == b"0002":
        aessock = AESSocket(sock, server_key=server_key, client_key=client_key)
        while not aessock.do_handshake():
            pass
        return FluxRobot(aessock)
    else:
        raise RobotError("Robot version not support")


def _connect(endpoint, conn_callback):
    logger.info("Connecting")
    while True:
        t = time()

        try:
            s = socket.socket()
            s.connect(endpoint)
            return s
        except ConnectionRefusedError:
            if conn_callback and conn_callback():
                t = min(0.6 - time() + t, 0.6)
                if t > 0:
                    sleep(t)
                continue
            raise
