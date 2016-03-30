
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


def connect_camera(endpoint, device=None, server_key=None, client_key=None,
                   conn_callback=None):
    s = _connect(endpoint, conn_callback)
    s.settimeout(8)

    version = msg_waitall(s, 8)
    wrap_sock = _select_wrapper(s, version, device, server_key, client_key)
    return FluxCamera(wrap_sock)


def connect_robot(endpoint, device=None, server_key=None, client_key=None,
                  conn_callback=None):
    # TODO: argument will be change after next fluxghost release
    if not client_key:
        from fluxclient.commands.misc import get_or_create_default_key
        client_key = get_or_create_default_key()

    s = _connect(endpoint, conn_callback)
    s.settimeout(8)

    version = msg_waitall(s, 8)
    wrap_sock = _select_wrapper(s, version, device, server_key, client_key)
    return FluxRobot(wrap_sock)


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


def _select_wrapper(sock, version, device=None, server_key=None,
                    client_key=None):
    if version[:4] != b"FLUX":
        raise RobotError("Magic number error")
    elif version[4:] == b"0002":
        if not server_key and device:
            server_key = device.slave_key

        aessock = AESSocket(sock, server_key=server_key, client_key=client_key)
        while not aessock.do_handshake():
            pass
        return aessock
    elif version[4:] == b"0003":
        if not server_key and device:
            server_key = device.master_key
        sslsock = SSLSocket(sock, server_key=server_key, client_key=client_key)
        while sslsock.do_handshake() != 0:
            pass
        return sslsock
    else:
        raise RobotError("Robot version not support")
