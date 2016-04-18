
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


def connect_camera(endpoint, metadata=None, server_key=None, client_key=None,
                   conn_callback=None):
    s = _connect(endpoint, conn_callback)
    s.settimeout(8)

    version = msg_waitall(s, 8)
    wrap_sock = _select_wrapper(s, version, metadata, server_key, client_key)
    return FluxCamera(wrap_sock)


def connect_robot(endpoint, metadata=None, server_key=None, client_key=None,
                  conn_callback=None):
    s = _connect(endpoint, conn_callback)
    s.settimeout(8)

    version = msg_waitall(s, 8)
    wrap_sock = _select_wrapper(s, version, metadata, server_key, client_key)
    return FluxRobot(wrap_sock)


def _connect(endpoint, conn_callback):
    logger.info("Connecting")
    while True:
        t = time()

        try:
            s = socket.socket()
            s.connect(endpoint)
            return s
        except ConnectionRefusedError:  # noqa
            if conn_callback and conn_callback():
                t = min(0.6 - time() + t, 0.6)
                if t > 0:
                    sleep(t)
                continue
            raise


def _select_wrapper(sock, version, metadata, server_key, client_key):
    if version[:4] != b"FLUX":
        raise RobotError("PROTOCOL_ERROR", "MAGICNUMBER_ERROR")
    elif version[4:] == b"0002":
        if not server_key and metadata:
            server_key = metadata["slave_key"]

        if metadata and "slave_key" in metadata:
            aessock = AESSocket(sock, server_key=metadata["slave_key"],
                                client_key=client_key)
        else:
            aessock = AESSocket(sock, server_key=None, client_key=client_key)

        while not aessock.do_handshake():
            pass
        return aessock
    elif version[4:] == b"0003":
        if not server_key and metadata:
            server_key = metadata["master_key"]
        sslsock = SSLSocket(sock, server_key=server_key, client_key=client_key)
        while sslsock.do_handshake() != 0:
            pass
        return sslsock
    else:
        raise RobotError("Robot version not support")
