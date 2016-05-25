
"""`fluxclient.robot` provide interface to control a FLUX device.

A simple example to connect and control a device::

    from fluxclient.encrypt import KeyObject
    from fluxclient.robot import connect_robot
    from fluxclient.upnp import discover_device
    from uuid import UUID

    # Get device uuid
    uuid = UUID(hex="123345789abcdef123456789abcdef00")

    # Load client key
    with open("mykey.pem", "r") as f:
        client_key = KeyObject.load_keyobj(f.read())

    # Get device metadata
    metadata = discover_device(uuid)

    # Connect device
    robot = connect_robot((metadata["ipaddr"], 23811), client_key, metadata)
"""

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


def connect_camera(endpoint, client_key, metadata=None, conn_callback=None):
    """Make a connection to device camera service, backends will be selecte
    automatically.

    :param tuple endpoint: A tuple contain a pair of IP address and port to \
connect. For example: ("192.168.1.1", 23812)
    :param encrypt.KeyObject client_key: Client identify key
    :param dict metadata: metadata is an internal param, it is not recommend \
to assign value because it may has different definition in different version.
    :param callable conn_callback: A callback will be invoked while trying \
    connect to device
    """

    s = _connect(endpoint, conn_callback)
    s.settimeout(8)

    version = msg_waitall(s, 8)
    wrap_sock = _select_wrapper(s, client_key, version, metadata)
    return FluxCamera(wrap_sock)


def connect_robot(endpoint, client_key, metadata=None, conn_callback=None):
    """Make a connection to device robot service, backends will be selecte
    automatically.

    :param tuple endpoint: A tuple contain a pair of IP address and port to \
connect. For example: ("192.168.1.1", 23811)
    :param encrypt.KeyObject client_key: Client identify key
    :param dict metadata: metadata is an internal param, it is not recommend \
to assign value because it may has different definition in different version.
    :param callable conn_callback: A callback will be invoked while trying \
    connect to device

    :rtype: fluxclient.robot.robot.FluxRobots
    """
    s = _connect(endpoint, conn_callback)
    s.settimeout(8)

    version = msg_waitall(s, 8)
    wrap_sock = _select_wrapper(s, client_key, version, metadata)
    return FluxRobot(wrap_sock, metadata=metadata)


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


def _select_wrapper(sock, client_key, version, metadata):
    if version[:4] != b"FLUX":
        raise RobotError("PROTOCOL_ERROR", "MAGICNUMBER_ERROR")
    elif version[4:] == b"0002":
        if metadata and "slave_key" in metadata:
            server_key = metadata["slave_key"]
        else:
            server_key = None

        aessock = AESSocket(sock, client_key=client_key, server_key=server_key)
        while not aessock.do_handshake():
            pass

        return aessock

    elif version[4:] == b"0003":
        if metadata and "master_key" in metadata:
            server_key = metadata["master_key"]
        else:
            server_key = None

        sslsock = SSLSocket(sock, client_key=client_key, server_key=server_key)
        while sslsock.do_handshake() != 0:
            pass
        return sslsock
    else:
        raise RobotError("Robot version not support")
