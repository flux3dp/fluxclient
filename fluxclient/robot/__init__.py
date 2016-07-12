
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

    # Case 1: Get device from uuid
    device = discover_device(uuid)
    robot = device.connect_robot(client_key)

    # Case 2: Connect to robot direct
    robot = connect_robot((metadata["ipaddr"], 23811), client_key)
"""

from .robot import FluxRobot
from .camera import FluxCamera


def connect_camera(endpoint, client_key, device=None, conn_callback=None):
    """Make a connection to device camera service, backends will be selecte
    automatically.

    :param tuple endpoint: A tuple contain a pair of IP address and port to \
connect. For example: ("192.168.1.1", 23812)
    :param encrypt.KeyObject client_key: Client identify key
    :param dict device: Device instance
    :param callable conn_callback: A callback will be invoked while trying \
    connect to device
    """
    return FluxCamera(endpoint, client_key, device)


def connect_robot(endpoint, client_key, device=None, conn_callback=None):
    """Make a connection to device robot service, backends will be selecte
    automatically.

    :param tuple endpoint: A tuple contain a pair of IP address and port to \
connect. For example: ("192.168.1.1", 23811)
    :param encrypt.KeyObject client_key: Client identify key
    :param dict device: Device instance
to assign value because it may has different definition in different version.
    :param callable conn_callback: A callback will be invoked while trying \
    connect to device

    :rtype: fluxclient.robot.robot.FluxRobot
    """
    return FluxRobot(endpoint, client_key, device)
