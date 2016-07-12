
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
    robot = FluxRobot((metadata["ipaddr"], 23811), client_key)
"""

from .robot import FluxRobot
from .camera import FluxCamera

__all__ = ["FluxRobot", "FluxCamera"]
