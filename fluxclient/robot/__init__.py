
"""`fluxclient.robot` provides an interface to control a FLUX device.

An example to connect and control a device::

    from fluxclient.encrypt import KeyObject
    from fluxclient.robot import connect_robot
    from fluxclient.commands.misc import get_or_create_default_key

    client_key = get_or_create_default_key("./sdk_connection.pem")
    robot = connect_robot(("192.168.1.5", 23811), client_key)

.. note:: You might want to check `this link </tutorials/device_control.html>`_ for details.

    # Case 1: Get device from uuid
    device = discover_device(uuid)
    robot = device.connect_robot(client_key)

    # Case 2: Connect to robot direct
    robot = FluxRobot((metadata["ipaddr"], 23811), client_key)
"""

from .robot import FluxRobot
from .camera import FluxCamera

__all__ = ["FluxRobot", "FluxCamera"]
