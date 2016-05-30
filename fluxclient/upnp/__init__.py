

from .abstract_backend import (UpnpError, NotSupportError, AuthError,
                               TimeoutError)
from .discover import UpnpDiscover
from .device import Device
from .task import UpnpTask

__all__ = ["UpnpDiscover", "UpnpTask", "UpnpError", "NotSupportError",
           "AuthError", "TimeoutError", "discover_device", "Device"]


def discover_device(uuid, lookup_callback=None):
    """Discover and return device

    :param uuid uuid: Device UUID
    :param callable lookup_callback: A callable object will be invoke during \
discover.
    :rtype: :class:`fluxclient.upnp.device.Device`"""

    result = []

    def found_callback(discover, device, **kw):
        result.append(device)
        discover.stop()

    discover = UpnpDiscover(uuid)
    discover.discover(found_callback, lookup_callback)

    return result[0]
