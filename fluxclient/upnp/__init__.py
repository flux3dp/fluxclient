

from .abstract_backend import (UpnpError, NotSupportError, AuthError,
                               TimeoutError)
from .discover import UpnpDiscover
from .device import Device
from .task import UpnpTask

__all__ = ["UpnpDiscover", "UpnpTask", "UpnpError", "NotSupportError",
           "AuthError", "TimeoutError", "discover_device", "Device"]


def discover_device(uuid, lookup_callback=None, return_device=False):
    """Discover and return device

    :param uuid uuid: Device UUID
    :param callable lookup_callback: A callable object will be invoke during \
discover.
    :rtype: :class:`fluxclient.upnp.device.Device`"""

    result = []

    def found_callback(discover, **kw):
        result.append(kw)
        discover.stop()

    discover = UpnpDiscover(uuid)
    discover.discover(found_callback, lookup_callback)

    if return_device:
        return result[0]["device"]
    else:
        return result[0]
