
"""
The `fluxclient.upnp` module provide method to discover, authorized, manage \
security and network for device.
"""


from .abstract_backend import (UpnpError, NotSupportError, AuthError,
                               TimeoutError)
from .discover import UpnpDiscover
from .task import UpnpTask

__all__ = ["UpnpDiscover", "UpnpTask", "UpnpError", "NotSupportError",
           "AuthError", "TimeoutError", "discover_device"]


def discover_device(uuid, lookup_callback=None, return_device=False):
    """Discover and return device metadata

    :param uuid: Device UUID
    :param lookup_callback: A callable object will be invoke during discover.
Callback definition::
    def callback(discover_instance):
        pass
    :rtype: Device metadata"""

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
