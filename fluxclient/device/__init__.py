

from .discover import DeviceDiscover
from .manager import DeviceManager
from .device import Device

__all__ = ["DeviceDiscover", "Device", "DeviceManager", "discover_device"]


def discover_device(uuid, lookup_callback=None, timeout=float("INF")):
    """Discover and return device

    :param uuid uuid: Device UUID
    :param callable lookup_callback: A callable object will be invoke during \
discover.
    :rtype: :class:`fluxclient.device.device.Device`"""

    result = []

    def found_callback(discover, device, **kw):
        result.append(device)
        discover.stop()

    discover = DeviceDiscover(uuid)
    discover.discover(found_callback, lookup_callback, timeout)

    return result[0] if result else None
