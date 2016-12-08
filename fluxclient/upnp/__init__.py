
import warnings

from fluxclient.device import discover_device
from .discover import UpnpDiscover
from .device import Device
from .task import UpnpTask, UpnpError, UpnpException

__all__ = ["UpnpTask", "UpnpDiscover", "discover_device", "Device",
           "UpnpError", "UpnpException"]

warnings.warn("fluxclient.upnp. is going to be deprecate, "
              "use fluxclient.device instead", DeprecationWarning)

# Module rename note:
#   fluxclient.upnp.device.Device -> fluxclient.device.device.Device
#   fluxclient.upnp.discover.UpnpDiscover -> fluxclient.device.discover.DeviceDiscover
#   fluxclient.upnp.task.UpnpTask -> fluxclient.device.manager.DeviceManager
#   fluxclient.upnp.task.UpnpError -> fluxclient.device.manager.CommandError
#   fluxclient.upnp.task.UpnpException -> fluxclient.device.manager.ManagerException
