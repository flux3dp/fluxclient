
import warnings

from fluxclient.device.manager import DeviceManager as UpnpTask
from fluxclient.device.manager_backends import ManagerError as UpnpError
from fluxclient.device.manager_backends import ManagerException as UpnpException


warnings.warn("fluxclient.upnp.task is going to be deprecate, "
              "use fluxclient.device.manager instead", DeprecationWarning)

__all__ = ["UpnpTask"]
