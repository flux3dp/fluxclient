
import warnings

from fluxclient.device.device import Device, DEVICE_STATUS_CODE

warnings.warn("fluxclient.upnp.device is going to be deprecate, "
              "use fluxclient.device.device instead", DeprecationWarning)

__all__ = ["Device", DEVICE_STATUS_CODE]
