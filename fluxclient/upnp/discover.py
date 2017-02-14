
import warnings

from fluxclient.device.discover import DeviceDiscover as UpnpDiscover

warnings.warn("fluxclient.upnp.discover is going to be deprecate, "
              "use fluxclient.device.discover instead", DeprecationWarning)

__all__ = ["UpnpDiscover"]
