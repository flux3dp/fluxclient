
from fluxclient.device.discover import DeviceDiscover
from .base import (ManagerError, ManagerException, NotSupportError, AuthError,
                   TimeoutError, ConnectionBroken)
from .ssl1 import SSL1Backend
from .udp1 import Udp1Backend
from .host2host1 import Host2HostBackend1
from .uart import UartBackend

__all__ = ["Host2HostBackend1", "UartBackend",
           "ManagerError", "ManagerException",
           "NotSupportError", "AuthError", "TimeoutError", "ConnectionBroken",
           "get_backend_via_uuid", "get_backend_via_ipaddr"]


def get_backend_via_network(device):
    for klass in (SSL1Backend, Udp1Backend):
        if klass.support_device(device.model_id, device.version):
            return klass
    raise RuntimeError("NOT_SUPPORT", device.model_id, device.version)


def get_backend_via_uuid(uuid, lookup_callback=None,
                         lookup_timeout=float("INF")):
    result = []

    def on_discovered(instance, device, **kw):
        result.append(device)
        instance.stop()

    d = DeviceDiscover(uuid=uuid)
    d.discover(on_discovered, lookup_callback, lookup_timeout)

    if result:
        return get_backend_via_network(result[0]), result[0]
    else:
        raise TimeoutError()


def get_backend_via_ipaddr(ipaddr, lookup_callback=None,
                           lookup_timeout=float("INF")):
    result = []

    def on_discovered(instance, device, **kw):
        result.append(device)
        instance.stop()

    d = DeviceDiscover(device_ipaddr=ipaddr)
    d.discover(on_discovered, lookup_callback, lookup_timeout)

    if result:
        return get_backend_via_network(result[0]), result[0]
    else:
        raise TimeoutError()
