
from fluxclient.utils.version import StrictVersion
from fluxclient.upnp.discover import UpnpDiscover
from .abstract_backend import NotSupportError
from .udp1_backend import UpnpUdp1Backend

BACKENDS = [UpnpUdp1Backend]


class UpnpTask(object):
    name = None
    uuid = None
    serial = None
    model_id = None
    version = None
    ipaddr = None
    meta = None

    _backend = None

    def __init__(self, uuid, client_key, ipaddr=None, remote_profile=None,
                 backend_options={}, lookup_callback=None,
                 lookup_timeout=float("INF")):
        self.uuid = uuid
        self.client_key = client_key
        self.backend_options = backend_options

        if remote_profile:
            self.update_remote_profile(**remote_profile)
        else:
            self.reload_remote_profile(lookup_callback, lookup_timeout)

        self.initialize_backend()

    def reload_remote_profile(self, lookup_callback=None,
                              lookup_timeout=float("INF")):
        def on_discovered(instance, **kw):
            self.update_remote_profile(**kw)
            instance.stop()

        if self.uuid.int:
            d = UpnpDiscover(uuid=self.uuid)
        else:
            d = UpnpDiscover(device_ipaddr=self.ipaddr)

        d.discover(on_discovered, lookup_callback, lookup_timeout)

    def update_remote_profile(self, uuid, name, serial, model_id, version,
                              ipaddr, **meta):
        if not self.uuid or self.uuid.int == 0:
            self.uuid = uuid
        self.name = name
        self.serial = serial
        self.model_id = model_id
        self.version = StrictVersion(version)
        self.ipaddr = ipaddr[0]
        self.device_meta = meta

    def initialize_backend(self):
        for klass in BACKENDS:
            if klass.support_device(self.model_id, self.version):
                self._backend = klass(self.client_key, self.uuid, self.version,
                                      self.model_id, self.ipaddr,
                                      self.device_meta, self.backend_options)
                return

        raise NotSupportError(self.model_id, self.version)

    def rename(self, new_name):
        self._backend.rename(new_name)

    def modify_password(self, old_password, new_password):
        self._backend.modify_password(old_password, new_password)

    def modify_network(self, settings):
        self._backend.modify_network(settings)

    def get_wifi_list(self):
        return self._backend.get_wifi_list()
