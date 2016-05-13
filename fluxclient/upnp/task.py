
import logging

from fluxclient.utils.version import StrictVersion
from fluxclient.upnp.discover import UpnpDiscover
from .abstract_backend import UpnpError, UpnpException, NotSupportError
from .udp1_backend import UpnpUdp1Backend
from .ssl1_backend import UpnpSSL1Backend

__all__ = ["UpnpTask", "UpnpError", "UpnpException"]

BACKENDS = [
    UpnpSSL1Backend,
    UpnpUdp1Backend]

logger = logging.getLogger(__name__)


class UpnpTask(object):
    """UpnpTask provide basic configuration method to device

    :param uuid.UUID uuid: Device uuid, set UUID(int=0) while trying connect \
via ip address.
    :param encrypt.KeyObject client_key: Client key to connect to device.
    :param str ipaddr: IP Address to connect to.
    :param dict device_metadata: Device metadata
    :param dict backend_options: More configuration for UpnpTask
    :param callable lookup_callback: Invoke repeated while finding device
    :param float lookup_timeout: Raise error if device can not be found after \
timeout value
    :raises UpnpError: For protocol or operation error
    :raises socket.error: For socket error
"""
    name = None
    uuid = None
    serial = None
    model_id = None
    version = None
    ipaddr = None
    meta = None

    _backend = None

    def __init__(self, uuid, client_key, ipaddr=None, device_metadata=None,
                 remote_profile=None, backend_options={}, lookup_callback=None,
                 lookup_timeout=float("INF")):
        self.uuid = uuid
        self.ipaddr = ipaddr
        self.client_key = client_key
        self.backend_options = backend_options

        if device_metadata:
            self.update_remote_profile(**device_metadata)
        elif remote_profile:
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
        self.ipaddr = ipaddr
        self.device_meta = meta

    def initialize_backend(self):
        for klass in BACKENDS:
            if klass.support_device(self.model_id, self.version):
                self._backend = klass(self.client_key, self.uuid, self.version,
                                      self.model_id, self.ipaddr,
                                      self.device_meta, self.backend_options)
                # TODO: debug information, remove after bugfix
                logger.info("Backend %s selected", klass.__name__)
                return
            # TODO: debug information, remove after bugfix
            logger.warn("Backend %s does not support device version `%s`",
                        klass.__name__, self.version)

        raise NotSupportError(self.model_id, self.version)

    def close(self):
        """Close upnp socket connection. After close(), any other method \
should not be called anymore."""

        self._backend.close()

    @property
    def authorized(self):
        "Is connection authorized. If connection not authorized, it must \
call `authorize_with_password` first to complete authorize."

        return self._backend.authorized

    @property
    def connected(self):
        """Return True if Upnp is connected with device"""
        return self._backend.connected

    def authorize_with_password(self, password):
        """Authorize via password, only use when RSA key is not been trusted \
from device.

        :param str password: Device password"""

        if not self._backend.connected:
            raise UpnpError("Disconnected")
        if self._backend.authorized:
            raise UpnpError("Already authorized")
        self._backend.authorize_with_password(password)

    def add_trust(self, label, pem):
        """Add client_key to device trust list

        :param str label: Key label will show for human only
        :param str pem: A vailed RSA key pem
        :return: Key hash
        :rtype: str"""

        self._backend.add_trust(label, pem)

    def list_trust(self):
        """Get all trusted key in device

        :return: ((label, key hash), (label, key hash), ...)"""

        return self._backend.list_trust()

    def remove_trust(self, access_id):
        """Remove trusted key

        :param str access_id: Key hash which will be removed"""

        return self._backend.remove_trust(access_id)

    def rename(self, new_name):
        """Rename device

        :param str new_name: New device name"""

        if not self._backend.connected:
            raise UpnpError("Disconnected")
        if not self._backend.authorized:
            raise UpnpError("Authorize required")
        self._backend.rename(new_name)

    def modify_password(self, old_password, new_password, reset_acl=True):
        """Change device password, if **reset_acl** set to True, all other \
authorized user will be deauthorized.

        :param str old_password: Old device password
        :param str new_password: New device password
        :param bool reset_acl: Clear authorized user list in device"""

        if not self._backend.connected:
            raise UpnpError("Disconnected")
        if not self._backend.authorized:
            raise UpnpError("Authorize required")
        self._backend.modify_password(old_password, new_password, reset_acl)

    def modify_network(self, **settings):
        """Mofify device modify_network, look document for more help"""

        if not self._backend.connected:
            raise UpnpError("Disconnected")
        if not self._backend.authorized:
            raise UpnpError("Authorize required")
        self._backend.modify_network(**settings)

    def get_wifi_list(self):
        """Get wifi list discovered from device"""

        if not self._backend.connected:
            raise UpnpError("Disconnected")
        if not self._backend.authorized:
            raise UpnpError("Authorize required")
        return self._backend.get_wifi_list()
