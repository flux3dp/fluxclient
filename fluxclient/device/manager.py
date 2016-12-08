
import logging

from fluxclient.utils.version import StrictVersion
from fluxclient.device.discover import DeviceDiscover
from .manager_backends import (ManagerError, ManagerException, NotSupportError,
                               NETWORK_BACKENDS)

__all__ = ["DeviceManager", "ManagerError", "ManagerException"]

logger = logging.getLogger(__name__)


class DeviceManager(object):
    """DeviceManager provides some configuration methods for the device. When \
creating a DeviceManager instance, the argument **uuid** is required. If \
parameter **device_metadata** is not given, DeviceManager will use \
lookup_callback and lookup_timeout to create a DeviceDiscover instance and \
try to get metadata from network.

    :param uuid.UUID uuid: Device uuid, set UUID(int=0) while trying to \
connect via ip address.
    :param encrypt.KeyObject client_key: Client key to connect to device.
    :param str ipaddr: IP Address of the machine.
    :param dict device_metadata: This is an internal parameter, which is not \
recommended to provide because it may has different definitions in \
different versions.
    :param dict backend_options: More configuration for DeviceManager.
    :param callable lookup_callback: Invoke repeatedly while looking for \
device.
    :param float lookup_timeout: Raise an error if the program can not find \
the device in a limited time.
    :raises ManagerError: For protocol or operation error.
    :raises socket.error: For system defined socket error.
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
            if 'uuid' in device_metadata:
                device_metadata.pop('uuid')
            self.update_remote_profile(uuid, **device_metadata)
        elif remote_profile:
            self.update_remote_profile(uuid, **remote_profile)
        else:
            self.reload_remote_profile(lookup_callback, lookup_timeout)

        self.initialize_backend()

    def reload_remote_profile(self, lookup_callback=None,
                              lookup_timeout=float("INF")):
        def on_discovered(instance, device, **kw):
            self.update_remote_profile(**(device.to_old_dict()))
            instance.stop()

        if self.uuid.int:
            d = DeviceDiscover(uuid=self.uuid)
        else:
            d = DeviceDiscover(device_ipaddr=self.ipaddr)

        d.discover(on_discovered, lookup_callback, lookup_timeout)

    def update_remote_profile(self, uuid, name, serial, model_id, version,
                              ipaddr, **meta):
        if not self.uuid or self.uuid.int == 0:
            self.uuid = uuid
        self.name = name
        self.serial = serial
        self.model_id = model_id
        self.version = StrictVersion(str(version))
        self.ipaddr = ipaddr
        self.device_meta = meta

    def initialize_backend(self):
        for klass in NETWORK_BACKENDS:
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
        """Closes the manager socket connection. After close(), any other \
method should not be called anymore."""

        self._backend.close()

    @property
    def authorized(self):
        "Indicates whether the connection has been authorized with a correct \
password or RSA key. If the connection is not authorized, you must call \
`authorize_with_password` first to authorize."

        return self._backend.authorized

    @property
    def connected(self):
        """Indicates whether the manager connection is connected with the \
device"""
        return self._backend.connected

    def authorize_with_password(self, password):
        """Authorizes via password, only use when the RSA key has not been \
trusted from device.

        :param str password: Device password"""

        if not self._backend.connected:
            raise ManagerError("Disconnected")
        if self._backend.authorized:
            raise ManagerError("Already authorized")
        self._backend.authorize_with_password(password)

    def add_trust(self, label, key):
        """Adds a client_key to device trust list

        :param str label: Key label will show for human only
        :param object key: A vaild RSA key object or pem
        :return: Key hash
        :rtype: str"""

        if isinstance(key, str):
            pem = key
        elif isinstance(key, bytes):
            pem = key.decode("ascii")
        else:
            pem = key.public_key_pem.decode("ascii")

        self._backend.add_trust(label, pem)

    def list_trust(self):
        """Gets all trusted key in the device

        :return: ((label, key hash), (label, key hash), ...)"""

        return self._backend.list_trust()

    def remove_trust(self, access_id):
        """Removes a trusted key

        :param str access_id: Key hash which will be removed"""

        return self._backend.remove_trust(access_id)

    def rename(self, new_name):
        """Renames the device

        :param str new_name: New device name"""

        if not self._backend.connected:
            raise ManagerError("Disconnected")
        if not self._backend.authorized:
            raise ManagerError("Authorize required")
        self._backend.rename(new_name)

    def modify_password(self, old_password, new_password, reset_acl=True):
        """Changes the device password, if **reset_acl** set to True, all other \
authorized user will be deauthorized.

        :param str old_password: Old device password
        :param str new_password: New device password
        :param bool reset_acl: Clear authorized user list in device"""

        if not self._backend.connected:
            raise ManagerError("Disconnected")
        if not self._backend.authorized:
            raise ManagerError("Authorize required")
        self._backend.modify_password(old_password, new_password, reset_acl)

    def modify_network(self, **settings):
        """Modifies the device network, details will be revealed in future \
documentation."""

        if not self._backend.connected:
            raise ManagerError("Disconnected")
        if not self._backend.authorized:
            raise ManagerError("Authorize required")
        self._backend.modify_network(**settings)

    def get_wifi_list(self):
        """Gets wifi lists discovered from the device"""

        if not self._backend.connected:
            raise ManagerError("Disconnected")
        if not self._backend.authorized:
            raise ManagerError("Authorize required")
        return self._backend.get_wifi_list()
