
import logging

from .manager_backends import (ManagerError, ManagerException,
                               Host2HostBackend1, UartBackend,
                               get_backend_via_ipaddr, get_backend_via_uuid,
                               get_backend_via_network)

__all__ = ["DeviceManager", "ManagerError", "ManagerException"]

logger = logging.getLogger(__name__)


class DeviceManager(object):
    """DeviceManager provides configuration methods for the device. Use the
following class methods to create a new manager instance:
    `fluxclient.device.manager.DeviceManager.from_uuid`
    `fluxclient.device.manager.DeviceManager.from_ipaddr`
    `fluxclient.device.manager.DeviceManager.from_device`
    """

    _backend = None

    def __init__(self, backend):
        backend.connect()
        self._backend = backend
        self.client_key = backend.client_key

    @classmethod
    def from_uuid(cls, client_key, uuid, lookup_callback=None,
                  lookup_timeout=float("INF")):
        """Manage device by uuid

        :param uuid.UUID uuid: Device uuid"""
        klass, device = get_backend_via_uuid(uuid, lookup_callback,
                                             lookup_timeout)
        backend = klass(client_key, device)
        return cls(backend)

    @classmethod
    def from_ipaddr(cls, client_key, ipaddr, lookup_callback=None,
                    lookup_timeout=float("INF")):
        """Manage device by ip address

        :param str ipaddr: IP Address of the machine"""
        klass, device = get_backend_via_ipaddr(ipaddr, lookup_callback,
                                               lookup_timeout)
        backend = klass(client_key, device)
        return cls(backend)

    @classmethod
    def from_device(cls, client_key, device):
        """Manage device by device object

        :param fluxclient.device.device.Device device: Device object"""
        klass = get_backend_via_network(device)
        backend = klass(client_key, device)
        return cls(backend)

    @classmethod
    def from_usb(cls, client_key, usbprotocol):
        backend = Host2HostBackend1(client_key, usbprotocol)
        return cls(backend)

    @classmethod
    def from_uart(cls, client_key, port, baudrate=115200):
        backend = UartBackend(client_key, port, baudrate)
        return cls(backend)

    def close(self):
        """Closes the manager socket connection. After close(), any other \
method should not be called anymore."""

        self._backend.close()

    @property
    def authorized(self):
        "Indicates whether the connection has been authorized with a correct \
password or RSA key. If the connection is not authorized, you must call \
`authorize_with_password` first to authorize."

        return self._backend.authorized()

    @property
    def serial(self):
        return self._backend.serial

    @property
    def uuid(self):
        return self._backend.uuid

    @property
    def version(self):
        return self._backend.version

    @property
    def model_id(self):
        return self._backend.model_id

    @property
    def nickname(self):
        return self._backend.nickname

    @property
    def endpoint(self):
        return self._backend.endpoint

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
        if self._backend.authorized():
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

    def _check_status(self):
        if not self._backend.connected:
            raise ManagerError("Disconnected")
        if not self._backend.authorized:
            raise ManagerError("Authorize required")

    def list_trust(self):
        """Gets all trusted key in the device

        :return: ((label, key hash), (label, key hash), ...)"""

        self._check_status()
        return self._backend.list_trust()

    def remove_trust(self, access_id):
        """Removes a trusted key

        :param str access_id: Key hash which will be removed"""

        self._check_status()
        return self._backend.remove_trust(access_id)

    def set_nickname(self, nickname):
        """Renames the device

        :param str nickname: Change device nickname"""

        self._check_status()
        self._backend.set_nickname(nickname)

    def set_password(self, old_password, new_password, reset_acl=True):
        """Changes the device password, if **reset_acl** set to True, all other \
authorized user will be deauthorized.

        :param str old_password: Old device password
        :param str new_password: New device password
        :param bool reset_acl: Clear authorized user list in device"""

        self._check_status()
        self._backend.set_password(old_password, new_password, reset_acl)

    def reset_password(self, new_password):
        """Reset device password, some of device informations will be deleted.

        :param str old_password: Old device password
        :param str new_password: New device password
        :param bool reset_acl: Clear authorized user list in device"""
        self._check_status()
        self._backend.reset_password(new_password)

    def set_network(self, **network_options):
        """Modifies the device network, details will be revealed in future \
documentation."""

        self._check_status()
        self._backend.set_network(**network_options)

    def scan_wifi_access_points(self):
        """Gets wifi lists discovered from the device"""

        self._check_status()
        return self._backend.scan_wifi_access_points()

    def get_wifi_ssid(self):
        """Gets wifi ssid connected, return None if not connected"""

        self._check_status()
        return self._backend.get_wifi_ssid()

    def get_ipaddr(self):
        """"Gets device ipaddr, return a list"""

        self._check_status()
        return self._backend.get_ipaddr()

    def rename(self, new_name):
        return self.set_nickname(new_name)

    def modify_password(self, old_password, new_password, reset_acl=True):
        return self.set_password(old_password, new_password, reset_acl)

    def modify_network(self, **settings):
        return self.set_network(**settings)

    def get_wifi_list(self):
        return self.scan_wifi_access_points()
