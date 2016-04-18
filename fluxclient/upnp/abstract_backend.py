
import abc

__all__ = ["UpnpAbstractBackend"]


class UpnpAbstractBackend(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, client_key, uuid, version, model_id, ipaddr,
                 metadata=None, options=None):
        self.client_key = client_key
        self.uuid = uuid
        self.version = version
        self.model_id = model_id
        self.ipaddr = ipaddr

    @classmethod
    def support_device(cls, model_id, version):
        return False

    @abc.abstractmethod
    def connect(self):
        pass

    @abc.abstractmethod
    def close(self):
        pass

    @abc.abstractmethod
    def rename(self, new_device_name):
        pass

    @abc.abstractmethod
    def modify_password(self, old_password, new_password):
        pass

    @abc.abstractmethod
    def modify_network(self, network_options):
        pass

    @abc.abstractmethod
    def get_wifi_list(self):
        pass


class UpnpError(RuntimeError):
    error_label = "UNKNOWN_ERROR"


class NotSupportError(UpnpError):
    error_label = "NOT_SUPPORT"

    def __init__(self, model_id, version):
        super(NotSupportError, self).__init__(
            "Device '%s' with '%s' is not supported" % (model_id, version))


class AuthError(UpnpError):
    error_label = "AUTH_ERROR"


class TimeoutError(UpnpError):
    error_label = "TIMEOUT"
