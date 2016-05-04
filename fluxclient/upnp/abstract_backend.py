
import abc

__all__ = ["UpnpAbstractBackend"]


class UpnpAbstractBackend(object):
    __metaclass__ = abc.ABCMeta
    _authorized = False

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

    @abc.abstractproperty
    def authorized(self):
        return self._authorized

    @abc.abstractmethod
    def connect(self):
        pass

    @abc.abstractproperty
    def connected(self):
        pass

    @abc.abstractmethod
    def close(self):
        pass

    @abc.abstractmethod
    def authorize_with_password(self, password):
        pass

    @abc.abstractmethod
    def add_trust(self, label, pem):
        pass

    @abc.abstractmethod
    def list_trust(self):
        pass

    @abc.abstractmethod
    def remove_trust(self, access_id):
        pass

    @abc.abstractmethod
    def rename(self, new_device_name):
        pass

    @abc.abstractmethod
    def modify_password(self, old_password, new_password, reset_acl):
        pass

    @abc.abstractmethod
    def modify_network(self, network_options):
        pass

    @abc.abstractmethod
    def get_wifi_list(self):
        pass


class UpnpError(RuntimeError):
    def __init__(self, *args, **kw):
        super(UpnpError, self).__init__(*args)
        if "err_symbol" in kw:
            self.err_symbol = kw["err_symbol"]
        else:
            self.err_symbol = ("UNKNOWN_ERROR", )


class UpnpException(Exception):
    def __init__(self, *args, **kw):
        super(UpnpException, self).__init__(*args)
        if "err_symbol" in kw:
            self.err_symbol = kw["err_symbol"]
        else:
            self.err_symbol = ("UNKNOWN_ERROR", )


def NotSupportError(model_id, version):  # noqa
    return UpnpError(
        "Device '%s' with '%s' is not supported" % (model_id, version),
        err_symbol=("NOT_SUPPORT", ))


def AuthError(reason):  # noqa
    return UpnpError(reason, err_symbol=("AUTH_ERROR",))


def TimeoutError():  # noqa
    return UpnpException("Connection timeout", err_symbol=("TIMEOUT", ))


def ConnectionBroken():  # noqa
    return UpnpException("Connection broken",
                         err_symbol=("CONNECTION_BROKEN", ))
