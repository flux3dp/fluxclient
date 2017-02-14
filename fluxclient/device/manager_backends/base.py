
import abc

__all__ = ["ManagerAbstractBackend"]


class ManagerAbstractBackend(object):
    __metaclass__ = abc.ABCMeta
    _authorized = False

    def __init__(self, client_key, uuid, serial, model_id, version,
                 nickname=None):
        self.client_key = client_key
        self.uuid = uuid
        self.serial = serial
        self.model_id = model_id
        self.version = version
        self.nickname = nickname

    @classmethod
    def support_device(cls, model_id, version):
        return False

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

    def authorize_with_password(self, password):
        raise NotSupportError(self.model_id, self.version)

    def add_trust(self, label, pem):
        raise NotSupportError(self.model_id, self.version)

    def list_trust(self):
        raise NotSupportError(self.model_id, self.version)

    def remove_trust(self, access_id):
        raise NotSupportError(self.model_id, self.version)

    def set_nickname(self, nickname):
        raise NotSupportError(self.model_id, self.version)

    def set_password(self, old_passwd, new_passwd, reset_acl):
        raise NotSupportError(self.model_id, self.version)

    def reset_password(self, new_passwd):
        raise NotSupportError(self.model_id, self.version)

    def set_network(self, **network_options):
        raise NotSupportError(self.model_id, self.version)

    def scan_wifi_access_points(self):
        raise NotSupportError(self.model_id, self.version)

    def get_wifi_ssid(self):
        raise NotSupportError(self.model_id, self.version)

    def get_ipaddr(self):
        raise NotSupportError(self.model_id, self.version)


class ManagerError(RuntimeError):
    """When a request can not be processed, ManagerError will be raised"""
    def __init__(self, *args, **kw):
        super(ManagerError, self).__init__(*args)
        if "err_symbol" in kw:
            self.err_symbol = kw["err_symbol"]
        else:
            self.err_symbol = ("UNKNOWN_ERROR", )


class ManagerException(Exception):
    """When manage session got a fatel error, ManagerException will be raised \
and the UpnpTask instance should be closed and can not be used anymore."""

    def __init__(self, *args, **kw):
        super(ManagerException, self).__init__(*args)
        if "err_symbol" in kw:
            self.err_symbol = kw["err_symbol"]
        else:
            self.err_symbol = ("UNKNOWN_ERROR", )


def NotSupportError(model_id, version, text=None):  # noqa
    if text:
        return ManagerError(text, err_symbol=("NOT_SUPPORT", ))
    else:
        return ManagerError(
            "Device '%s' with '%s' is not supported" % (model_id, version),
            err_symbol=("NOT_SUPPORT", ))


def AuthError(reason):  # noqa
    return ManagerError(reason, err_symbol=("AUTH_ERROR",))


def TimeoutError():  # noqa
    return ManagerException("Connection timeout", err_symbol=("TIMEOUT", ))


def ConnectionBroken():  # noqa
    return ManagerException("Connection broken",
                            err_symbol=("CONNECTION_BROKEN", ))


def BadProtocol(txt="Protocol broken"):  # noqa
    return ManagerException(txt, err_symbol=("BAD_PROTOCOL"))
