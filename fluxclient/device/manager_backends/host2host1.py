
from .base import ManagerAbstractBackend, ManagerError


def build_error(errno, **refs):
    err_message = refs.get(errno[0])
    if errno[0] == "BAD_PARAMS" and err_message is None:
        err_message = "Bad params"

    if err_message:
        return ManagerError(err_message, err_symbol=errno, **refs)
    else:
        return ManagerError(", ".join(errno), err_symbol=errno, **refs)


class Host2HostBackend1(ManagerAbstractBackend):
    channel = None

    def __init__(self, client_key, usbprotocol):
        super(Host2HostBackend1, self).__init__(
            client_key, usbprotocol.uuid, usbprotocol.serial,
            usbprotocol.model_id, usbprotocol.version, usbprotocol.nickname)
        self.usbprotocol = usbprotocol

    def connect(self):
        self.channel = self.usbprotocol.open_channel("config")
        self._authorized = True
        self.endpoint = "USB:%i@%i" % (self.usbprotocol._dev.address,
                                       self.channel.index)

    def connected(self):
        return self.channel is not None

    def close(self):
        if self.channel:
            self.channel.close()
            self.channel = None

    def add_trust(self, label, pem):
        self.channel.send_object({"cmd": "add_trust",
                                  "options": {"label": label, "pem": pem}})
        ret = self.channel.get_object()
        if ret.get("status") != "ok":
            raise build_error(ret.get("error"),
                              OPERATION_ERROR="Key already in list")

    def list_trust(self):
        self.channel.send_object({"cmd": "list_trust"})
        results = []
        ret = self.channel.get_object()
        while ret["status"] == "data":
            results.append(ret["data"])
            ret = self.channel.get_object()
        if ret.get("status") == "ok":
            return results
        else:
            raise build_error(ret.get("error"))

    def remove_trust(self, access_id):
        self.channel.send_object({"cmd": "remove_trust",
                                  "options": {"access_id": access_id}})
        ret = self.channel.get_object()
        if ret.get("status") != "ok":
            raise build_error(ret.get("error"),
                              NOT_FOUND="Access id not exist")

    def set_nickname(self, nickname):
        self.channel.send_object({"cmd": "set_nickname",
                                  "options": {"nickname": nickname}})
        ret = self.channel.get_object()
        if ret.get("status") != "ok":
            raise build_error(ret.get("error"))

    def reset_password(self, new_passwd):
        self.channel.send_object({"cmd": "reset_password",
                                  "options": {"password": new_passwd}})
        ret = self.channel.get_object()
        if ret.get("status") != "ok":
            raise build_error(ret.get("error"))

    def set_password(self, old_passwd, new_passwd, reset_acl):
        self.channel.send_object({"cmd": "set_password",
                                  "options": {"oldpasswd": old_passwd,
                                              "password": new_passwd,
                                              "clear_acl": reset_acl}})
        ret = self.channel.get_object()
        if ret.get("status") != "ok":
            raise build_error(ret.get("error"), AUTH_ERROR="Bad password")

    def set_network(self, **network_options):
        self.channel.send_object({"cmd": "set_network",
                                  "options": network_options})
        ret = self.channel.get_object()
        if ret.get("status") != "ok":
            raise build_error(ret.get("error"))

    def scan_wifi_access_points(self):
        self.channel.send_object({"cmd": "scan_wifi"})
        results = []
        ret = self.channel.get_object()
        while ret["status"] == "data":
            results.append(ret["data"])
            ret = self.channel.get_object()
        if ret.get("status") == "ok":
            return results
        else:
            raise build_error(ret.get("error"))

    def get_wifi_ssid(self):
        self.channel.send_object({"cmd": "get_wifi"})
        ret = self.channel.get_object()
        if ret.get("status") == "ok":
            return ret["result"]["ssid"]
        else:
            raise build_error(ret.get("error"))

    def get_ipaddr(self):
        self.channel.send_object({"cmd": "get_ipaddr"})
        ret = self.channel.get_object()
        if ret.get("status") == "ok":
            return ret["result"]["ipaddrs"]
        else:
            raise build_error(ret.get("error"))
