
from select import select
from time import time
from uuid import UUID
import platform
import logging
import struct
import json

from serial import Serial

from serial.serialutil import SerialTimeoutException
from serial.serialutil import SerialException

from fluxclient.utils.version import StrictVersion
from fluxclient.encryptor import KeyObject
from .base import (ManagerAbstractBackend, ManagerException, ManagerError,
                   NotSupportError, TimeoutError, BadProtocol)

logger = logging.getLogger(__name__)

CODE_DISCOVER = 0x00
CODE_RSAKEY = 0x01
CODE_AUTH = 0x02
CODE_CONFIG_GENERAL = 0x03
CODE_CONFIG_NETWORK = 0x04
CODE_GET_SSID = 0x05
CODE_LIST_SSID = 0x08
CODE_GET_IPADDR = 0x07
CODE_RESET_PASSWD = 0x06


def is_windows():
    return platform.platform().startswith("Windows")


class UartBackend(ManagerAbstractBackend):
    _authorized = True
    s = None

    def __init__(self, client_key, port, baudrate=115200):
        # Select does not work with windows..
        super(UartBackend, self).__init__(
            client_key, None, None, None, None)

        self.port = port
        self.baudrate = baudrate

    def connect(self):
        try:
            if is_windows():
                self.s = Serial(port=self.port, baudrate=self.baudrate,
                                timeout=0.1)
            else:
                self.s = Serial(port=self.port, baudrate=self.baudrate,
                                timeout=0)
        except SerialException as e:
            raise ManagerException(*e.args, err_symbol=("DEVICE_ERROR", ))

        self.s.write(b"\x00" * 16)
        if is_windows():
            try:
                self.s.readall()  # Normally returns with empty string
            except SerialTimeoutException:
                logger.error("Serial timeout")
        else:
            while True:
                rl = select((self.s, ), (), (), 0.1)[0]
                if rl:
                    self.s.readall()
                else:
                    break
        self._discover()

    def _discover(self):
        resp = self._make_request(CODE_DISCOVER, timeout=0.25)
        info = {}
        for pair in resp.split(b"\x00"):
            spair = pair.decode("utf8", "ignore").split("=", 1)
            if len(spair) == 1:
                logger.error("Can not parse device info: %s" % pair)
            info[spair[0]] = spair[1]

        self.uuid = UUID(hex=info["uuid"])
        self.serial = info["serial"]
        self.model_id = info["model"]
        self.version = StrictVersion(info["ver"])
        self.nickname = info["name"]
        self.endpoint = "UART:%s" % self.port

        rsakey = self._make_request(CODE_RSAKEY)
        self.device_rsakey = KeyObject.load_keyobj(rsakey)

    @property
    def connected(self):
        return self.s is not None

    def close(self):
        self.s.close()
        self.s = None

    def _make_request(self, code, buf=b"", timeout=6.0):
        try:
            header = struct.pack("<3sHH", b'\x97\xae\x02', code, len(buf))
            payload = header + buf
            self.s.write(payload)

            ttl = time() + timeout
            resp = b""
            while time() < ttl:
                rl = False
                if is_windows():
                    # In windows, the timeout=0.1 works like select
                    rl = True
                else:
                    rl = select((self.s, ), (), (), max(time() - ttl, 0))[0]
                if rl:
                    resp += self.s.readall()
                    if self._try_parse_response(code, resp):
                        status = struct.unpack("<b", resp[7:8])[0]
                        if status == 1:
                            return resp[8:]
                        else:
                            m = resp[8:].decode("ascii", "ignore")
                            raise ManagerError("Operation error",
                                               "status: %i" % status,
                                               err_symbol=m.split(" "))

            raise TimeoutError()
        except SerialException as e:
            logger.exception("UART device error")
            raise ManagerException(*e.args, err_symbol="DEVICE_ERROR")

    def _try_parse_response(self, code, buf):
        buf_length = len(buf)

        if buf_length < 8:
            return False

        mn, rcode, length, status = struct.unpack("<3sHHb", buf[:8])
        if mn != b'\x97\xae\x02':
            raise BadProtocol("BAD_PROTOCOL")
        if rcode != code:
            raise BadProtocol("Should get %i but get %i" % (code, rcode))

        total_length = length + 8

        if buf_length < total_length:
            return False
        elif buf_length == total_length:
            return True
        else:
            raise BadProtocol("Response too long")

    def set_nickname(self, nickname):
        message = "name=%s" % nickname
        ret = self._make_request(CODE_CONFIG_GENERAL, message.encode())
        return ret.decode("ascii", "ignore")

    def reset_password(self, new_passwd):
        ret = self._make_request(
            CODE_RESET_PASSWD,
            new_passwd.encode() + b"\x00" + self.client_key.public_key_pem)
        return ret.decode("utf8", "ignore")

    def set_password(self, old_passwd, new_passwd, reset_acl):
        if not reset_acl:
            raise NotSupportError("reset_acl can not be false")
        return self.reset_password(new_passwd)

    def add_trust(self, label, pem, timeout=6.0):
        ret = self._make_request(CODE_AUTH, pem.encode(),
                                 timeout=timeout)
        if ret == b"OK":
            pass
        elif ret == b"ALREADY_TRUSTED":
            raise ManagerError("Key already in list",
                               err_symbol=("OPERATION_ERROR", ))
        else:
            raise ManagerError(ret.decode("ascii", "ignore"))

    def set_network(self, **network_options):
        message = "\x00".join(("%s=%s" % i for i in network_options.items()))
        ret = self._make_request(CODE_CONFIG_NETWORK, message.encode())
        return ret.decode("ascii", "ignore")

    def get_wifi_ssid(self):
        return self._make_request(CODE_GET_SSID,
                                  timeout=15.0).decode("utf8", "ignore")

    def scan_wifi_access_points(self):
        doc = self._make_request(CODE_LIST_SSID,
                                 timeout=15.0).decode("utf8", "ignore")
        return json.loads(doc)

    def get_ipaddr(self):
        ret = self._make_request(CODE_GET_IPADDR,
                                 timeout=15.0).decode("utf8", "ignore")
        if ret:
            return ret.split(" ")
        else:
            return []
