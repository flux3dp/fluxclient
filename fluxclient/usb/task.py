
from select import select
from time import time
from uuid import UUID
import logging
import struct

from serial import Serial

from fluxclient import encryptor as E

logger = logging.getLogger(__name__)

CODE_DISCOVER = 0x00
CODE_RSAKEY = 0x01
CODE_AUTH = 0x02
CODE_CONFIG_GENERAL = 0x03
CODE_CONFIG_NETWORK = 0x04
CODE_GET_SSID = 0x05
CODE_GET_IPADDR = 0x07
CODE_SET_PASSWORD = 0x06


class UsbTask(object):
    def __init__(self, port, baudrate=115200):
        self.s = Serial(port=port, baudrate=115200, timeout=0)
        self.s.write(b"\x00" * 16)

        while True:
            rl = select((self.s, ), (), (), 0.1)[0]
            if rl:
                self.s.readall()
            else:
                break

        self.keyobj = E.get_or_create_keyobj()
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
        self.timedelta = time() - float(info["time"])
        self.remote_version = info["ver"]
        self.has_password = True if int(info["pwd"]) == 1 else False
        self.name = info["name"]
        self.remote_addrs = None

        rsakey = self._make_request(CODE_RSAKEY)
        self.device_rsakey = E.load_keyobj(rsakey)

    def _make_request(self, code, buf=b"", timeout=6.0):
        header = struct.pack("<3sHH", b'\x97\xae\x02', code, len(buf))
        payload = header + buf
        self.s.write(payload)

        ttl = time() + timeout
        resp = b""
        while time() < ttl:
            rl = select((self.s, ), (), (), max(time() - ttl, 0))[0]
            if rl:
                resp += self.s.readall()
                if self._try_parse_response(code, resp):
                    status = struct.unpack("<b", resp[7:8])[0]
                    if status == 1:
                        return resp[8:]
                    else:
                        raise UsbTaskError(resp[8:].decode("ascii", "ignore"),
                                           "status: %i" % status)
        raise UsbTaskError("TIMEOUT")

    def _try_parse_response(self, code, buf):
        buf_length = len(buf)

        if buf_length < 8:
            return False

        mn, rcode, length, status = struct.unpack("<3sHHb", buf[:8])
        if mn != b'\x97\xae\x02':
            raise UsbTaskException("BAD_PROTOCOL")
        if rcode != code:
            raise UsbTaskException("BAD_PROTOCOL",
                                   "Should get %i but get %i" % (code, rcode))

        total_length = length + 8

        if buf_length < total_length:
            return False
        elif buf_length == total_length:
            return True
        else:
            raise UsbTaskException("BAD_PROTOCOL",
                                   "Response too long")

    def require_auth(self, timeout=6.0):
        ret = self._make_request(CODE_AUTH,
                                 E.get_public_key_pem(self.keyobj),
                                 timeout=timeout)
        if ret not in (b"OK", b"ALREADY_TRUSTED"):
            raise RuntimeError(ret)

    def auth(self, passwd=None):
        if passwd:
            payload = b"PASSWORD" + passwd.encode() + b"\x00" + \
                      E.get_public_key_pem(self.keyobj)
        else:
            payload = E.get_public_key_pem(self.keyobj)

        return self._make_request(CODE_AUTH, payload)

    def config_general(self, options):
        message = "\x00".join(("%s=%s" % i for i in options.items()))
        ret = self._make_request(CODE_CONFIG_GENERAL, message.encode())
        return ret.decode("ascii", "ignore")

    def config_network(self, options):
        message = "\x00".join(("%s=%s" % i for i in options.items()))
        ret = self._make_request(CODE_CONFIG_NETWORK, message.encode())
        return ret.decode("ascii", "ignore")

    def set_password(self, passwd):
        ret = self._make_request(
            CODE_SET_PASSWORD,
            passwd.encode() + b"\x00" + E.get_public_key_pem(self.keyobj))
        return ret.decode("utf8", "ignore")

    def get_ssid(self):
        return self._make_request(CODE_GET_SSID).decode("utf8", "ignore")

    def get_ipaddr(self):
        ret = self._make_request(CODE_GET_IPADDR).decode("utf8", "ignore")
        if ret:
            return ret.split(" ")
        else:
            return []

    def close(self):
        self.s.close()


class UsbTaskError(RuntimeError):
    pass


class UsbTaskException(Exception):
    pass
