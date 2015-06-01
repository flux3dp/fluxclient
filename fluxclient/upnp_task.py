
from time import time
import binascii
import struct
import json

from fluxclient.upnp_base import UpnpBase


CODE_NOPWD_ACCESS = 0x04
CODE_RESPONSE_NOPWD_ACCESS = 0x05

CODE_PWD_ACCESS = 0x06
CODE_RESPONSE_PWD_ACCESS = 0x07

CODE_CHANGE_PWD = 0xa0
CODE_RESPONSE_CHANGE_PWD = 0xa1

CODE_SET_NETWORK = 0xa2
CODE_RESPONSE_SET_NETWORK = 0xa3

CODE_CONTROL_STATUS = 0x80
CODE_RESPONSE_CONTROL_STATUS = 0x81

CODE_RESET_CONTROL = 0x82
CODE_RESPONSE_RESET_CONTROL = 0x83

CODE_REQUEST_ROBOT = 0x84
CODE_RESPONSE_REQUEST_ROBOT = 0x85

class UpnpTask(UpnpBase):
    def auth_with_password(self, passwd, timeout=1.2):
        der = self.publickey_der

        req_code = CODE_PWD_ACCESS
        resp_code = CODE_RESPONSE_PWD_ACCESS

        buf = b"\x00".join([
            str(self.create_timestemp()).encode(),
            passwd.encode(),
            der
        ])
        resp = self.make_request(req_code, resp_code, buf, encrypt=True)
        return resp

    def auth_without_password(self, timeout=1.2):
        der = self.publickey_der

        req_code = CODE_NOPWD_ACCESS
        resp_code = CODE_RESPONSE_NOPWD_ACCESS
        msg = struct.pack("<d%ss" % len(der), self.create_timestemp(), der)

        resp = self.make_request(req_code, resp_code, msg,
                                 encrypt=False, timeout=timeout)
        if resp and resp.get("status") == "ok":
            self.access_id = binascii.a2b_hex(resp.get("access_id"))
        return resp

    def passwd(self, password, old_password=""):
        req_code = CODE_CHANGE_PWD
        resp_code = CODE_RESPONSE_CHANGE_PWD

        message = "\x00".join((password, old_password))
        request = self.sign_request(message.encode())

        return self.make_request(req_code, resp_code, request)

    def config_network(self, options):
        req_code = CODE_SET_NETWORK
        resp_code = CODE_RESPONSE_SET_NETWORK

        message = "\x00".join(("%s=%s" % i for i in options.items()))
        request = self.sign_request(message.encode())

        return self.make_request(req_code, resp_code, request)

    def kill_control(self):
        req_code = CODE_CONTROL_STATUS
        resp_code = CODE_RESPONSE_CONTROL_STATUS

        message = b""
        request = self.sign_request(message.encode())

        return self.make_request(req_code, resp_code, request)

    def require_robot(self, timeout=6):
        req_code = CODE_REQUEST_ROBOT
        resp_code = CODE_RESPONSE_REQUEST_ROBOT
        start_at = time()

        while timeout >= (time() - start_at):
            request = self.sign_request(b"")
            resp = self.make_request(req_code, resp_code, request)
            if resp:
                return resp

        raise RuntimeError("Timeout")


    def require_auth(self, timeout=6):
        start_at = time()
        while timeout >= (time() - start_at):
            resp = self.auth_without_password()
            if resp:
                if resp.get("status") == "ok":
                    return resp
                else:
                    raise RuntimeError("Auth failed")

        raise RuntimeError("Timeout")
