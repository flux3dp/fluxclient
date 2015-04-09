
from time import time
import binascii
import struct

from fluxclient.upnp_base import UpnpBase


CODE_NOPWD_ACCESS = 0x04
CODE_RESPONSE_NOPWD_ACCESS = 0x05

CODE_PWD_ACCESS = 0x06
CODE_RESPONSE_PWD_ACCESS = 0x07

CODE_CHANGE_PWD = 0x08
CODE_RESPONSE_CHANGE_PWD = 0x09

CODE_SET_NETWORK = 0x0a
CODE_RESPONSE_SET_NETWORK = 0x0b


class UpnpTask(UpnpBase):
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
        request = self.sign_request(message)

        return self.make_request(req_code, resp_code, request)

    def config_network(self, options):
        req_code = CODE_SET_NETWORK
        resp_code = CODE_RESPONSE_SET_NETWORK

        message = "\x00".join(("%s=%s" % i for i in options.items()))
        request = self.sign_request(message)

        return self.make_request(req_code, resp_code, request)

    def require_auth(self, timeout=3):
        start_at = time()
        while timeout >= (time() - start_at):
            resp = self.auth_without_password()
            if resp and resp.get("status") != "ok":
                raise RuntimeError("Auth failed")
