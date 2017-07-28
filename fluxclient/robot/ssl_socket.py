
from pyasn1.codec.der import decoder as cert_decoder
from binascii import a2b_hex as from_hex
from hashlib import sha1
from hmac import HMAC
import logging
import ssl

from .errors import RobotSessionError

__all__ = ["SSLSocket"]
__READY__ = 0
__WAIT_REMOTE__ = 4 + 1
__SENDING_IDENTIFY__ = 8 + 2
__WAIT_ACK__ = 16 + 1

logger = logging.getLogger(__name__)


def parse_cert(der_cert):
    def wolker(node, dataset, oi=None):
        if node.typeId in (3, 4):
            for e in node:
                oi = wolker(e, dataset, oi)
            return oi

        elif node.typeId is None:
            if node.tagSet.uniq == (0, 6):
                return node.asTuple()

            elif node.tagSet.uniq == (0, 12):
                if oi == (2, 5, 4, 3):
                    values = str(node).split(":")
                    if len(values) == 3:
                        dataset["uuid"] = values[0]
                        dataset["serial"] = values[1]
                        dataset["identify"] = values[2]

            elif node.tagSet.uniq == (0, 3):
                if oi == (1, 2, 840, 113549, 1, 1, 1):
                    buf = []
                    offset = 7
                    val = 0
                    for b in node:
                        if offset:
                            val += b << offset
                            offset -= 1
                        else:
                            buf.append(val + b)
                            offset = 7
                            val = 0

                    dataset["publickey"] = bytes(buf)

            else:
                return oi

    rootdoc = cert_decoder.decode(der_cert)[0]
    dataset = {}
    wolker(rootdoc, dataset)

    return dataset


class SSLSocket(ssl.SSLSocket):
    server_key = None
    client_key = None

    def __init__(self, sock, client_key=None, device=None,
                 ignore_key_validation=False):
        super(SSLSocket, self).__init__(sock, do_handshake_on_connect=False)

        self.__buffer = bytearray(4096)
        self.__bufferv = memoryview(self.__buffer)
        self.__buffered = 0
        self.__handshake_flag = __WAIT_REMOTE__

        self._validation = not ignore_key_validation

        if hasattr(device, "master_key"):
            self.server_key = device.master_key

        self.client_key = client_key

    def __read_buffer(self, length):
        if length <= self.__buffered:
            return

        l = self.recv_into(self.__bufferv[self.__buffered:length])
        if l:
            self.__buffered += l
            return l
        else:
            raise RobotSessionError("Broken pipe")

    def do_handshake(self):
        try:
            super(SSLSocket, self).do_handshake()
        except ssl.SSLWantReadError:
            return 1
        except ssl.SSLWantWriteError:
            return 2

        return self._do_flux_handshake()

    def _do_flux_handshake(self):
        if self.__handshake_flag == __WAIT_REMOTE__:
            self.__read_buffer(64)
            if self.__buffered == 64:
                bincert = self.getpeercert(True)
                cert = parse_cert(bincert)  # noqa
                # TODO: check cert

                binuuid = from_hex(cert["uuid"])
                tosign = HMAC(binuuid, self.__bufferv[:64].tobytes(), sha1).digest()
                access_id = self.client_key.get_access_id(binary=True)

                self.__buffered = 0
                self.__bufferv[:20] = access_id
                self.__bufferv[20:20 + self.client_key.size] = \
                    self.client_key.sign(tosign)
                self.__handshake_flag = __SENDING_IDENTIFY__
            else:
                return __WAIT_REMOTE__

        if self.__handshake_flag == __SENDING_IDENTIFY__:
            message_length = 20 + self.client_key.size
            # TODO!
            self.send(self.__bufferv[:message_length])

            self.__buffered = 0
            self.__handshake_flag = __WAIT_ACK__
            return __WAIT_ACK__

        if self.__handshake_flag == __WAIT_ACK__:
            self.__read_buffer(16)
            if self.__buffered == 16:
                st = self.__buffer[:16].\
                    decode("ascii", "ignore").rstrip(" \x00")
                if st == "OK":
                    logger.debug("Client identify successed")
                    self.__buffered = None
                    self.__handshake_flag = __READY__
                else:
                    raise RobotSessionError(
                        "Handshake failed (%s)" % st,
                        error_symbol=st.split(" "))
            else:
                return __WAIT_ACK__

        return __READY__
