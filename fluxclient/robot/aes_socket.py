
import logging
import socket
from Crypto.Cipher import AES

from .errors import RobotError, RobotSessionError, RobotNotReadyError

logger = logging.getLogger(__name__)

__WAIT_IDENTIFY__ = 1
__WAIT_RESPONSE__ = 2
__WAIT_AESKEY__ = 3
__READY__ = 4


class AESSocket(object):
    _sock = None
    _buffer = None
    _bufferv = None
    _buffered = 0
    _encoder = None
    _decoder = None
    __handshake_flag = None
    server_key = None
    client_key = None

    def __init__(self, sock, client_key=None, device=None,
                 ignore_key_validation=False):
        self._buffer = bytearray(4096)
        self._bufferv = memoryview(self._buffer)
        self._buffered = 0
        self._sock = sock
        self._validation = not ignore_key_validation

        if hasattr(device, "slave_key"):
            self.server_key = device.slave_key

        self.client_key = client_key
        self.__handshake_flag = __WAIT_IDENTIFY__

    def __read_buffer(self, length):
        if length <= self._buffered:
            return

        l = self._sock.recv_into(self._bufferv[self._buffered:length])

        if l:
            self._buffered += l
            return l
        else:
            raise RobotSessionError("EOF Error")

    def do_handshake(self):
        if self.__handshake_flag == __READY__:
            return 0
        elif self.__handshake_flag == __WAIT_IDENTIFY__:
            # Stage 1
            self._do_handshake_validate_remote()
            return 1
        elif self.__handshake_flag == __WAIT_RESPONSE__:
            # Stage 2
            self._do_handshake_wait_validate()
            return 2
        elif self.__handshake_flag == __WAIT_AESKEY__:
            # Stage 3
            return self._do_handshake_wait_aeskey()

    def _do_handshake_validate_remote(self):
        bufsize = (self.server_key.size + 128) if self.server_key else 4096
        self.__read_buffer(bufsize)

        if self.server_key:
            if bufsize == self._buffered:
                buf = self._bufferv[:bufsize]
                sign, randbytes = buf[:self.server_key.size], buf[-128:]

                if self._validation:
                    if not self.server_key.verify(randbytes.tobytes(),
                                                  sign.tobytes()):
                        raise RobotSessionError(
                            "Remote identify error",
                            error_symbol=("REMOTE_IDENTIFY_ERROR", ))
                else:
                    logger.warn("Warning: skip validate remote")
            else:
                return
        else:
            if self._buffered == 256:
                buf = self._buffer[:256]
                sign, randbytes = buf[:128], buf[-128:]
            else:
                return

        ck = self.client_key
        buf = ck.get_access_id(binary=True) + ck.sign(randbytes)
        self._sock.send(buf)
        self._buffered = 0
        self.__handshake_flag = __WAIT_RESPONSE__

    def _do_handshake_wait_validate(self):
        self.__read_buffer(16)
        if self._buffered == 16:
            st = self._buffer[:16].decode("ascii", "ignore").rstrip("\x00")
            if st == "OK":
                logger.debug("Client identify successed")
                self._buffered = 0
                self.__handshake_flag = __WAIT_AESKEY__
            else:
                raise RobotSessionError("Handshake return failed: %s" % st,
                                        error_symbol=st.split(" "))

    def _do_handshake_wait_aeskey(self):
        self.__read_buffer(self.client_key.size)
        if self._buffered == self.client_key.size:
            self.__handshake_flag = __READY__
            buf = self._buffer[:self.client_key.size]
            aes_init = self.client_key.decrypt(buf)
            key, iv = aes_init[:32], aes_init[32:48]
            self._encoder = AES.new(key, AES.MODE_CFB, iv)
            self._decoder = AES.new(key, AES.MODE_CFB, iv)
            return 0
        else:
            return 3

    def recv(self, size, flag=0):
        if flag & socket.MSG_PEEK > 0:
            raise RobotError("recv flag not support",
                             error_symbol=("BAD_PARAMS",
                                           "MSG_PEEK_NOT_ALLOWED"))

        if self._decoder:
            buf = self._sock.recv(size, flag)
            return self._decoder.decrypt(buf)
        else:
            raise RobotNotReadyError("Handshake not complete")

    def send(self, buf):
        if not self._encoder:
            raise RobotNotReadyError("Handshake not complete")

        l = len(buf)
        length = l
        chiptext = memoryview(self._encoder.encrypt(buf))

        sent = self._sock.send(chiptext)
        while sent < length:
            sent += self._sock.send(chiptext[sent:])

        return sent

    def fileno(self):
        return self._sock.fileno()

    @property
    def family(self):
        return self._sock.family

    def getpeername(self, *args, **kw):  # peer ip and port
        return self._sock.getpeername(*args, **kw)

    def getsockname(self, *args, **kw):  # local ip and port
        return self._sock.getsockname(*args, **kw)

    def getsockopt(self, *args, **kw):
        return self._sock.getsockopt(*args, **kw)

    def gettimeout(self, *args, **kw):
        return self._sock.gettimeout(*args, **kw)

    def shutdown(self, *args, **kw):
        return self._sock.shutdown(*args, **kw)

    def close(self):
        return self._sock.close()
