
import logging
import socket

from .errors import RobotSessionError

logger = logging.getLogger(__name__)


class InitBackend(object):
    _buffer = b''

    def __init__(self, endpoint, blocking=True):
        logger.info("Initlize connection to %s", endpoint)
        self.sock = socket.socket()

        if not blocking:
            self.sock.setblocking(False)
        self.sock.connect(endpoint)

    def fileno(self):
        return self.sock.fileno()

    def do_handshake(self):
        buf = self.sock.recv(8 - len(self._buffer))

        if len(buf) == 0:
            raise RobotSessionError("Broken Pipe")

        self._buffer += buf
        if len(self._buffer) == 8:
            magic_str = self._buffer[:4]
            proto_ver = self._buffer[4:]

            if magic_str != b"FLUX":
                raise RobotSessionError(
                    "Hello error",
                    error_symbol=("PROTOCOL_ERROR", "MAGICNUMBER_ERROR"))
            else:
                return int(proto_ver)
