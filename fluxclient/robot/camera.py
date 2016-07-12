
from select import select
import struct

from .errors import RobotError, RobotSessionError
from .aes_socket import AESSocket
from .ssl_socket import SSLSocket
from .backends import InitBackend


class Camera(object):
    __running__ = False
    __buffer__ = b""
    __image_length__ = 0

    def __init__(self, endpoint, client_key, device=None, conn_callback=None):
        self._device = device

        init_backend = InitBackend(endpoint)
        proto_ver = init_backend.do_handshake()

        if proto_ver == 2:
            self.sock = AESSocket(init_backend.sock, client_key=client_key,
                                  device=device)
        elif proto_ver == 3:
            self.sock = SSLSocket(init_backend.sock, client_key=client_key,
                                  device=device)
        else:
            raise RobotSessionError("Protocol not support")

        while self.sock.do_handshake() > 0:
            pass

    def fileno(self):
        return self.sock.fileno()

    def feed(self, image_callback):
        b = self.sock.recv(4096)
        if b:
            self.__buffer__ += b
        else:
            self.close()
            raise RobotError("DISCONNECTED")
        self.unpack_image(image_callback)

    def capture(self, image_callback):
        self.__running__ = True

        while self.__running__:
            rl = select((self.sock, ), (), (), 0.05)[0]
            if rl:
                self.feed(image_callback)

    def unpack_image(self, image_callback):
        length = self.__image_length__
        while True:
            if length:
                if len(self.__buffer__) >= length:
                    image = self.__buffer__[4:length]
                    self.__buffer__ = self.__buffer__[length:]
                    length = 0
                    image_callback(self, image)
                else:
                    break
            else:
                if len(self.__buffer__) >= 4:
                    length = struct.unpack("<I",
                                           self.__buffer__[:4])[0] + 4
                else:
                    break
        self.__image_length__ = length

    def abort(self):
        self.__running__ = False

    def close(self):
        self.sock.close()


FluxCamera = Camera
