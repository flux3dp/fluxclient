
from select import select
import socket
import struct

from .errors import RobotError, RobotSessionError
from .ssl_socket import SSLSocket
from .backends import InitBackend


class USBBridgeSocket(object):
    def __init__(self, usbprotocol):
        self.channel = usbprotocol.open_channel("camera")
        self.lsock, self.rsock = socket.socketpair()
        self.channel.binary_stream = self.rsock

    def __getattr__(self, name):
        return getattr(self.lsock, name)

    def send(self, buf):
        self.channel.send_binary(buf)

    def close(self):
        self.lsock.close()
        self.channel.close()


class FluxCamera(object):
    @classmethod
    def from_usb(cls, client_key, usbprotocol):
        sock = USBBridgeSocket(usbprotocol)
        return cls("USB", client_key, sock=sock)

    """Connect to camera service.

    :param tuple endpoint: A tuple contain a pair of IP address and port to \
connect. For example: ("192.168.1.1", 23812)
    :param encrypt.KeyObject client_key: Client identify key
    :param dict device: Device instance
    """

    __running__ = False
    __buffer__ = b""
    __image_length__ = 0

    def __init__(self, endpoint, client_key, sock=None, device=None):
        self._device = device

        if sock:
            self.sock = sock
        else:
            init_backend = InitBackend(endpoint)
            proto_ver = init_backend.do_handshake()

            if proto_ver == 3:
                self.sock = SSLSocket(init_backend.sock, client_key=client_key,
                                      device=device)
            else:
                raise RobotSessionError("Protocol not support")

            while self.sock.do_handshake() > 0:
                pass

    def enable_streaming(self):
        self.sock.send(struct.pack("<H2s", 4, b"s+"))

    def disable_streaming(self):
        self.sock.send(struct.pack("<H2s", 4, b"s-"))

    def require_frame(self):
        self.sock.send(struct.pack("<H1s", 3, b"f"))

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
