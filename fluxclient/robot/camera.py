
from select import select
import struct

from .errors import RobotError


class FluxCamera(object):
    __running__ = False
    __buffer__ = b""
    __image_length__ = 0

    def __init__(self, sock):
        self.sock = sock

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
