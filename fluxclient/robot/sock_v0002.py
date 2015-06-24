
from Crypto.Cipher import AES

from .sock_base import RobotSocketBase


class RobotSocketV2(RobotSocketBase):
    def __init__(self, sock, key, iv):
        super(RobotSocketV2, self).__init__(sock)
        self.encoder = AES.new(key, AES.MODE_CFB, iv)
        self.decoder = AES.new(key, AES.MODE_CFB, iv)

    def recv(self, size, flag=0):
        buf = self.sock.recv(size, flag)
        return self.decoder.decrypt(buf)

    def send(self, buf):
        l = len(buf)
        length = l
        chiptext = memoryview(self.encoder.encrypt(buf))

        sent = self.sock.send(chiptext)
        while sent < length:
            sent += self.sock.send(chiptext[sent:])
        return sent
