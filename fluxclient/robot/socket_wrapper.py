
import socket
from Crypto.Cipher import AES


class SocketWrapper(object):
    def __init__(self, sock):
        self.sock = sock

    def fileno(self):
        return self.sock.fileno()

    @property
    def family(self):
        return self.sock.family

    def getpeername(self, *args, **kw):
        return self.sock.getpeername(*args, **kw)

    def getsockname(self, *args, **kw):
        return self.sock.getsockname(*args, **kw)

    def getsockopt(self, *args, **kw):
        return self.sock.getsockopt(*args, **kw)

    def gettimeout(self, *args, **kw):
        return self.sock.gettimeout(*args, **kw)

    def shutdown(self, *args, **kw):
        return self.sock.shutdown(*args, **kw)

    def close(self):
        return self.sock.close()


class AESEncryptedSocket(SocketWrapper):
    def __init__(self, sock, key, iv):
        super(AESEncryptedSocket, self).__init__(sock)
        self.encoder = AES.new(key, AES.MODE_CFB, iv)
        self.decoder = AES.new(key, AES.MODE_CFB, iv)

    def recv(self, size, flag=0):
        if flag & socket.MSG_PEEK > 0:
            raise RuntimeError("MSG_PEEK is not allowed here!")

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
