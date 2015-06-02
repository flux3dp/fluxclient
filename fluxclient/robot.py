
from time import time, sleep
import socket
import os

from fluxclient import encryptor as E


class RobotClient(object):
    mode = "cmd"

    def __init__(self, ipaddr=None, server_key=None, stdout=None):
        self.output = stdout

        self.output("Connecting..")
        self.sock = s = self.connect(ipaddr)
        self.output(" OK\n")
        # self.sock = s = socket.socket()
        # s.connect(ipaddr)

        buf = s.recv(4096)

        ver, sign, randbytes = buf[:8], buf[8:-128], buf[-128:]

        if server_key:
            self.output("Warning: server key checking not implement!\n")
        else:
            self.output("Warning: can not validate remote\n")

        rsakey = E.get_or_create_keyobj()
        self.output("Protocol: %s\n" % ver.decode("ascii", "ignore"))
        self.output("Access ID: %s\n" % E.get_access_id(rsakey))
        buf = E.get_access_id(rsakey, binary=True) + E.sign(rsakey, randbytes)
        s.send(buf)

        status = s.recv(16).rstrip(b"\x00").decode()
        self.output("Handshake: %s\n" % status)

        if status != "OK":
            raise RuntimeError("Handshake failed.")

    def connect(self, ipaddr):
        while True:
            t = time()

            try:
                self.output(".")
                s = socket.socket()
                s.connect(ipaddr)
                return s
            except ConnectionRefusedError:
                sleep(min(0.6 - time() + t, 0.6))

    def recv(self, l=4096):
        buf = self.sock.recv(4096)
        return buf

    def send(self, buf):
        if self.mode == "cmd":
            self.send_cmd(buf)
        elif self.mode == "raw":
            self.send_raw(buf)

    def send_cmd(self, buf):
        if buf == b"raw":
            self.mode = "raw"
        elif buf.startswith(b"upload "):
            filename = buf.split(b" ", 1)[-1].decode("utf-8")
            with open(filename, "rb") as f:
                self.output("FILE OPENED")
                size = os.fstat(f.fileno()).st_size
                buf = ("upload %i" % size).encode()
                self.sock.send(buf)

                sent = 0

                self.output("%s %s" % (sent, size))
                while sent < size:
                    buf = f.read(4096)
                    l = len(buf)

                    if l == 0:
                        raise RuntimeError("File size error")
                    sent += l

                    self.sock.send(buf)
                    self.output("UPLOADING: %.3f %i / %i" %
                                (sent / size, sent, size))

            self.output("UPLOAD COMPLETE")
            return

        self.sock.send(buf)

    def send_raw(self, buf):
        if buf == b"quit":
            self.sock.send(buf)
            self.mode = "cmd"
        else:
            self.sock.send(buf + b"\r\n")
