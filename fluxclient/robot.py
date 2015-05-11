
import socket
import curses

from fluxclient import encryptor as E


class RobotClient(object):
    mode = "cmd"

    def __init__(self, ipaddr=None, server_key=None, stdout=None):
        self.sock = s = socket.socket()
        self.output = stdout

        s.connect(ipaddr)
        buf = s.recv(4096)

        ver, sign, randbytes = buf[:8], buf[8:-128], buf[-128:]

        if server_key:
            self.output("Warning: server key checking not implement!")
        else:
            self.output("Warning: can not validate remote")

        rsakey = E.get_or_create_keyobj()
        self.output("Access ID: %s" % E.get_access_id(rsakey))
        buf = E.get_access_id(rsakey, binary=True) + E.sign(rsakey, randbytes)
        s.send(buf)

        status = s.recv(16).rstrip(b"\x00").decode()
        self.output("Handshake: %s" % status)

        if status != "OK":
            raise RuntimeError("Handshake failed.")

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

        self.sock.send(buf)

    def send_raw(self, buf):
        if buf == b"quit":
            self.sock.send(buf)
            self.mode = "cmd"
        else:
            self.sock.send(buf + b"\r\n")

