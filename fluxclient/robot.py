
import socket
import curses

from fluxclient import encryptor as E


class RobotClient(object):
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

