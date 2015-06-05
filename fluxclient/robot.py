
from time import time, sleep
import socket
import os

from Crypto.Cipher import AES

from fluxclient import encryptor as E


class RobotClient(object):
    mode = "cmd"

    def __init__(self, ipaddr=None, server_key=None, stdout=None):
        self.output = stdout

        self.output("Connecting..")
        self.sock = s = self.connect(ipaddr)
        self.output(" OK\n")

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

        status = s.recv(16, socket.MSG_WAITALL).rstrip(b"\x00").decode()
        self.output("Handshake: %s\n" % status)

        if status == "OK":
            aes_enc_init = s.recv(E.rsa_size(rsakey), socket.MSG_WAITALL)
            aes_init = E.rsa_decrypt(rsakey, aes_enc_init)

            self.aes_enc = AES.new(aes_init[:32], AES.MODE_CFB,
                                   aes_init[32:48], segment_size=128)
            self.aes_dec = AES.new(aes_init[:32], AES.MODE_CFB,
                                   aes_init[32:48], segment_size=128)
        else:
            raise RuntimeError("Handshake failed: %s" % status)

        self._recv_buf = bytearray(4096)
        self._recv_bufview = memoryview(self._recv_buf)
        self._recv_offset = 0

        self.output("Ready\n")

    def fileno(self):
        return self.sock.fileno()

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

    def recv(self, length=None):
        l = self.sock.recv_into(self._recv_bufview[self._recv_offset:])

        offset = self._recv_offset + l  # Total data in buffer
        chunk_offset = 128 * (l // 128)   # Data can be processed
        left_offset = offset - chunk_offset

        buf = self.aes_dec.decrypt(self._recv_bufview[:chunk_offset].tobytes())
        if left_offset:
            self._recv_bufview[:left_offset] = \
                self._recv_bufview[chunk_offset:offset]
        return buf

    def _send(self, buf):
        l = len(buf)
        if l > 4096:
            raise RuntimeError("Do not send message larger then 4096, got %i" %
                               l)

        pad = b"\x00" * ((256 - (l % 128)) % 128)
        payload = self.aes_enc.encrypt(buf + pad)
        self.sock.send(payload)

    def send_cmd(self, cmd):
        cmd = cmd.strip()
        if self.mode == "cmd":
            if cmd == b"raw":
                self._send(cmd)
                self.mode = "raw"
            elif cmd.startswith(b"upload "):
                filename = cmd.split(b" ", 1)[-1].decode("utf-8")
                with open(filename, "rb") as f:
                    self.output("FILE OPENED\n")
                    size = os.fstat(f.fileno()).st_size
                    cmd = ("upload %i" % size).encode()
                    self._send(cmd)
                    sleep(1.0)

                    sent = 0
                    ts = time()

                    fbuf = bytearray(4096)
                    while sent < size:
                        l = f.readinto(fbuf)

                        if l == 0:
                            raise RuntimeError("File size error")
                        sent += l

                        self._send(bytes(fbuf[:l]))
                        if time() - ts > 1.0:
                            self.output("UPLOADING: %.3f %i / %i\n" %
                                        (sent / size, sent, size))
                            ts = time()

                self.output("UPLOAD COMPLETE\n")
                return
            else:
                self._send(cmd)
        elif self.mode == "raw":
            if cmd.strip() == b"quit":
                self._send(cmd)
                self.mode = "cmd"
            else:
                self._send(cmd + b"\n")
