
from errno import EPIPE
from io import BytesIO
from time import time
import struct
import logging
import socket
import os

from fluxclient import encryptor as E
from .base import RobotError
from .sock_v0002 import RobotSocketV2

logger = logging.getLogger(__name__)


def raise_error(ret):
    if ret.startswith("error "):
        raise RuntimeError(*(ret.split(" ")[1:]))
    else:
        raise RuntimeError("UNKNOW_ERROR", ret)


def ok_or_error(fn, resp="ok"):
    def wrap(self, *args, **kw):
        ret = fn(self, *args, **kw).decode("utf8", "ignore")
        if ret == resp:
            return ret
        else:
            raise_error(ret)
    return wrap


class FluxRobotV0002(object):
    def __init__(self, sock, server_key=None):
        sock.settimeout(300)
        buf = sock.recv(4096)
        sign, randbytes = buf[8:-128], buf[-128:]

        if server_key:
            logger.error("Warning: server key checking not implement!")
        else:
            logger.warn("Warning: can not validate remote")

        rsakey = E.get_or_create_keyobj()
        logger.info("Protocol: FLUX0002")
        logger.info("Access ID: %s" % E.get_access_id(rsakey))

        buf = E.get_access_id(rsakey, binary=True) + E.sign(rsakey, randbytes)
        sock.send(buf)
        status = sock.recv(16, socket.MSG_WAITALL).rstrip(b"\x00").decode()
        if status == "OK":
            aes_enc_init = sock.recv(E.rsa_size(rsakey), socket.MSG_WAITALL)
            aes_init = E.rsa_decrypt(rsakey, aes_enc_init)

            self.sock = RobotSocketV2(sock, aes_init[:32], aes_init[32:48])
        else:
            raise RuntimeError("Handshake failed: %s" % status)

    def fileno(self):
        return self.sock.fileno()

    def on_recv(self):
        return self.sock.recv(4096)

    def _send_cmd(self, buf):
        l = len(buf) + 2
        self.sock.send(struct.pack("<H", l) + buf)

    def get_resp(self, timeout=30.):
        bml = self.sock.recv(2, socket.MSG_WAITALL)
        if not bml:
            raise socket.error(EPIPE, "Broken pipe")
        message_length = struct.unpack("<H", bml)[0]
        return self.sock.recv(message_length, socket.MSG_WAITALL)

    def _make_cmd(self, buf):
        self._send_cmd(buf)
        return self.get_resp()

    # Command Tasks
    def position(self):
        ret = self._make_cmd(b"position").decode("ascii", "ignore")
        if ret.startswith("error "):
            raise RuntimeError(*(ret.split(" ")[1:]))
        else:
            return ret

    def list_file(self):
        # TODO: TBC
        self._send_cmd(b"ls")
        files = []
        while True:
            line = self.get_resp()
            if line.startswith(b"file "):
                files.append(line[5:].decode("utf8", "ignore"))
            elif line == b"ok":
                return files
            elif line.startswith(b"error "):
                errarg = line.decode("ascii", "ignore").split(" ")[1:]
                raise RuntimeError(*errarg)
            else:
                raise RuntimeError(line)

    @ok_or_error
    def select_file(self, fileid):
        return self._make_cmd(b"select " + fileid.encode())

    @ok_or_error
    def start_play(self):
        return self._make_cmd(b"start")

    def begin_upload(self, length, cmd="upload"):
        cmd = ("%s %i" % (cmd, length)).encode()
        upload_ret = self._make_cmd(cmd).decode("ascii", "ignore")
        if upload_ret == "continue":
            return self.sock
        else:
            raise RuntimeError(upload_ret)

    def upload_stream(self, stream, length, cmd="upload",
                      progress_callback=None):
        cmd = ("%s %i" % (cmd, length)).encode()

        upload_ret = self._make_cmd(cmd).decode("ascii", "ignore")
        if upload_ret == "continue":
            logger.info(upload_ret)
        if upload_ret != "continue":
            raise RuntimeError(upload_ret)

        logger.debug("Upload stream length: %i" % length)

        sent = 0
        ts = 0
        while sent < length:
            buf = stream.read(4096)
            lbuf = len(buf)
            if lbuf == 0:
                raise RobotError("Upload file error")
            sent += lbuf
            self.sock.send(buf)

            if progress_callback and time() - ts > 1.0:
                ts = time()
                progress_callback(self, sent, length)

        progress_callback(self, sent, length)
        final_ret = self.get_resp()
        logger.debug("File uploaded")

        if final_ret != b"ok":
            raise_error(final_ret)

    def upload_file(self, filename, cmd="upload", progress_callback=None):
        with open(filename, "rb") as f:
            logger.debug("File opened")
            size = os.fstat(f.fileno()).st_size
            return self.upload_stream(f, size, cmd, progress_callback)

    @ok_or_error
    def begin_scan(self):
        return self._make_cmd(b"scan")

    @ok_or_error
    def quit_task(self):
        return self._make_cmd(b"quit")

    @ok_or_error
    def kick(self):
        return self._make_cmd(b"kick")

    # Play Tasks
    @ok_or_error
    def pause_play(self):
        return self._make_cmd(b"pause")

    @ok_or_error
    def abort_play(self):
        return self._make_cmd(b"abort")

    @ok_or_error
    def resume_play(self):
        return self._make_cmd(b"resume")

    def report_play(self):
        return self._make_cmd(b"report").decode("utf8", "ignore")

    @ok_or_error
    def scan_laser(self, left, right):
        bcmd = b"scanlaser "
        if left:
            bcmd += b"l"
        if right:
            bcmd += b"r"

        return self._make_cmd(bcmd)

    @ok_or_error
    def set_scanlen(self, l):
        cmd = "set steplen %.5f" % l
        return self._make_cmd(cmd.encode())

    def oneshot(self):
        self._send_cmd(b"oneshot")
        images = []
        while True:
            resp = self.get_resp().decode("ascii", "ignore").split(" ")

            if resp[0] == "binary":
                mime, length = resp[1], int(resp[2])
                logger.debug("Recv image %s %i" % (mime, length))

                buf = BytesIO()
                left = length
                while left > 0:
                    left -= buf.write(self.sock.recv(min(4096, left)))

                images.append((mime, buf.getvalue()))

            elif resp[0] == "ok":
                return images

            else:
                raise RuntimeError(resp)

    def scanimages(self):
        self._send_cmd(b"scanimages")
        images = []
        while True:
            resp = self.get_resp().decode("ascii", "ignore").split(" ")

            if resp[0] == "binary":
                mime, length = resp[1], int(resp[2])
                logger.debug("Recv image %s %i" % (mime, length))

                buf = BytesIO()
                left = length
                while left > 0:
                    left -= buf.write(self.sock.recv(min(4096, left)))

                images.append((mime, buf.getvalue()))

            elif resp[0] == "ok":
                return images

            else:
                raise RuntimeError(resp)

    # Scan Tasks
    def taks_scanshot(self):
        pass

    @ok_or_error
    def scan_next(self, resolution=200):
        return self._make_cmd(b"scan_next")

    @ok_or_error
    def scan_backward(self):
        # TODO: change protocal, this actually means to go backward a step
        return self._make_cmd(b"scan_forword")

    @ok_or_error
    def begin_maintain(self):
        return self._make_cmd(b"maintain")

    @ok_or_error
    def maintain_home(self):
        return self._make_cmd(b"home")

    def raw_mode(self):
        ret = self._make_cmd(b"raw")
        if ret == b"continue":
            return self.sock
        else:
            raise_error(ret.decode("ascii", "ignore"))

    @ok_or_error
    def set_setting(self, key, value):
        cmd = "set %s %s" % (key, value)
        return self._make_cmd(cmd.encode())
