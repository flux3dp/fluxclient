
from io import BytesIO
from time import time
import logging
import socket
import os

from fluxclient import encryptor as E
from .base import RobotError
from .sock_v0002 import RobotSocketV2

logger = logging.getLogger(__name__)


def ok_or_error(fn, resp="ok"):
    def wrap(self, *args):
        ret = fn(self, *args)
        if ret == resp:
            return ret
        elif ret.startswith("error "):
            raise RuntimeError(ret.split(" ")[1:])
        else:
            raise RuntimeError("UNKNOW_ERROR", ret)
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
        l = len(buf)
        pad = b"\x00" * ((256 - (l % 128)) % 128)
        self.sock.send(buf + pad)

    def _recv_resp(self):
        buf = self.sock.recv(128, socket.MSG_WAITALL)
        return buf.rstrip(b"\x00\n").decode("utf8", "ignore")

    def _make_cmd(self, buf):
        self._send_cmd(buf)
        return self._recv_resp()

    # Command Tasks
    def position(self):
        ret = self._make_cmd(b"position")
        if ret.startswith("error "):
            raise RuntimeError(*ret.split(" ")[1:])
        else:
            return ret

    def list_file(self):
        # TODO: TBC
        ret = self._make_cmd(b"ls")
        return ret

    def select_file(self, fildid):
        raise RuntimeError("NOT_SUPPORT")

    def start_play(self):
        ret = self._make_cmd(b"start")
        if ret != "ok":
            raise RuntimeError(ret)

    def upload_stream(self, stream, length, cmd="upload",
                      progress_callback=None):
        cmd = ("%s %i" % (cmd, length)).encode()

        upload_ret = self._make_cmd(cmd)
        if upload_ret != "continue":
            raise RobotError(upload_ret)

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
                progress_callback(self, sent, size)

        progress_callback(self, sent, size)
        buf = self.sock.recv(128, socket.MSG_WAITALL)
        logger.debug("File uploaded")

        final_ret = buf.rstrip(b"\x00\n").decode("utf8")
        if final_ret != "ok":
            raise RobotError(final_ret)

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
        return self._make_cmd(b"report")

    def oneshot(self):
        self._send_cmd(b"oneshot")
        images = []
        while True:
            resp = self._recv_resp().split(" ")

            if resp[0] == "binary":
                mime, length = resp[1], int(resp[2])
                logger.debug("Recv image %s %i" % (mime, length))

                buf = BytesIO()
                left = length
                while left > 0:
                    left -= buf.write(self.sock.recv(min(4096, left)))

                # No use padding
                self.sock.recv((256 - (length % 128)) % 128,
                               socket.MSG_WAITALL)
                images.append((mime, buf.getvalue()))

            elif resp[0] == "ok":
                return images

            else:
                raise RuntimeError(resp)

    def scanimages(self):
        self._send_cmd(b"scanimages")
        images = []
        while True:
            resp = self._recv_resp().split(" ")

            if resp[0] == "binary":
                mime, length = resp[1], int(resp[2])
                logger.debug("Recv image %s %i" % (mime, length))

                buf = BytesIO()
                left = length
                while left > 0:
                    left -= buf.write(self.sock.recv(min(4096, left)))

                # No use padding
                self.sock.recv((256 - (length % 128)) % 128,
                               socket.MSG_WAITALL)
                images.append((mime, buf.getvalue()))

            elif resp[0] == "ok":
                return images

            else:
                raise RuntimeError(resp)

    # Scan Tasks
    def taks_scanshot(self):
        pass

    @ok_or_error
    def scan_next(self):
        return self._make_cmd(b"scan_next")

    @ok_or_error
    def scan_forword(self):
        return self._make_cmd(b"scan_forword")

    def raw_mode(self):
        ret = self._make_cmd(b"raw")
        if ret == "continue":
            return self.sock
        elif ret.startswith("error "):
            raise RuntimeError(ret.split(" ")[1:])
        else:
            raise RuntimeError("UNKNOW_ERROR", ret)
