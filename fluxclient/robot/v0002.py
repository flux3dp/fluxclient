
from time import time
import logging
import socket
import os

from fluxclient import encryptor as E
from .base import RobotError
from .sock_v0002 import RobotSocketV2

logger = logging.getLogger(__name__)

def ok_or_error(fn):
    def wrap(self, *args):
        ret = fn(self, *args)
        if ret == "ok":
            return ret
        elif ret.startswith("error "):
            raise RuntimeError(ret.split(" ")[1:])
        else:
            raise RuntimeError("UNKNOW_ERROR", ret)
    return wrap


class FluxRobotV0002(object):
    def __init__(self, sock, server_key=None):
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

    def _make_cmd(self, buf):
        l = len(buf)
        pad = b"\x00" * ((256 - (l % 128)) % 128)
        self.sock.send(buf + pad)
        buf = self.sock.recv(128)
        return buf.rstrip(b"\x00\n").decode("utf8")

    # Command Tasks
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

    def upload_file(self, filename, cmd="upload", progress_callback=None):
        with open(filename, "rb") as f:
            logger.debug("File opened")
            size = os.fstat(f.fileno()).st_size
            cmd = ("%s %i" % (cmd, size)).encode()

            upload_ret = self._make_cmd(cmd)
            if upload_ret != "continue":
                raise RobotError(upload_ret)

            sent = 0
            ts = 0
            while sent < size:
                buf = f.read(4096)
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

    # Scan Tasks
    def taks_scanshot(self):
        pass

    def scan_next(self):
        pass

    def scan_forword(self):
        pass
