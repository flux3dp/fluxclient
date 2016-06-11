
from select import select
from errno import EPIPE
from io import BytesIO
from time import time
import logging
import socket
import struct
import json
import os

from fluxclient.utils import mimetypes
from .aes_socket import AESSocket
from .errors import RobotError
from .misc import msg_waitall

logger = logging.getLogger(__name__)


def raise_error(ret):
    if ret.startswith("error ") or ret.startswith("er "):
        raise RobotError(*(ret.split(" ")[1:]))
    else:
        raise RobotError("UNKNOW_ERROR", ret)


def ok_or_error(fn, resp="ok"):
    def wrap(self, *args, **kw):
        ret = fn(self, *args, **kw).decode("utf8", "ignore")
        if ret != resp:
            raise_error(ret)
    return wrap


def split_path(rawpath):
    args = rawpath.split("/", 2)
    if len(args) < 2:
        raise RobotError("Wrong path: %s" % rawpath,
                         error_symbol=("BAD_PARAMS"))
    entry = args[1]
    path = args[2] if len(args) > 2 else ""
    return entry, path


class MaintainTaskMixIn(object):
    @ok_or_error
    def begin_maintain(self):
        return self._make_cmd(b"maintain")

    def maintain_home(self):
        self._send_cmd(b"home")
        while True:
            ret = self.get_resp(6.0).decode("ascii", "ignore")
            if ret.startswith("DEBUG:"):
                logger.info(ret)
            elif ret == "ok":
                return
            else:
                raise_error(ret)

    @ok_or_error
    def maintain_reset_mb(self):
        return self._make_cmd(b"reset_mb")

    def maintain_calibration(self, threshold=None, navigate_callback=None,
                             clean=False):
        cmd = ['calibration']
        if threshold:
            cmd.append(str(threshold))
        if clean:
            cmd.append("clean")

        ret = self._make_cmd(cmd.encode())

        if ret == b"continue":
            nav = "continue"
            while True:
                if nav.startswith("ok "):
                    return [float(item) for item in nav.split(" ")[1:]]
                elif nav.startswith("error "):
                    raise_error(nav)
                else:
                    navigate_callback(nav)
                nav = self.get_resp().decode("ascii", "ignore")
        else:
            raise_error(ret.decode("ascii", "ignore"))

    def maintain_zprobe(self, navigate_callback, manual_h=None):
        if manual_h:
            ret = self._make_cmd(("zprobe %.4f" % manual_h).encode())
        else:
            ret = self._make_cmd(b"zprobe")

        if ret == b"continue":
            nav = "continue"
            while True:
                if nav.startswith("ok "):
                    return float(nav[3:])
                elif nav.startswith("error "):
                    raise_error(nav)
                else:
                    navigate_callback(nav)
                nav = self.get_resp().decode("ascii", "ignore")
        else:
            raise_error(ret.decode("ascii", "ignore"))

    def maintain_manual_level(self, h):
        ret = self._make_cmd(("zprobe %.4f" % h).encode())
        if ret == b"continue":
            nav = "continue"
            while True:
                if nav.startswith("ok "):
                    return float(nav[3:])
                nav = self.get_resp().decode("ascii", "ignore")

    def maintain_head_info(self):
        ret = self._make_cmd(b"headinfo").decode("ascii", "ignore")
        if ret.startswith("ok "):
            return json.loads(ret[3:])
        else:
            raise_error(ret)

    def maintain_head_status(self):
        ret = self._make_cmd(b"headstatus").decode("ascii", "ignore")
        if ret.startswith("ok "):
            return json.loads(ret[3:])
        else:
            raise_error(ret)

    def maintain_load_filament(self, index, temp, navigate_callback):
        ret = self._make_cmd(
            ("load_filament %i %.1f" % (index, temp)).encode())

        if ret == b"continue":
            while True:
                try:
                    ret = self.get_resp().decode("ascii", "ignore")
                    if ret.startswith("CTRL "):
                        navigate_callback(ret[5:])
                    elif ret == "ok":
                        return
                    else:
                        raise_error(ret)
                except KeyboardInterrupt:
                    self._send_cmd(b"stop_load_filament")
                    logger.info("Interrupt load filament")
        else:
            raise_error(ret.decode("ascii", "utf8"))

    def maintain_unload_filament(self, index, temp, navigate_callback):
        ret = self._make_cmd(
            ("unload_filament %i %.1f" % (index, temp)).encode())

        if ret == b"continue":
            while True:
                ret = self.get_resp().decode("ascii", "ignore")
                if ret.startswith("CTRL "):
                    navigate_callback(ret[5:])
                elif ret == "ok":
                    return
                else:
                    raise_error(ret)
        else:
            raise_error(ret.decode("ascii", "utf8"))

    def maintain_interrupt_load_filament(self):
        self._send_cmd("stop_load_filament")

    @ok_or_error
    def maintain_extruder_temperature(self, index, temp):
        ret = self._make_cmd(
            ("extruder_temp %i %.1f" % (index, temp)).encode())
        return ret

    def maintain_update_hbfw(self, mimetype, stream, size,
                             progress_callback=None):
        def uplaod_process_callback(self, processed, total):
            progress_callback("CTRL UPLOADING %i" % processed)

        logger.debug("File opened")
        self.upload_stream(stream, size, mimetype, None, "update_head",
                           uplaod_process_callback)
        while True:
            ret = self.get_resp().decode("utf8", "ignore")
            if ret == "ok":
                return
            elif ret.startswith("CTRL "):
                progress_callback(ret)
            else:
                raise_error(ret)


class ScanTaskMixIn(object):
    @ok_or_error
    def begin_scan(self):
        return self._make_cmd(b"scan")

    @ok_or_error
    def scan_laser(self, left, right):
        bcmd = b"scanlaser "
        if left:
            bcmd += b"l"
        if right:
            bcmd += b"r"

        return self._make_cmd(bcmd)

    @ok_or_error
    def scan_step_length(self, l):
        cmd = "set steplen %.5f" % l
        return self._make_cmd(cmd.encode())

    def scan_check_camera(self):
        self._send_cmd(b"scan_check")
        resp = self.get_resp(timeout=99999).decode("ascii", "ignore")
        return resp

    def scan_calibrate(self):
        self._send_cmd(b"calibrate")
        resp = self.get_resp(timeout=99999).decode("ascii", "ignore")
        return resp

    def scan_get_calibrate(self):
        self._send_cmd(b"get_cab")
        resp = self.get_resp().decode("ascii", "ignore")
        return resp

    def scan_oneshot(self):
        self._send_cmd(b"oneshot")
        images = []
        while True:
            resp = self.get_resp().decode("ascii", "ignore")

            if resp.startswith("binary "):
                images.append(self.recv_binary(resp))

            elif resp == "ok":
                return images

            else:
                raise RobotError(resp)

    def scan_images(self):
        self._send_cmd(b"scanimages")
        images = []
        while True:
            resp = self.get_resp().decode("ascii", "ignore")

            if resp.startswith("binary "):
                images.append(self.recv_binary(resp))

            elif resp == "ok":
                return images

            else:
                raise RobotError(resp)

    @ok_or_error
    def scan_forward(self):
        return self._make_cmd(b"scan_next")

    @ok_or_error
    def scan_backward(self):
        return self._make_cmd(b"scan_backward")


class RobotBackend2(ScanTaskMixIn, MaintainTaskMixIn):
    def __init__(self, sock, client_key, device=None,
                 ignore_key_validation=False):
        self.sock = aessock = AESSocket(
            sock, client_key=client_key, device=device,
            ignore_key_validation=False)

        while not aessock.do_handshake():
            pass

    def fileno(self):
        return self.sock.fileno()

    def close(self):
        self.sock.close()

    def _send_cmd(self, buf):
        l = len(buf) + 2
        self.sock.send(struct.pack("<H", l) + buf)

    def get_resp(self, timeout=180.0):
        rl = select((self.sock, ), (), (), timeout)[0]
        if not rl:
            raise RobotError("get resp timeout")
        bml = msg_waitall(self.sock, 2, timeout)
        if not bml:
            logger.error("Message payload recv error")
            raise socket.error(EPIPE, "Broken pipe")

        message_length = struct.unpack("<H", bml)[0]

        message = b""

        while len(message) != message_length:
            buf = self.sock.recv(message_length - len(message))

            if not buf:
                logger.error("Recv empty message")
                raise socket.error(EPIPE, "Broken pipe")
            message += buf

        return message

    def _make_cmd(self, buf, timeout=30.0):
        self._send_cmd(buf)
        return self.get_resp(timeout)

    def recv_binary_into(self, binary_header, stream, callback=None):
        mn, mimetype, ssize = binary_header.split(" ")
        assert mn == "binary"
        size = int(ssize)
        logger.debug("Recv %s %i" % (mimetype, size))
        if size == 0:
            return mimetype
        left = size
        while left > 0:
            left -= stream.write(self.sock.recv(min(4096, left)))
            if callback:
                callback(left, size)
        return mimetype

    def recv_binary(self, binary_header, callback=None):
        buf = BytesIO()
        mimetype = self.recv_binary_into(binary_header, buf, callback)
        return (mimetype, buf.getvalue())

    # Command Tasks
    def position(self):
        ret = self._make_cmd(b"position").decode("ascii", "ignore")
        if ret.startswith("error "):
            raise RobotError(*(ret.split(" ")[1:]))
        else:
            return ret

    # file commands
    def list_files(self, path):
        entry, path = split_path(path)

        self._send_cmd(b"file ls " + entry.encode() + b" " + path.encode())
        resp = self.get_resp().decode("ascii", "ignore")
        if resp == "continue":
            files = self.get_resp().decode("utf8", "ignore")
            last_resp = self.get_resp().decode("ascii", "ignore")
            if last_resp != "ok":
                raise_error("error PROTOCOL_ERROR NOT_OK")

            return [(node.startswith("D"), node[1:])
                    for node in files.split("\x00") if node]
        else:
            raise_error(resp)

    def file_info(self, path):
        info = [None, None]
        entry, path = split_path(path)

        self._send_cmd(b"file info " + entry.encode() + b" " + path.encode())

        resp = self.get_resp().decode("utf8", "ignore")
        images = []
        while resp.startswith("binary "):
            images.append(self.recv_binary(resp))
            resp = self.get_resp().decode("utf8", "ignore")
        info[1] = images

        def fixmeta(key, value=None):
            return key, value
        if resp.startswith("ok "):
            info[0] = dict(fixmeta(*pair.split("=", 1)) for pair in
                           resp[3:].split("\x00"))
            return info
        else:
            raise_error(resp)

    @ok_or_error
    def mkdir(self, path):
        entry, path = split_path(path)
        return self._make_cmd(
            b"file mkdir " + entry.encode() + b" " + path.encode())

    @ok_or_error
    def rmdir(self, path):
        entry, path = split_path(path)
        return self._make_cmd(
            b"file rmdir " + entry.encode() + b" " + path.encode())

    @ok_or_error
    def cpfile(self, source_path, dist_path):
        source_entry, source = split_path(source_path)
        target_entry, target = split_path(dist_path)
        return self._make_cmd(
            b"file cp " + source_entry.encode() + b" " + source.encode() +
            b" " + target_entry.encode() + b" " + target.encode())

    @ok_or_error
    def rmfile(self, path):
        entry, path = split_path(path)
        return self._make_cmd(b"file rm " + entry.encode() + b" " +
                              path.encode())

    def file_md5(self, path):
        entry, path = split_path(path)
        bresp = self._make_cmd(b"file md5 " + entry.encode() + b" " +
                               path.encode())
        resp = bresp.decode("ascii", "ignore")
        if resp.startswith("md5 "):
            return resp[4:]
        else:
            raise_error(resp)

    def download_file(self, path, stream, callback=None):
        entry, path = split_path(path)
        self._send_cmd(("file download %s %s" % (entry, path)).encode())
        resp = self.get_resp().decode("utf8", "ignore")
        if resp.startswith("binary "):
            mimetype = self.recv_binary_into(resp, stream, callback)
            ret = self.get_resp().decode("utf8", "ignore")
            if ret == "ok":
                return mimetype
            else:
                raise_error(resp)
        else:
            raise_error(resp)

    def upload_file(self, filename, upload_to="#", cmd="file upload",
                    progress_callback=None):
        mimetype, _ = mimetypes.guess_type(filename)
        if not mimetype:
            mimetype = "binary"
        with open(filename, "rb") as f:
            logger.debug("File opened")
            size = os.fstat(f.fileno()).st_size
            return self.upload_stream(f, size, mimetype, upload_to, cmd,
                                      progress_callback)

    # player commands
    @ok_or_error
    def select_file(self, path):
        entry, path = split_path(path)
        self._send_cmd(b"player select " + entry.encode() + b" " +
                       path.encode())
        return self.get_resp()

    @ok_or_error
    def start_play(self):
        return self._make_cmd(b"player start")

    @ok_or_error
    def pause_play(self):
        return self._make_cmd(b"player pause")

    @ok_or_error
    def abort_play(self):
        return self._make_cmd(b"player abort")

    @ok_or_error
    def resume_play(self):
        return self._make_cmd(b"player resume")

    def report_play(self):
        # TODO
        msg = self._make_cmd(b"player report").decode("utf8", "ignore")
        if msg.startswith("{"):
            return json.loads(msg, "ignore")
        else:
            raise_error(msg)

    def play_info(self):
        self._send_cmd(b"player info")
        metadata = None

        resp = self.get_resp().decode("ascii", "ignore")
        if resp.startswith("binary text/json"):
            _, bm = self.recv_binary(resp)
            metadata = json.loads(bm.decode("utf8", "ignore"))
        else:
            raise_error(resp)

        images = []
        while True:
            resp = self.get_resp().decode("ascii", "ignore")
            if resp.startswith("binary "):
                images.append(self.recv_binary(resp))
            elif resp == "ok":
                return metadata, images
            else:
                raise RobotError(resp)

    @ok_or_error
    def quit_play(self):
        return self._make_cmd(b"player quit")

    def upload_stream(self, stream, length, mimetype, upload_to=None,
                      cmd="file upload", progress_callback=None):
        if upload_to == "#":
            # cmd = [cmd] [mimetype] [length] #
            cmd = "%s %s %i #" % (cmd, mimetype, length)
        elif upload_to:
            entry, path = upload_to.split("/", 1)
            # cmd = [cmd] [mimetype] [length] [entry] [path]
            cmd = "%s %s %i %s %s" % (cmd, mimetype, length, entry, path)
        else:
            cmd = "%s %s %i" % (cmd, mimetype, length)

        upload_ret = self._make_cmd(cmd.encode()).decode("ascii", "ignore")
        if upload_ret == "continue":
            logger.info(upload_ret)
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
                progress_callback(self, sent, length)

        progress_callback(self, sent, length)
        final_ret = self.get_resp()
        logger.debug("File uploaded")

        if final_ret != b"ok":
            raise_error(final_ret.decode("ascii", "ignore"))

    # Others
    def begin_upload(self, mimetype, length, cmd="upload", upload_to="#"):
        cmd = ("%s %s %i %s" % (cmd, mimetype, length, upload_to)).encode()
        upload_ret = self._make_cmd(cmd).decode("ascii", "ignore")
        if upload_ret == "continue":
            return self.sock
        else:
            raise RobotError(upload_ret)

    @ok_or_error
    def quit_task(self):
        return self._make_cmd(b"quit")

    @ok_or_error
    def kick(self):
        return self._make_cmd(b"kick")

    def raw_mode(self):
        ret = self._make_cmd(b"task raw")
        if ret == b"continue":
            return self.sock
        else:
            raise_error(ret.decode("ascii", "ignore"))

    @ok_or_error
    def quit_raw_mode(self):
        self.sock.send(b"quit")
        sync = 0
        while True:
            ret = ord(self.sock.recv(1))
            if sync > 8:
                if ret > 0:
                    break
            else:
                if ret == 0:
                    sync += 1
        return self.get_resp()

    @ok_or_error
    def config_set(self, key, value):
        return self._make_cmd(b"config set " + key.encode() + b" " +
                              value.encode())

    def config_get(self, key):
        ret = self._make_cmd(b"config get " + key.encode()).decode("utf8",
                                                                   "ignore")
        if ret.startswith("ok VAL "):
            return ret[7:]
        elif ret.startswith("ok EMPTY"):
            return None
        else:
            raise_error(ret)

    @ok_or_error
    def config_del(self, key):
        return self._make_cmd(b"config del " + key.encode())

    @ok_or_error
    def set_setting(self, key, value):
        cmd = "set %s %s" % (key, value)
        return self._make_cmd(cmd.encode())

    def deviceinfo(self):
        ret = self._make_cmd(b"deviceinfo").decode("utf8", "ignore")
        if ret.startswith("ok\n"):
            info = {}
            for raw in ret[3:].split("\n"):
                if ":" in raw:
                    key, val = raw.split(":", 1)
                    info[key] = val
            return info
        else:
            raise_error(ret)
