
from binascii import a2b_base64
from select import select
from io import BytesIO
from time import time
import logging
import struct
import json

from PIL import Image

from .aes_socket import AESSocket
from .ssl_socket import SSLSocket
from .errors import RobotError, RobotSessionError
from .misc import msg_waitall

logger = logging.getLogger(__name__)


def raise_error(ret):
    if ret.startswith("error ") or ret.startswith("er "):
        errors = ret.split(" ")[1:]
        if errors and errors[0] == "KICKED":
            raise RobotSessionError(*errors, error_symbol=errors)
        else:
            raise RobotError(*errors, error_symbol=errors)
    else:
        raise RobotError(ret, error_symbol="L_UNKNOW_ERROR")


def ok_or_error(fn, resp="ok"):
    def wrap(self, *args, **kw):
        ret = fn(self, *args, **kw)
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


def make_pair(item):
    if "=" in item:
        return item.split("=", 1)
    else:
        return item, None


class MaintainTaskMixIn(object):
    @ok_or_error
    def begin_maintain(self):
        return self.make_cmd(b"maintain")

    def maintain_home(self):
        self.send_cmd(b"home")
        while True:
            ret = self.get_resp()
            if ret.startswith("DEBUG"):
                logger.debug(ret[6:])
            elif ret == "ok":
                return
            else:
                raise_error(ret)

    @ok_or_error
    def maintain_reset_atmel(self):
        return self.make_cmd(b"reset_mb")

    def maintain_calibration(self, instance, threshold=None, clean=False,
                             process_callback=None):
        cmd = ['calibration']
        if threshold:
            cmd.append(str(threshold))
        if clean:
            cmd.append("clean")

        nav = self.make_cmd(" ".join(cmd).encode())
        while True:
            if nav.startswith("ok "):
                return [float(item) for item in nav.split(" ")[1:]]
            elif nav.startswith("error "):
                raise_error(nav)
            elif process_callback:
                process_callback(instance, *nav.split(" "))
            nav = self.get_resp()

    def maintain_zprobe(self, instance, process_callback=None):
        ret = self.make_cmd(b"zprobe")

        if ret == "continue":
            nav = "continue"
            while True:
                if nav.startswith("ok "):
                    return float(nav[3:])
                elif nav.startswith("error "):
                    raise_error(nav)
                elif process_callback:
                    process_callback(instance, *nav.split(" "))
                nav = self.get_resp()
        else:
            raise_error(ret)

    def maintain_manual_level(self, h):
        ret = self.make_cmd(("zprobe %.4f" % h).encode())
        if ret == "continue":
            nav = "continue"
            while True:
                if nav.startswith("ok "):
                    return float(nav[3:])
                elif nav.startswith("error "):
                    raise_error(nav)
                nav = self.get_resp()
        else:
            raise_error(ret)

    def maintain_head_info(self):
        ret = self.make_cmd(b"headinfo")
        if ret.startswith("ok "):
            return json.loads(ret[3:])
        else:
            raise_error(ret)

    def maintain_head_status(self):
        ret = self.make_cmd(b"headstatus")
        if ret.startswith("ok "):
            return json.loads(ret[3:])
        else:
            raise_error(ret)

    def maintain_set_heater(self, index, temperature):
        ret = self.make_cmd(
            ("extruder_temp %i %.1f" % (index, temperature)).encode())
        if ret != "ok":
            raise_error(ret)

    def maintain_diagnosis_sensor(self):
        ret = self.make_cmd(b"diagnosis_sensor")
        if ret.startswith("ok "):
            return dict(make_pair(item) for item in ret[3:].split("\x00"))
        else:
            raise_error(ret)

    def maintain_diagnosis(self, option):
        ret = self.make_cmd(option.encode() + b"_diagnosis")
        if ret.startswith("ok "):
            return ret[3:]
        else:
            raise_error(ret)

    def maintain_move(self, *ignore, **commands):
        params = tuple(' %s:%f' % (k, v) for k, v in commands.items() if k in 'fxyze')
        if not params:
            raise TypeError('maintain_move need at least 1 keyword argumens of f, x, y, z, e')
        ret = self.make_cmd(('move' + ''.join(params)).encode())
        if ret != "ok":
            raise_error(ret)

    def calibrate_beambox_camera(self, *ignore, **commands):
        ret = self.make_cmd('calibrate_beambox_camera'.encode())
        logger.info("calibrate_beambox_camera!!")
        if ret != "ok":
            logger.info("Return", ret)
            raise_error(ret)

    def __load_filament(self, instance, cmd, process_callback):
        ret = self.make_cmd(cmd)

        if ret == "continue":
            while True:
                try:
                    ret = self.get_resp()
                    if ret.startswith("CTRL ") and process_callback:
                        process_callback(instance, *(ret.split(" ")[1:]))
                    elif ret == "ok":
                        return
                    else:
                        raise_error(ret)
                except KeyboardInterrupt:
                    self.send_cmd(b"stop_load_filament")
                    logger.info("Interrupt load filament")
        else:
            raise_error(ret)

    def maintain_load_filament(self, instance, index, temp, process_callback):
        cmd = ("load_filament %i %.1f" % (index, temp)).encode()
        self.__load_filament(instance, cmd, process_callback)

    def maintain_load_flexible_filament(self, instance, index, temp, process_callback):
        cmd = ("load_flexible_filament %i %.1f" % (index, temp)).encode()
        self.__load_filament(instance, cmd, process_callback)

    def maintain_unload_filament(self, instance, index, temp,
                                 process_callback):
        ret = self.make_cmd(
            ("unload_filament %i %.1f" % (index, temp)).encode())

        if ret == "continue":
            while True:
                try:
                    ret = self.get_resp()
                    if ret.startswith("CTRL ") and process_callback:
                        process_callback(instance, *(ret.split(" ")[1:]))
                    elif ret == "ok":
                        return
                    else:
                        raise_error(ret)
                except KeyboardInterrupt:
                    self.send_cmd(b"stop_load_filament")
                    logger.info("Interrupt load filament")
        else:
            raise_error(ret)

    def maintain_interrupt_load_filament(self):
        self.send_cmd("stop_load_filament")

    @ok_or_error
    def maintain_extruder_temperature(self, index, temp):
        ret = self.make_cmd(
            ("extruder_temp %i %.1f" % (index, temp)).encode())
        return ret

    def maintain_update_hbfw(self, instance, stream, size,
                             process_callback=None):
        def upload_process_cb(instance, sent, total):
            if process_callback:
                process_callback(instance, "UPLOADING", sent, total)

        logger.debug("File opened")
        cmd = "update_head binary %i" % (size)
        self._upload_stream(instance, cmd, stream, size, upload_process_cb)

        ret = self.get_resp()
        if ret != "ok":
            raise_error(ret)

        while True:
            ret = self.get_resp()
            if ret == "ok":
                return
            elif ret.startswith("CTRL ") and process_callback:
                process_callback(instance, *(ret[5:].split(" ")))
            else:
                raise_error(ret)


class ScanTaskMixIn(object):
    @ok_or_error
    def begin_scan(self):
        return self.make_cmd(b"scan")

    @ok_or_error
    def scan_laser(self, left, right):
        bcmd = b"scanlaser "
        if left:
            bcmd += b"l"
        if right:
            bcmd += b"r"

        return self.make_cmd(bcmd)

    @ok_or_error
    def scan_step_length(self, l):
        cmd = "set steplen %.5f" % l
        return self.make_cmd(cmd.encode())

    def scan_check_camera(self):
        self.send_cmd(b"scan_check")
        resp = self.get_resp()
        return resp

    def scan_calibrate(self):
        self.send_cmd(b"calibrate")
        resp = self.get_resp()
        return resp

    def scan_get_calibrate(self):
        self.send_cmd(b"get_cab")
        resp = self.get_resp()
        return resp

    def scan_oneshot(self):
        self.send_cmd(b"oneshot")
        images = []
        while True:
            resp = self.get_resp()

            if resp.startswith("binary "):
                mime, img_buff = self.recv_binary_buff(resp)
                img_buff.seek(0)

                try:
                    img = Image.open(img_buff)
                except OSError:
                    raise RobotError("Image broken",
                                     error_symbol=("FILE_BROKEN", ))

                if img.size[0] >= 720:
                    img = img.transpose(Image.ROTATE_90)
                    fake_file = BytesIO()
                    img.save(fake_file, "jpeg")
                    images.append((mime, fake_file.getvalue()))
                else:
                    images.append((mime, img_buff.getvalue()))

            elif resp == "ok":
                return images

            else:
                raise_error(resp)

    def scan_images(self, stack_result=None, iterations=0):
        self.send_cmd(b"scanimages")
        images = []
        is_hd_camera = False
        img_idx = 0
        while True:
            resp = self.get_resp()
            if resp.startswith("binary "):
                mime, img_buff = self.recv_binary_buff(resp)
                img_buff.seek(0)

                try:
                    img = Image.open(img_buff)
                except OSError:
                    raise RobotError("Image broken",
                                     error_symbol=("FILE_BROKEN", ))

                if img.size[0] >= 720:
                    img = img.transpose(Image.ROTATE_90)
                    is_hd_camera = True
                    if iterations < 1:
                        # save sample image
                        images.append(img)
                    else:
                        old_img = stack_result[img_idx]
                        img = Image.blend(old_img, img, 0.5)
                        fake_file = BytesIO()
                        img.save(fake_file, "jpeg")
                        images.append((mime, fake_file.getvalue()))
                    img_idx = img_idx + 1
                else:
                    images.append((mime, img_buff.getvalue()))

            elif resp == "ok":
                if is_hd_camera and iterations < 1:
                    return self.scan_images(images, iterations + 1)

                return images

            else:
                raise_error(resp)
        logger.info("exit loop")

    @ok_or_error
    def scan_forward(self):
        return self.make_cmd(b"scan_next")

    @ok_or_error
    def scan_backward(self):
        return self.make_cmd(b"scan_backward")


class RobotBackend2(ScanTaskMixIn, MaintainTaskMixIn):
    def __init__(self, sock, client_key, device=None,
                 ignore_key_validation=False):
        self.sock = aessock = AESSocket(
            sock, client_key=client_key, device=device,
            ignore_key_validation=False)

        while aessock.do_handshake() > 0:
            pass

    def fileno(self):
        return self.sock.fileno()

    def close(self):
        self.sock.close()

    def send_binary(self, buf):
        return self.sock.send(buf)

    def send_cmd(self, buf):
        l = len(buf) + 2
        self.sock.send(struct.pack("<H", l) + buf)

    _send_cmd = send_cmd

    def get_resp(self, timeout=180.0):
        rl = select((self.sock, ), (), (), timeout)[0]
        if not rl:
            self.close()
            raise RobotSessionError("Get remote response timeout",
                                    error_symbol=["TIMEOUT"])
        bml = msg_waitall(self.sock, 2, timeout)
        if not bml:
            self.close()
            raise RobotSessionError("Message payload recv error")

        message_length = struct.unpack("<H", bml)[0]

        message = b""

        while len(message) != message_length:
            buf = self.sock.recv(message_length - len(message))

            if not buf:
                raise RobotSessionError("Recv empty message")
            message += buf

        return message.decode("utf8", "ignore")

    def make_cmd(self, buf, timeout=180.0):
        self.send_cmd(buf)
        return self.get_resp(timeout)

    _make_cmd = make_cmd

    def recv_binary_into(self, binary_header, stream, callback=None):
        mn, mimetype, ssize = binary_header.split(" ")
        if mn != "binary":
            raise RobotSessionError(
                "Binary recv error, payload broken: %r" % binary_header,
                error_symbol=["PROTOCOL_ERROR"])
        size = int(ssize)
        logger.debug("Receiving %s, length: %i", mimetype, size)
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

    def recv_binary_buff(self, binary_header, callback=None):
        buf = BytesIO()
        mimetype = self.recv_binary_into(binary_header, buf, callback)
        return (mimetype, buf)

    # file commands
    def list_files(self, path):
        entry, path = split_path(path)

        self.send_cmd(b"file ls " + entry.encode() + b" " + path.encode())
        resp = self.get_resp()
        if resp == "continue":
            files = self.get_resp()
            last_resp = self.get_resp()
            if last_resp != "ok":
                raise_error(last_resp)

            return [(node.startswith("D"), node[1:])
                    for node in files.split("\x00") if node]
        else:
            raise_error(resp)

    def file_info(self, path):
        info = [None, None]
        entry, path = split_path(path)

        self.send_cmd(b"file info " + entry.encode() + b" " + path.encode())

        resp = self.get_resp()
        images = []
        while resp.startswith("binary "):
            images.append(self.recv_binary(resp))
            resp = self.get_resp()
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
        return self.make_cmd(
            b"file mkdir " + entry.encode() + b" " + path.encode())

    @ok_or_error
    def rmdir(self, path):
        entry, path = split_path(path)
        return self.make_cmd(
            b"file rmdir " + entry.encode() + b" " + path.encode())

    @ok_or_error
    def cpfile(self, source_path, dist_path):
        source_entry, source = split_path(source_path)
        target_entry, target = split_path(dist_path)
        return self.make_cmd(
            b"file cp " + source_entry.encode() + b" " + source.encode() +
            b" " + target_entry.encode() + b" " + target.encode())

    @ok_or_error
    def rmfile(self, path):
        entry, path = split_path(path)
        return self.make_cmd(b"file rm " + entry.encode() + b" " +
                             path.encode())

    def file_md5(self, path):
        entry, path = split_path(path)
        bresp = self.make_cmd(b"file md5 " + entry.encode() + b" " +
                              path.encode())
        resp = bresp
        if resp.startswith("md5 "):
            return resp[4:]
        else:
            raise_error(resp)

    def download_file(self, path, stream, callback=None):
        entry, path = split_path(path)
        self.send_cmd(("file download %s %s" % (entry, path)).encode())
        resp = self.get_resp()
        if resp.startswith("binary "):
            mimetype = self.recv_binary_into(resp, stream, callback)
            ret = self.get_resp()
            if ret == "ok":
                return mimetype
            else:
                raise_error(ret)
        else:
            raise_error(resp)

    # player commands
    @ok_or_error
    def select_file(self, path):
        entry, path = split_path(path)
        self.send_cmd(b"player select " + entry.encode() + b" " +
                      path.encode())
        return self.get_resp()

    @ok_or_error
    def start_play(self):
        return self.make_cmd(b"player start")

    @ok_or_error
    def pause_play(self):
        return self.make_cmd(b"player pause")

    @ok_or_error
    def abort_play(self):
        return self.make_cmd(b"player abort")

    @ok_or_error
    def resume_play(self):
        return self.make_cmd(b"player resume")

    @ok_or_error
    def set_toolhead_operating_in_play(self):
        return self.make_cmd(b"player set_toolhead_operating")

    @ok_or_error
    def set_toolhead_standby_in_play(self):
        return self.make_cmd(b"player set_toolhead_standby")

    @ok_or_error
    def set_toolhead_heater_in_play(self, temp, index=0):
        return self.make_cmd(
            ("player set_toolhead_header %i %.1f" % (index, temp)).encode())

    @ok_or_error
    def load_filament_in_play(self, index):
        return self.make_cmd(("player load_filament %i" % index).encode())

    @ok_or_error
    def unload_filament_in_play(self, index):
        return self.make_cmd(("player unload_filament %i" % index).encode())

    @ok_or_error
    def press_button_in_play(self):
        return self.make_cmd(b"player press_button")

    def report_play(self):
        # TODO
        msg = self.make_cmd(b"player report")
        if msg.startswith("{"):
            return json.loads(msg)
        else:
            raise_error(msg)

    def play_info(self):
        self.send_cmd(b"player info")
        metadata = None

        resp = self.get_resp()
        if resp.startswith("binary text/json"):
            _, bm = self.recv_binary(resp)
            metadata = json.loads(bm.decode("utf8", "ascii"))
        else:
            raise_error(resp)

        images = []
        while True:
            resp = self.get_resp()
            if resp.startswith("binary "):
                images.append(self.recv_binary(resp))
            elif resp == "ok":
                return metadata, images
            else:
                raise_error(resp)

    @ok_or_error
    def restart_play(self):
        return self.make_cmd(b"player restart")

    @ok_or_error
    def quit_play(self):
        return self.make_cmd(b"player quit")

    def _upload_stream(self, instance, cmd, stream, size,
                       process_callback=None):
        if cmd:
            upload_ret = self.make_cmd(cmd.encode())

            if upload_ret == "continue":
                logger.info(upload_ret)
            else:
                raise_error(upload_ret)

        logger.debug("Upload stream length: %i" % size)
        sent = 0
        ts = 0

        if process_callback:
            process_callback(instance, sent, size)

        while sent < size:
            buf = stream.read(4096)
            lbuf = len(buf)
            if lbuf == 0:
                raise RobotError("Upload file error")
            sent += lbuf
            self.send_binary(buf)

            if process_callback and time() - ts > 1.0:
                ts = time()
                process_callback(instance, sent, size)

        if process_callback:
            process_callback(instance, sent, size)

    def upload_stream(self, instance, stream, mimetype, size, upload_to=None,
                      process_callback=None):
        if upload_to == "#":
            # cmd = [cmd] [mimetype] [length] #
            cmd = "file upload %s %i #" % (mimetype, size)
        elif upload_to:
            if upload_to.startswith("/"):
                entry, path = upload_to[1:].split("/", 1)
            else:
                entry, path = upload_to.split("/", 1)
            # cmd = [cmd] [mimetype] [length] [entry] [path]
            cmd = "file upload %s %i %s %s" % (mimetype, size, entry, path)
        else:
            cmd = "file upload %s %i" % (mimetype, size)

        self._upload_stream(instance, cmd, stream, size, process_callback)
        final_ret = self.get_resp()
        if final_ret != "ok":
            raise_error(final_ret)

    def yihniwimda_upload_stream(self, mimetype, length, upload_to=None):
        if upload_to == "#":
            # cmd = [cmd] [mimetype] [length] #
            cmd = "file upload %s %i #" % (mimetype, length)
        elif upload_to:
            if upload_to.startswith("/"):
                entry, path = upload_to[1:].split("/", 1)
            else:
                entry, path = upload_to.split("/", 1)
            # cmd = [cmd] [mimetype] [length] [entry] [path]
            cmd = "file upload %s %i %s %s" % (mimetype, length, entry, path)
        else:
            cmd = "file upload %s %i" % (mimetype, length)

        upload_ret = self.make_cmd(cmd.encode())
        if upload_ret == "continue":
            logger.info(upload_ret)
        if upload_ret != "continue":
            raise_error(upload_ret)

        self._sent = 0

        def feeder(buf):
            lbuf = len(buf)
            if lbuf == 0:
                raise RobotSessionError("Upload file error")
            self._sent += lbuf
            self.send_binary(buf)
            return self._sent

        while self._sent < length:
            yield feeder

        final_ret = self.get_resp()
        logger.debug("File uploaded")
        if final_ret != "ok":
            raise_error(final_ret)

    def update_firmware(self, instance, stream, size, process_callback=None):
        cmd = "update_fw binary/flux-firmware %i #" % (size)
        self._upload_stream(instance, cmd, stream, size, process_callback)
        final_ret = self.get_resp(timeout=45.0)
        if final_ret != "ok":
            raise_error(final_ret)

    def update_atmel(self, instance, stream, size, process_callback=None):
        cmd = "update_mbfw binary/flux-firmware %i #" % (size)
        self._upload_stream(instance, cmd, stream, size, process_callback)
        final_ret = self.get_resp()
        if final_ret != "ok":
            raise_error(final_ret)

    def begin_icontrol(self):
        ret = self.make_cmd(b"task icontrol")
        if ret == "ok":
            return self.sock
        else:
            raise_error(ret)

    @ok_or_error
    def quit_task(self):
        return self.make_cmd(b"quit")

    @ok_or_error
    def kick(self):
        return self.make_cmd(b"kick")

    def begin_raw(self):
        ret = self.make_cmd(b"task raw")
        if ret == "continue":
            return self.sock
        else:
            raise_error(ret)

    @ok_or_error
    def quit_raw_mode(self):
        self.send_binary(b"quit")
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
        return self.make_cmd(b"config set " + key.encode() + b" " +
                             value.encode())

    def config_get(self, key):
        ret = self.make_cmd(b"config get " + key.encode())
        if ret.startswith("ok VAL "):
            return ret[7:]
        elif ret.startswith("ok EMPTY"):
            return None
        else:
            raise_error(ret)

    @ok_or_error
    def config_del(self, key):
        return self.make_cmd(b"config del " + key.encode())

    def deviceinfo(self):
        ret = self.make_cmd(b"deviceinfo")
        if ret.startswith("ok\n"):
            info = {}
            for raw in ret[3:].split("\n"):
                if ":" in raw:
                    key, val = raw.split(":", 1)
                    info[key] = val
            if "cloud" in info:
                try:
                    # TODO:...
                    info["cloud"] = eval(info["cloud"])
                except Exception:
                    pass
            return info
        else:
            raise_error(ret)

    def fetch_log(self, path, stream, callback=None):
        self.send_cmd(("fetch_log %s" % path).encode())
        resp = self.get_resp()
        if resp.startswith("binary "):
            mimetype = self.recv_binary_into(resp, stream, callback)
            ret = self.get_resp()
            if ret == "ok":
                return mimetype
            else:
                raise_error(ret)
        else:
            raise_error(resp)

    def get_cloud_validation_code(self):
        ret = self.make_cmd(b"cloud_validation_code")
        if ret.startswith("ok "):
            token, validate_b64_hash = ret[3:].split(" ")
            return token, a2b_base64(validate_b64_hash)
        else:
            raise_error(ret)


class RobotBackend3(RobotBackend2):
    def __init__(self, sock, client_key, device=None,
                 ignore_key_validation=False):
        self.sock = sslsock = SSLSocket(
            sock, client_key=client_key, device=device,
            ignore_key_validation=False)

        while sslsock.do_handshake() > 0:
            pass
