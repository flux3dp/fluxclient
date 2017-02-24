
from tempfile import NamedTemporaryFile
from select import select
from time import time
import mimetypes
import logging
import shlex
import os

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class RobotConsole(object):
    _mode = "standard"
    task = None
    _thread = None
    _raw_sock = None

    def __init__(self, robot_obj):
        self.robot_obj = robot_obj
        self.simple_mapping = {
            "start": robot_obj.start_play,
            "pause": robot_obj.pause_play,
            "resume": robot_obj.resume_play,
            "abort": robot_obj.abort_play,
            "report": robot_obj.report_play,
            "kick": robot_obj.kick,

            "play": {
                "quit": robot_obj.quit_play,
                "toolhead": {
                    "on": robot_obj.set_toolhead_operating_in_play,
                    "off": robot_obj.set_toolhead_standby_in_play
                }
            },
        }

        self.cmd_mapping = {
            "deviceinfo": self.deviceinfo,
            "ls": self.list_file,
            "fileinfo": self.fileinfo,
            "mkdir": self.mkdir,
            "rmdir": self.rmdir,
            "rmfile": self.rmfile,
            "cp": self.cpfile,
            "download": self.download_file,
            "upload": self.upload_file,
            "md5": self.md5,

            "fetch_log": self.fetch_log,
            "cloud_validation": self.get_cloud_validation_code,

            "select": self.select_file,
            "update_fw": self.update_fw,
            "update_atmel": self.update_atmel,
            "config": {
                "set": self.config_set,
                "get": self.config_get,
                "del": self.config_del
            },

            "calib": self.maintain_calibration,
            "zprobe": self.maintain_zprobe,
            "filament": {
                "load": self.load_filament,
                "unload": self.unload_filament
            },
            "extruder_temp": self.maintain_extruder_temp,
            "update_hbfw": self.maintain_update_hbfw,
            "play": {
                "info": self.play_info,
                "toolhead": {
                    "enable": self.play_enable_toolhead,
                    "disable": self.play_disable_toolhead,
                    "heater": self.play_set_toolhead_heater
                }
            },

            # Maintain Tasks
            "home": self.maintain_home,
            "reset": {
                "hardware": self.maintain_reset_hardware
            },
            "headinfo": self.maintain_headinfo,
            "headstatus": self.maintain_headstatus,
            "diagnosis_sensor": self.maintain_diagnosis_sensor,

            "scan": {
                "begin": self.begin_scan,
                "oneshot": self.scan_oneshot,
                "images": self.scanimages,
                "backward": self.scan_backward,
                "forward": self.scan_forward,
                "laser": self.scan_laser,
                "step": self.scan_step,
                "check": self.scan_check_camera,
            },

            "maintain": self.begin_maintain,
            "raw": self.raw_mode,
            "quit": self.quit_task,

            "help": self.print_help,
        }

    def call_command(self, ref, args, wrapper=None):
        if not args:
            return False
        cmd = args[0]
        if cmd in ref:
            obj = ref[cmd]
            if isinstance(obj, dict):
                return self.call_command(obj, args[1:], wrapper)
            else:
                if wrapper:
                    wrapper(obj, *args[1:])
                else:
                    obj(*args[1:])
                return True
        return False

    def on_cmd(self, arguments):
        if self._mode == "raw":
            if arguments == "quit":
                self.quit_raw_mode()
            else:
                self._raw_sock.send(arguments.encode() + b"\n")
        else:
            args = shlex.split(arguments)

            try:
                if self.call_command(self.simple_mapping, args,
                                     self.simple_cmd):
                    pass
                elif self.call_command(self.cmd_mapping, args):
                    pass
                else:
                    logger.error("Unknow Command: %s", arguments)

            except RuntimeError as e:
                logger.error("RuntimeError%s" % repr(e.args))

    def simple_cmd(self, func_ptr, *args):
        ret = func_ptr(*args)
        if ret:
            logger.info(ret)
        else:
            logger.info("ok")

    def deviceinfo(self):
        for k, v in self.robot_obj.deviceinfo.items():
            logger.info("    %s: %s", k, v)

    def list_file(self, path):
        for is_dir, node in self.robot_obj.list_files(path):
            if is_dir:
                logger.info("DIR %s" % os.path.join(path, node))
            else:
                logger.info("FILE %s" % os.path.join(path, node))
        logger.info("ls done.")

    def fetch_log(self, source, target):
        def callback(left, size):
            logger.info("Download %i / %i" % (size - left, size))

        with open(target, "wb") as f:
            self.robot_obj.fetch_log(source, f, callback)

    def get_cloud_validation_code(self):
        code = self.robot_obj.get_cloud_validation_code()
        for key, value in code.items():
            logger.info("  %s=%s", key, value)
        logger.info("done")

    def select_file(self, path):
        path = shlex.split(path)[0]
        entry, filename = path.split("/", 1)
        self.robot_obj.select_file(path)
        logger.info("ok")

    def fileinfo(self, path):
        info, images = self.robot_obj.file_info(path)
        for key, value in info.items():
            if len(value) > 30:
                logger.info("    :%s => %s", key, value[:30])
            else:
                logger.info("    :%s => %s", key, value)
        logger.info("%s" % info)

        previews = []
        for img in images:
            ext = mimetypes.guess_extension(img[0])
            if ext:
                ntf = NamedTemporaryFile(suffix=ext, delete=False)
                ntf.write(img[1])
                previews.append(ntf.name)
        if previews:
            os.system("open " + " ".join(previews))

    def mkdir(self, path):
        self.simple_cmd(self.robot_obj.mkdir, path)

    def rmdir(self, path):
        self.simple_cmd(self.robot_obj.rmdir, path)

    def rmfile(self, path):
        self.simple_cmd(self.robot_obj.rmfile, path)

    def cpfile(self, source, target):
        self.simple_cmd(self.robot_obj.cpfile, source, target)

    def download_file(self, source, target):
        def callback(left, size):
            if time() - callback.c > 0.9:
                callback.c = time()
                logger.info("Download %i / %i" % (size - left, size))
            elif left == 0:
                logger.info("Download %i / %i" % (size, size))

        start_at = time()
        callback.c = start_at
        with open(target, "wb") as f:
            self.robot_obj.download_file(source, f, callback)
        logger.info("Done, spent %.2f seconds", time() - start_at)

    def upload_file(self, source, upload_to="#"):
        start_at = time()
        self.robot_obj.upload_file(
            source, upload_to, process_callback=self.log_process_callback)
        logger.info("Done, spent %.2f seconds", time() - start_at)

    def update_fw(self, filename):
        with open(filename, "rb") as f:
            size = os.fstat(f.fileno()).st_size
            self.robot_obj.update_firmware(
                f, size, process_callback=self.log_process_callback)

    def update_atmel(self, filename):
        with open(filename, "rb") as f:
            size = os.fstat(f.fileno()).st_size
            self.robot_obj._backend.update_atmel(
                f, size, process_callback=self.log_process_callback)

    def md5(self, filename):
        md5 = self.robot_obj.file_md5(filename)
        logger.info("MD5 %s %s", filename, md5)

    def play_info(self):
        metadata, images = self.robot_obj.play_info()
        logger.info("Metadata:")
        for k, v in metadata.items():
            logger.info("  %s=%s", k, v)
        tempfiles = []
        if images:
            for mime, buf in images:
                ext = mimetypes.guess_extension(mime)
                if ext:
                    ntf = NamedTemporaryFile(suffix=".jpg", delete=False)
                    ntf.write(buf)
                    tempfiles.append(ntf)
            os.system("open " + " ".join([n.name for n in tempfiles]))

    def scan_oneshot(self, filename=None):
        images = self.task.oneshot()
        tempfiles = []
        for mime, buf in images:
            ext = mimetypes.guess_extension(mime)
            if ext:
                ntf = NamedTemporaryFile(suffix=".jpg", delete=False)
                ntf.write(buf)
                tempfiles.append(ntf)

        os.system("open " + " ".join([n.name for n in tempfiles]))

    def scanimages(self, filename=None):
        images = self.task.scanimages()
        tempfiles = []
        for mime, buf in images:
            ext = mimetypes.guess_extension(mime)
            if ext:
                ntf = NamedTemporaryFile(suffix=".jpg", delete=False)
                ntf.write(buf)
                tempfiles.append(ntf)

        os.system("open " + " ".join([n.name for n in tempfiles]))

    def scan_forward(self):
        self.task.forward()
        logger.info("ok")

    def scan_backward(self):
        self.task.backward()
        logger.info("ok")

    def scan_laser(self, flags=""):
        flags = flags.lower()
        self.task.laser("l" in flags, "r" in flags)
        logger.info("ok")

    def scan_step(self, length):
        self.task.step_length(float(length))
        logger.info("ok")

    def scan_check_camera(self):
        logger.info("%s", self.task.check_camera())

    def config_set(self, key, value):
        self.robot_obj.config_set(key, value)
        logger.info("ok")

    def config_get(self, key):
        value = self.robot_obj.config_get(key)
        if value:
            logger.info("%s=%s\nok" % (key, value))
        else:
            logger.info("%s not set\nok" % key)

    def config_del(self, key):
        self.robot_obj.config_del(key)
        logger.info("ok")

    def load_filament(self, index, temp):
        if self.task:
            def callback(instance, *nav):
                logger.info("OPERATION: %s", nav)

            self.task.load_filament(int(index), float(temp), callback)
            logger.info("ok")
        else:
            self.robot_obj.load_filament_in_play(int(index))

    def unload_filament(self, index, temp=None):
        if self.task:
            def callback(instance, *nav):
                logger.info("OPERATION: %s", nav)

            self.task.unload_filament(int(index), float(temp), callback)
            logger.info("ok")
        else:
            self.robot_obj.unload_filament_in_play(int(index))

    def maintain_home(self):
        self.task.home()
        logger.info("ok")

    def maintain_reset_hardware(self):
        self.task.reset_hardware()
        logger.info("ok")

    def maintain_calibration(self, *args):
        def callback(instance, *nav):
            logger.info("%s", " ".join(nav))

        clean = "clean" in args
        try:
            threshold = float(args[0])
        except (IndexError, ValueError):
            threshold = None

        ret = self.task.calibration(threshold=threshold, clean=clean,
                                    process_callback=callback)
        data_str = ", ".join(("%.4f" % i for i in ret))
        logger.info("Data: %s, Error: %.4f", data_str, (max(*ret) - min(*ret)))
        logger.info("ok")

    def maintain_zprobe(self, manual_h=None):
        def callback(instance, *nav):
            logger.info("%s", " ".join(nav))

        if manual_h:
            ret = self.task.manual_level(float(manual_h))
        else:
            ret = self.task.zprobe(process_callback=callback)
            logger.info("Data: %s", ret)

        logger.info("ok")

    def maintain_extruder_temp(self, sindex, stemp):
        self.task.set_extruder_temperature(int(sindex), float(stemp))
        logger.info("ok")

    def maintain_update_hbfw(self, filename):
        def callback(instance, status, *args):
            if status == "UPLOADING":
                logger.info("  UPLOADING %i / %i (%.2f%%)",
                            args[0], args[1], args[0] / args[1] * 100)
            else:
                logger.info("  %s %s", status, args)

        mimetype, _ = mimetypes.guess_type(filename)
        if not mimetype:
            mimetype = "binary"
        with open(filename, "rb") as f:
            size = os.fstat(f.fileno()).st_size
            self.task.update_hbfw(f, size, callback)
        logger.info("ok")

    def maintain_headinfo(self):
        for key, value in self.task.head_info().items():
            logger.info(" :%s => %s", key, value)
        logger.info("ok")

    def maintain_headstatus(self):
        for key, value in self.task.head_status().items():
            logger.info("    :%s => %s", key, value)
        logger.info("ok")

    def maintain_diagnosis_sensor(self):
        for key, value in self.task.diagnosis_sensor().items():
            logger.info("    :%s => %s", key, value)
        logger.info("ok")

    def play_enable_toolhead(self):
        self.robot_obj.set_toolhead_operating_in_play()

    def play_disable_toolhead(self):
        self.robot_obj.set_toolhead_standby_in_play()

    def play_set_toolhead_heater(self, temp, index="0"):
        self.robot_obj.set_toolhead_heater_in_play(float(temp), int(index))

    def begin_scan(self):
        self.task = self.robot_obj.scan()
        logger.info("ok")

    def begin_maintain(self):
        self.task = self.robot_obj.maintain()
        logger.info("ok")

    def quit_task(self):
        self.task.quit()
        logger.info("ok")

    def raw_mode(self):
        import threading
        self.task = self.robot_obj.raw()
        self._raw_sock = self.task.sock
        self._mode = "raw"

        self._thread = threading.Thread(target=self.__raw_mode_thread)
        self._thread.setDaemon(True)
        self._thread.start()
        logger.info("RAW %s>", "=" * 16)

    def quit_raw_mode(self):
        self._mode = "standard"
        self._raw_sock = None
        if self._thread:
            self._thread.join()

        self.task.quit()
        logger.info("<%s RAW", "=" * 16)

    def log_process_callback(self, robot, progress, total):
        logger.info("Processing %3.1f %% (%i of %i)" %
                    (progress / total * 100.0, progress, total))

    def print_help(self, groups=""):
        if groups == "file":
            logger.info("File related commands:")
            logger.info("  'ls [path]' - List files on device, path is always "
                        "starts with /SD or /USB.")
            logger.info("  'fileinfo [path]' - Get f-code metadata")
            logger.info("  'mkdir [path]' - Create dir on device.")
            logger.info("  'rmdir [path]' - Remove dir on device.")
            logger.info("  'cp [source path] [target path]' - Copy file in "
                        "device. Note: files in /USB is readonly.")
            logger.info("  'download [remote path] [local path]' - Download "
                        "file from device to local.")
            logger.info("  'upload [local path] [remote path]' - Upload file "
                        "into device. If remote path is not given, file will "
                        "put into cache and can be play directory.")
            logger.info("  'md5 [path]' - Get file md5.")
            logger.info("  'rmfile [path]' - Delete file.")
        elif groups == "play":
            logger.info("Play related commands:")
            logger.info("  'select [path]' - Select a f-code file to play.")
            logger.info("  'start' - Start a task.")
            logger.info("  'pause' - Pause a task.")
            logger.info("  'resume' - Resume a task.")
            logger.info("  'abort' - Cancel a task.")
            logger.info("  'play info' - Get current playing task informations"
                        ".")
            logger.info("  'play quit' - Terminate and clean a playing task. "
                        "Note: Can be use when completed or aborted.")
        elif groups == "maintain":
            logger.info("Maintain related commands:")
            logger.info("  'maintain' - Begin maintain mode. use 'quit' "
                        "command to quit maintain mode. Only maintain commands"
                        "allowed in matain mode.")
            logger.info("  'home' - Home")
            logger.info("  'calib [threshold] [clean]' - Do calibration, "
                        "threshold is optional, if clean given, device will do"
                        "calibration from clean status.")
            logger.info("  'zprobe' - Make a zprobe")
            logger.info("  'filament load [index] [temp]' - Load filament"
                        " index should be 0, temp is temperature")
            logger.info("  'filament unload [index] [temp]' - Unload filament"
                        " index should be 0, temp is temperature")
            logger.info("  'extruder_temp [index] [temp]' - Set toolhead "
                        "temperature")
            logger.info("  'headinfo' - Get toolhead informations")
            logger.info("  'headstatus' - Get toolhead status")
            logger.info("  'update_hbfw [local path]' - Update toolhead "
                        "firmware.")
        else:
            logger.info("Commands:")
            logger.info("  'deviceinfo' - Print device informations")
            logger.info("  'update_fw [local file]' - Upload a firmware and "
                        "update device.")
            logger.info("  'kick' - Kick any other user who is using maintain"
                        "/scan functions.")
            logger.info("  'report' - Print device current status")
            logger.info("  'help files' - Print file related commands")
            logger.info("  'help play' - Print play related commands")
            logger.info("  'help maintain' - Print maintain related commands")

    def __raw_mode_thread(self):
        try:
            while self._mode == "raw":
                rl = select((self._raw_sock, ), (), (), 0.1)[0]
                if rl:
                    buf = rl[0].recv(4096)
                    if buf:
                        msg = buf.decode("utf8", "replace")
                        for ln in msg.split("\n"):
                            logger.info(ln.rstrip("\r\x00"))
                    else:
                        logger.error("Connection closed")
                        return

        except Exception:
            self._mode == "standard"
            logger.exception("Raw mode fatel, your session my be broken")
