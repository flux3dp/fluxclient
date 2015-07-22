
from tempfile import NamedTemporaryFile
from select import select
import logging
import socket
import os

logger = logging.getLogger(__name__)


class RobotConsole(object):
    _mode = "standard"
    _thread = None
    _raw_sock = None

    def __init__(self, robot_obj):
        self.robot_obj = robot_obj
        self.simple_mapping = {
            "select": robot_obj.select_file,
            "start": robot_obj.start_play,
            "pause": robot_obj.pause_play,
            "resume": robot_obj.resume_play,
            "abort": robot_obj.abort_play,
            "report": robot_obj.report_play,
            "position": robot_obj.position,
            "quit": robot_obj.quit_task,
            "kick": robot_obj.kick,

            "scan": robot_obj.begin_scan,
            "scan_forword": robot_obj.scan_forword,
            "scan_next": robot_obj.scan_next,

            "maintain": robot_obj.begin_maintain,
            "home": robot_obj.maintain_home,

        }

        self.cmd_mapping = {
            "ls": self.list_file,
            "upload": self.upload_file,
            "oneshot": self.oneshot,
            "scanimages": self.scanimages,
            "raw": self.raw_mode,
            "set": self.set_setting,
        }

    def on_cmd(self, arguments):
        if self._mode == "raw":
            if arguments == "quit":
                self.quit_raw_mode()
            else:
                self._raw_sock.send(arguments.encode() + b"\n")
        else:
            args = arguments.split(" ", 1)
            cmd = args[0]

            try:
                if cmd in self.simple_mapping:
                    self.simple_cmd(self.simple_mapping[cmd], *args[1:])
                elif cmd in self.cmd_mapping:
                    func_ptr = self.cmd_mapping[cmd]
                    func_ptr(*args[1:])
                else:
                    logger.error("Unknow Command Q")

            except RuntimeError as e:
                logger.error("RuntimeError%s" % repr(e.args))

    def simple_cmd(self, func_ptr, *args):
        logger.info(func_ptr(*args))

    def list_file(self):
        for f in self.robot_obj.list_file():
            logger.info(f)
        logger.info("ok")

    def upload_file(self, filename):
        self.robot_obj.upload_file(
            filename.rstrip(), progress_callback=self.log_progress_callback)

    def oneshot(self, filename=None):
        images = self.robot_obj.oneshot()
        tempfiles = []
        for mime, buf in images:
            ntf = NamedTemporaryFile(suffix=".jpg", delete=False)
            ntf.write(buf)
            tempfiles.append(ntf)

        os.system("open " + " ".join([n.name for n in tempfiles]))

    def scanimages(self, filename=None):
        images = self.robot_obj.scanimages()
        tempfiles = []
        for mime, buf in images:
            ntf = NamedTemporaryFile(suffix=".jpg", delete=False)
            ntf.write(buf)
            tempfiles.append(ntf)

        os.system("open " + " ".join([n.name for n in tempfiles]))

    def set_setting(self, line):
        params = line.split(" ")
        if len(params) == 2:
            logger.info(
                self.robot_obj.set_setting(key=params[0], value=params[1])
            )
        else:
            logger.info("BAD_PARAMS")

    def raw_mode(self):
        import threading
        self._raw_sock = self.robot_obj.raw_mode()
        self._mode = "raw"

        self._thread = threading.Thread(target=self.__raw_mode_thread)
        self._thread.setDaemon(True)
        self._thread.start()
        logger.info("raw mode ->")

    def quit_raw_mode(self):
        self._mode = "standard"
        sock = self._raw_sock
        self._raw_sock = None
        if self._thread:
            self._thread.join()
        sock.send(b"quit")
        logger.info(self.robot_obj.get_resp().decode("ascii", "ignore"))

    def log_progress_callback(self, robot, progress, total):
        logger.info("Processing %3.1f %% (%i of %i)" %
                    (progress / total * 100.0, progress, total))

    def __raw_mode_thread(self):
        try:
            while self._mode == "raw":
                rl = select((self._raw_sock, ), (), (), 0.1)[0]
                if rl:
                    buf = rl[0].recv(4096).decode("utf8", "ignore")
                    if buf:
                        logger.info(buf.rstrip("\n\x00"))
                    else:
                        logger.error("Connection closed")
                        return

        except Exception:
            self._mode == "standard"
            logger.exception("Raw mode fatel, your session my be broken")
