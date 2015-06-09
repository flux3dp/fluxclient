
import logging

logger = logging.getLogger(__name__)


class RobotConsole(object):
    def __init__(self, robot_obj):
        self.robot_obj = robot_obj
        self.simple_mapping = {
            "start": robot_obj.start_play,
            "pause": robot_obj.pause_play,
            "resume": robot_obj.resume_play,
            "abort": robot_obj.abort_play,
            "report": robot_obj.report_play,
            "quit": robot_obj.quit_task
        }

        self.cmd_mapping = {
            "ls": self.list_file,
            "upload": self.upload_file
        }

    def on_cmd(self, arguments):
        args = arguments.split(" ", 1)
        cmd = args[0]

        try:
            if cmd in self.simple_mapping:
                self.simple_cmd(self.simple_mapping[cmd], *args[1:])
            elif cmd in self.cmd_mapping:
                func_ptr = self.cmd_mapping[cmd]
                func_ptr(*args[1:])
            else:
                logger.error("Unknow Command")

        except RuntimeError as e:
            logger.error(" ".join(*e.args))

    def simple_cmd(self, func_ptr, *args):
        logger.info(func_ptr(*args))

    def list_file(self):
        logger.error(self.robot_obj.list_file())

    def upload_file(self, filename):
        self.robot_obj.upload_file(filename.rstrip(),
                              progress_callback=self.log_progress_callback)

    def log_progress_callback(self, robot, progress, total):
        logger.info("Processing %3.1f %% (%i of %i)" %
                    (progress / total * 100.0, progress, total))
