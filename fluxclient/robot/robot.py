
from functools import wraps
import os

from fluxclient.utils import mimetypes
from .backends import InitBackend
from .errors import RobotError, RobotSessionError
from .robot_backend_2 import RobotBackend2


def blocked_validator(fn):
    @wraps(fn)
    def wrapper(self, *args, **kw):
        if not self._backend:
            raise RobotSessionError("Device is already closed",
                                    error_symbol=("OPERATION_ERROR", ))
        if self._locked_obj:
            raise RobotError("Device is busy for %s" % self._locked_obj,
                             error_symbol=("OPERATION_ERROR", ))
        else:
            try:
                return fn(self, *args, **kw)
            except RobotSessionError:
                self.close()
                raise
    return wrapper


def invalied_validator(fn):
    @wraps(fn)
    def wrapper(self, *args, **kw):
        if self._actived:
            return fn(self, *args, **kw)
        else:
            raise RobotError("This operation is invalied.",
                             error_symbol=("OPERATION_ERROR", ))
    return wrapper


class FluxRobot(object):
    _device = None
    _backend = None
    _config_obj = None
    _locked_obj = None

    def __init__(self, endpoint, client_key, device=None,
                 ignore_key_validation=False):
        self._device = device

        init_backend = InitBackend(endpoint)
        proto_ver = init_backend.do_handshake()

        if proto_ver == 2:
            self._backend = RobotBackend2(
                self, init_backend.sock, client_key, device,
                ignore_key_validation)
        else:
            raise RobotSessionError("Protocol not support")

    @property
    def device(self):
        return self._device

    @blocked_validator
    def list_files(self, path):
        """List file on device.

    :param str path: Path on device, must start with /SD/ or /USB/
    :return: An array of files and directories. Each array element is a \
tuple, first is a boolean present this name is a folder or not and second is \
name of the file or folder.
    :rtype: list"""
        return self._backend.list_files(path)

    @blocked_validator
    def file_info(self, path):
        """Get f-code file information.

    :param str path: File path on device
    :return: [ \
        {"f-code-metadata-key": "f-code-metada-value", ...}, \
        [("image", b'image buffer object'), ...] \
    ] \
    The first return element in array is a key-value information of f-code \
    the second is a list of preview images. Image list length may be 0.
    :rtype: list"""
        return self._backend.file_info(path)

    @blocked_validator
    def file_md5(self, path):
        """Get file md5 on device.

    :param str path: File path on device
    :return: MD5 hex
    :rtype: str"""
        return self._backend.file_md5(path)

    @blocked_validator
    def mkdir(self, path):
        """Create folder on device storage.

    :param str path: Path to create"""
        return self._backend.mkdir(path)

    @blocked_validator
    def rmdir(self, path):
        """Remove folder on device storage.

    :param str path: Path to remove"""
        return self._backend.rmdir(path)

    @blocked_validator
    def cpfile(self, source_path, dist_path):
        """Copy file on device.

    :param str source_path: File can be on device or usb disk
    :param str dist_path: File can copy into device only"""
        return self._backend.cpfile(source_path, dist_path)

    @blocked_validator
    def rmfile(self, path):
        """Remove file on device.

    :param str path: File to be delete"""
        return self._backend.rmfile(path)

    @blocked_validator
    def download_file(self, path, stream, process_callback=None):
        """Download file from device.

    :param str path: File on device for download
    :param filelike_obj stream: A local file-like object to write
    :param function process_callback: A callable object which will be invoke \
during download progress"""
        return self._backend.download_file(path, stream, process_callback)

    @blocked_validator
    def upload_file(self, filename, upload_to="#", process_callback=None):
        """Upload file to device.

    :param str filename: File to upload to device
    :param str upload_to: Path on device to upload to
    :param function process_callback: A callable object which will be invoke \
during upload progress"""
        mimetype, _ = mimetypes.guess_type(filename)
        if not mimetype:
            mimetype = "binary"
        with open(filename, "rb") as f:
            size = os.fstat(f.fileno()).st_size
            return self.upload_stream(f, mimetype, size, upload_to,
                                      process_callback)

    @blocked_validator
    def upload_stream(self, stream, mimetype, size, upload_to="#",
                      process_callback=None):
        """Upload file to device.

    :param file stream: Filelike object
    :param str mimetype: Contents mimetype
    :param int size: Contents size
    :param str upload_to: Path on device to upload to
    :param function callback: A callable object which will be invoke during \
upload progress"""
        return self._backend.upload_stream(
            stream, mimetype, size, upload_to,
            process_callback=process_callback)

    @blocked_validator
    def yihniwimda_upload_stream(self, mimetype, size, upload_to="#"):
        """Yes, I have no idea what I'm doing about upload stream. You can \
use it if and only if you have any idea about this::

    for feeder in robot.yihniwimda_upload_stream(...):
        recived_size = feeder(buffer)
        print("%i bytes sent" % recived_size)
    print("Done")"""
        return self._backend.yihniwimda_upload_stream(
            mimetype, size, upload_to)

    @blocked_validator
    def select_file(self, path):
        """Select a file to play.

    :param str path: Path on device"""
        return self._backend.select_file(path)

    @blocked_validator
    def start_play(self):
        """Start play."""
        return self._backend.start_play()

    @blocked_validator
    def pause_play(self):
        """Pause play."""
        return self._backend.pause_play()

    @blocked_validator
    def abort_play(self):
        """Abort play."""
        return self._backend.abort_play()

    @blocked_validator
    def resume_play(self):
        """Resume play."""
        return self._backend.resume_play()

    @blocked_validator
    def report_play(self):
        """Report play."""
        return self._backend.report_play()

    @blocked_validator
    def play_info(self):
        """Play info."""
        return self._backend.play_info()

    @blocked_validator
    def quit_play(self):
        """Quit play."""
        return self._backend.quit_play()

    @property
    def deviceinfo(self):
        """Get device informations"""
        return self._backend.deviceinfo()

    @property
    def config(self):
        """Get a subscriptable device config object."""
        if not self._config_obj:
            self._config_obj = RobotConfigure(self)
        return self._config_obj

    @blocked_validator
    def config_set(self, key, value):
        return self._backend.config_set(key, value)

    @blocked_validator
    def config_get(self, key):
        return self._backend.config_get(key)

    @blocked_validator
    def config_del(self, key):
        return self._backend.config_del(key)

    @blocked_validator
    def kick(self):
        """Kick a work session which is occupy the device."""
        return self._backend.kick()

    @blocked_validator
    def update_firmware(self, stream, size, process_callback=None):
        """Upload and update flux device firmware"""
        return self._backend.update_firmware(stream, size, process_callback)

    @blocked_validator
    def scan(self):
        """Begin a scan task and return a scan object"""
        return ScanTasks(self)

    @blocked_validator
    def maintain(self):
        """Begin a maintain task and return a scan object"""
        return MaintainTasks(self)

    @blocked_validator
    def icontrol(self):
        def exit_callback():
            self._locked_obj = None

        sock = self._backend.begin_icontrol()
        self._locked_obj = "iControl"
        from fluxclient.sdk.delta import Delta
        return Delta(sock, exit_callback)

    @blocked_validator
    def raw(self):
        return RawTasks(self)

    def close(self):
        """Close device connection"""
        if self._backend:
            self._backend.close()
            self._backend = None


class RobotConfigure(object):
    __robot = None

    def __init__(self, robot):
        self.__robot = robot

    def __getitem__(self, key):
        return self.__robot.config_get(key)

    def __setitem__(self, key, value):
        self.__robot.config_set(key, value)

    def __delitem__(self, key):
        self.__robot.config_del(key)


class SubTasks(object):
    _robot = None
    _backend = None
    _actived = None

    def __init__(self, robot):
        self._robot = robot
        self._backend = robot._backend

        self._enter_task()

        self._actived = True
        self._robot._locked_obj = self

    def _enter_task(self):
        raise RuntimeError("Not implement")

    @property
    def activated(self):
        return self._actived

    def cleanup(self):
        self._actived = False

        if self._robot._locked_obj == self:
            self._robot._locked_obj = None
        else:
            raise SystemError("Lock obj contradiction")

    def quit(self):
        """Quit task"""
        self._backend.quit_task()
        self.cleanup()

    def __enter__(self):
        if self._actived is not True:
            raise RobotError("Task invailed",
                             error_symbol=("OPERATION_ERROR", ))

    def __exit__(self, type, value, traceback):
        if self._actived is True:
            self.quit()
            self._actived = False

            if self.__robot._locked_obj == self:
                self.__robot._locked_obj = None
            else:
                raise SystemError("Lock obj contradiction")


class MaintainTasks(SubTasks):
    def _enter_task(self):
        self._backend.begin_maintain()

    @invalied_validator
    def home(self):
        """Move to home position."""
        return self._backend.maintain_home()

    @invalied_validator
    def reset_hardware(self):
        self._backend.maintain_reset_atmel()
        self.cleanup()

    @invalied_validator
    def calibration(self, threshold=None, clean=False, process_callback=None):
        """Do a calibration"""
        return self._backend.maintain_calibration(threshold, clean,
                                                  process_callback)

    @invalied_validator
    def zprobe(self, process_callback=None):
        """Do a zprobe"""
        return self._backend.maintain_zprobe(process_callback)

    @invalied_validator
    def manual_level(self, h):
        """Set heigh level"""
        return self._backend.maintain_manual_level(h)

    @invalied_validator
    def head_info(self):
        """Get toolhead info"""
        return self._backend.maintain_head_info()

    @invalied_validator
    def head_status(self):
        """Get toolhead status"""
        return self._backend.maintain_head_status()

    @invalied_validator
    def load_filament(self, index=0, temperature=210.0,
                      process_callback=None):
        """Load filament"""
        return self._backend.maintain_load_filament(index, temperature,
                                                    process_callback)

    @invalied_validator
    def unload_filament(self, index=0, temperature=210.0,
                        process_callback=None):
        """Unload filament"""
        return self._backend.maintain_unload_filament(index, temperature,
                                                      process_callback)

    @invalied_validator
    def interrupt_load_filament(self):
        """Interrupt load/unload filament"""
        return self._backend.maintain_interrupt_load_filament()

    @invalied_validator
    def set_extruder_temperature(self, index, temperature):
        """Set extruder temperature"""
        return self._backend.maintain_extruder_temperature(index, temperature)

    @invalied_validator
    def update_hbfw(self, stream, size, process_callback=None):
        """Upload and update toolhead firmware"""
        return self._backend.maintain_update_hbfw(stream, size,
                                                  process_callback)


class ScanTasks(SubTasks):
    def _enter_task(self):
        self._backend.begin_scan()

    @invalied_validator
    def step_length(self, length):
        """Set scan step length"""
        return self._backend.scan_step_length(length)

    @invalied_validator
    def forward(self):
        """Let plate go forward 1 step"""
        return self._backend.scan_forward()

    @invalied_validator
    def backward(self):
        """Let plate go backward 1 step"""
        return self._backend.scan_backward()

    @invalied_validator
    def check_camera(self):
        """Check camera status"""
        return self._backend.scan_check_camera()

    @invalied_validator
    def laser(self, left, right):
        """Change scan laser"""
        return self._backend.scan_laser(left, right)

    @invalied_validator
    def calibrate(self):
        """Do a scan calibrate"""
        return self._backend.scan_calibrate()

    @invalied_validator
    def get_calibrate(self):
        """Get scan calibrate values"""
        return self._backend.scan_get_calibrate()

    @invalied_validator
    def oneshot(self):
        """Get a photo from camera"""
        return self._backend.scan_oneshot()

    @invalied_validator
    def scanimages(self):
        """Get a scan image set"""
        return self._backend.scan_images()


class RawTasks(SubTasks):
    def _enter_task(self):
        self.sock = self._backend.begin_raw()

    def quit(self):
        """Quit task"""
        self._backend.quit_raw_mode()
        self.cleanup()
