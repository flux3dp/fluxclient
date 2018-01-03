
from functools import wraps
from binascii import b2a_base64
import warnings
import os

from fluxclient.utils import mimetypes
from .backends import InitBackend
from .errors import RobotError, RobotSessionError
from .robot_backend_usb import RobotBackendUSB
from .robot_backend_2 import RobotBackend2, RobotBackend3


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
    @classmethod
    def from_usb(cls, client_key, usbprotocol):
        backend = RobotBackendUSB(usbprotocol)
        return cls("USB", client_key, backend=backend)

    """A `FluxRobot` object represents a live connection with a FLUX device.

    :param tuple endpoint: A tuple contain a pair of IP address and port to \
connect. For example: ("192.168.1.1", 23811)
    :param encrypt.KeyObject client_key: Client identify key
    :param dict device: Device instance to assign value because it may has \
different definition in different version.
    """
    _device = None
    _backend = None
    _config_obj = None
    _locked_obj = None

    def __init__(self, endpoint, client_key, device=None,
                 ignore_key_validation=False, backend=None):
        self._device = device
        self._client_key = client_key

        if backend:
            self._backend = backend
        else:
            init_backend = InitBackend(endpoint)
            proto_ver = init_backend.do_handshake()

            if proto_ver == 2:
                self._backend = RobotBackend2(init_backend.sock, client_key,
                                              device, ignore_key_validation)
            elif proto_ver == 3:
                self._backend = RobotBackend3(init_backend.sock, client_key,
                                              device, ignore_key_validation)
            else:
                raise RobotSessionError("Protocol not support")

    @property
    def device(self):
        return self._device

    @blocked_validator
    def list_files(self, path):
        """List files on the device.

    :param str path: Path on device, must start with /SD/ or /USB/
    :return: An array of files and directories. Each array element is a \
tuple, first is a boolean present this name is a folder or not and second is \
the name of the file or folder.
    :rtype: list"""
        return self._backend.list_files(path)

    @blocked_validator
    def file_info(self, path):
        """Gets fcode information from specific file path.

    :param str path: File path on the device
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
        """Gets file md5 on the device.

    :param str path: File path on the device
    :return: MD5 hex
    :rtype: str"""
        return self._backend.file_md5(path)

    @blocked_validator
    def mkdir(self, path):
        """Creates a folder on the device storage.

    :param str path: Path to be created"""
        return self._backend.mkdir(path)

    @blocked_validator
    def rmdir(self, path):
        """Removes folder on the device storage.

    :param str path: Path to be removed"""
        return self._backend.rmdir(path)

    @blocked_validator
    def cpfile(self, source_path, dist_path):
        """Copies a file on the device.

    :param str source_path: File can be on device or usb disk
    :param str dist_path: File can be copied only to the device"""
        return self._backend.cpfile(source_path, dist_path)

    @blocked_validator
    def rmfile(self, path):
        """Removes a file on the device.

    :param str path: File to be deleted"""
        return self._backend.rmfile(path)

    @blocked_validator
    def download_file(self, path, stream, process_callback=None):
        """Downloads a file from the device.

    :param str path: File on device for download
    :param filelike_obj stream: A local file-like object to write
    :param function process_callback: A callable object which will be invoked \
during download progress"""
        return self._backend.download_file(path, stream, process_callback)

    @blocked_validator
    def upload_file(self, filename, upload_to="#", process_callback=None):
        """Uploads a file to the device.

    :param str filename: File to upload to device
    :param str upload_to: Path on device to upload to
    :param function process_callback: A callable object which will be invoked \
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
        """Uploads a file to the device. ( In file stream )

    :param file stream: Filelike object
    :param str mimetype: Contents mimetype
    :param int size: Contents size
    :param str upload_to: Path on device to upload to
    :param function callback: A callable object which will be invoke during \
upload progress"""
        return self._backend.upload_stream(
            self, stream, mimetype, size, upload_to,
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
        """Selects a file to run.

    :param str path: Path on device"""
        return self._backend.select_file(path)

    @blocked_validator
    def start_play(self):
        """Starts currently selected task."""
        return self._backend.start_play()

    @blocked_validator
    def pause_play(self):
        """Pauses current task."""
        return self._backend.pause_play()

    @blocked_validator
    def abort_play(self):
        """Aborts current task."""
        return self._backend.abort_play()

    @blocked_validator
    def resume_play(self):
        """Resumes current task."""
        return self._backend.resume_play()

    @blocked_validator
    def report_play(self):
        """Reports current task status."""
        return self._backend.report_play()

    @blocked_validator
    def play_info(self):
        """Shows current task info."""
        return self._backend.play_info()

    @blocked_validator
    def set_toolhead_operating_in_play(self):
        return self._backend.set_toolhead_operating_in_play()

    @blocked_validator
    def set_toolhead_standby_in_play(self):
        return self._backend.set_toolhead_standby_in_play()

    @blocked_validator
    def set_toolhead_heater_in_play(self, temp, index=0):
        return self._backend.set_toolhead_heater_in_play(temp, index)

    @blocked_validator
    def load_filament_in_play(self, index):
        return self._backend.load_filament_in_play(index)

    @blocked_validator
    def unload_filament_in_play(self, index):
        return self._backend.unload_filament_in_play(index)

    @blocked_validator
    def press_button_in_play(self):
        return self._backend.press_button_in_play()

    @blocked_validator
    def restart_play(self):
        return self._backend.restart_play()

    @blocked_validator
    def quit_play(self):
        """Quits from current task status."""
        return self._backend.quit_play()

    @property
    def deviceinfo(self):
        """Gets the device information"""
        return self._backend.deviceinfo()

    @property
    def config(self):
        """Gets a subscriptable device config object."""
        if not self._config_obj:
            self._config_obj = RobotConfigure(self)
        return self._config_obj

    @blocked_validator
    def fetch_log(self, path, stream, process_callback=None):
        """Fetch log file from the device.

    :param str path: File on device for download
    :param filelike_obj stream: A local file-like object to write
    :param function process_callback: A callable object which will be invoked \
during download progress"""
        return self._backend.fetch_log(path, stream, process_callback)

    @blocked_validator
    def get_cloud_validation_code(self):
        """Return a tuple for cloud device relation validation."""
        token, code = self._backend.get_cloud_validation_code()
        access_id = self._client_key.get_access_id()
        signature = self._client_key.sign(code)
        return {
            "token": token, "access_id": access_id,
            "signature": b2a_base64(signature).decode("ascii")
        }

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
        """Kicks a work session which occupied the device."""
        return self._backend.kick()

    @blocked_validator
    def update_firmware(self, stream, size, process_callback=None):
        """Uploadshen and t updates a flux device firmware"""
        return self._backend.update_firmware(self, stream, size,
                                             process_callback)

    @blocked_validator
    def scan(self):
        """Begins a scanning task and return a scan object"""
        return ScanTasks(self)

    @blocked_validator
    def maintain(self):
        """Begins a maintaining task and return a scan object"""
        return MaintainTasks(self)

    @blocked_validator
    def icontrol(self):
        def exit_callback():
            self._locked_obj = None

        sock = self._backend.begin_icontrol()
        self._locked_obj = "iControl"
        from fluxclient.sdk.delta import Delta
        return Delta(sock, self._backend.sock.client_key, exit_callback)

    @blocked_validator
    def raw(self):
        return RawTasks(self)

    def close(self):
        """Closes a device connection"""
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
        """Quits current task"""
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
        """Moves to home position."""
        return self._backend.maintain_home()

    @invalied_validator
    def reset_hardware(self):
        self._backend.maintain_reset_atmel()
        self.cleanup()

    @invalied_validator
    def calibrate(self, threshold=None, clean=False, process_callback=None):
        """Does a calibration testing"""
        return self._backend.maintain_calibration(self, threshold, clean,
                                                  process_callback)

    @invalied_validator
    def calibration(self, threshold=None, clean=False, process_callback=None):
        warnings.warn("Use 'calibrate' method instead", DeprecationWarning)
        return self.calibrate(threshold=threshold, clean=clean,
                              process_callback=process_callback)

    @invalied_validator
    def zprobe(self, process_callback=None):
        """Does a zprobe testing"""
        return self._backend.maintain_zprobe(self, process_callback)

    @invalied_validator
    def manual_level(self, h):
        """Sets heigh level"""
        return self._backend.maintain_manual_level(h)

    @invalied_validator
    def head_info(self):
        """Gets the toolhead info"""
        return self._backend.maintain_head_info()

    @invalied_validator
    def head_status(self):
        """Gets the toolhead status"""
        return self._backend.maintain_head_status()

    @invalied_validator
    def set_heater(self, index, temperature):
        """Set extruder temperature"""
        return self._backend.maintain_set_heater(index, temperature)

    @invalied_validator
    def diagnosis_sensor(self):
        return self._backend.maintain_diagnosis_sensor()

    @invalied_validator
    def diagnosis(self, option):
        return self._backend.maintain_diagnosis(option)

    @invalied_validator
    def move(self, *ignore, **commands):
        return self._backend.maintain_move(**commands)

    @invalied_validator
    def calibrate_beambox_camera(self):
        return self._backend.calibrate_beambox_camera()

    @invalied_validator
    def load_filament(self, index=0, temperature=210.0,
                      process_callback=None):
        """Loads the filament"""
        return self._backend.maintain_load_filament(self, index, temperature,
                                                    process_callback)

    @invalied_validator
    def load_flexible_filament(self, index=0, temperature=210.0,
                               process_callback=None):
        """Loads the filament"""
        return self._backend.maintain_load_flexible_filament(
            self, index, temperature, process_callback)

    @invalied_validator
    def unload_filament(self, index=0, temperature=210.0,
                        process_callback=None):
        """Unloads the filament"""
        return self._backend.maintain_unload_filament(self, index, temperature,
                                                      process_callback)

    @invalied_validator
    def interrupt_load_filament(self):
        """Interrupt load/unload filament"""
        return self._backend.maintain_interrupt_load_filament()

    @invalied_validator
    def set_extruder_temperature(self, index, temperature):
        """Sets nozzel temperature"""
        return self._backend.maintain_extruder_temperature(index, temperature)

    @invalied_validator
    def update_hbfw(self, stream, size, process_callback=None):
        """Uploads and then updates the toolhead firmware"""
        return self._backend.maintain_update_hbfw(self, stream, size,
                                                  process_callback)


class ScanTasks(SubTasks):
    def _enter_task(self):
        self._backend.begin_scan()

    @invalied_validator
    def step_length(self, length):
        """Sets scanning steps length"""
        return self._backend.scan_step_length(length)

    @invalied_validator
    def forward(self):
        """Spins the plate to go forward 1 step"""
        return self._backend.scan_forward()

    @invalied_validator
    def backward(self):
        """Spins the plate to go backward 1 step"""
        return self._backend.scan_backward()

    @invalied_validator
    def check_camera(self):
        """Check the camera status"""
        return self._backend.scan_check_camera()

    @invalied_validator
    def laser(self, left, right):
        """Change scanning laser status"""
        return self._backend.scan_laser(left, right)

    @invalied_validator
    def calibrate(self):
        """Initiate a scanning calibration"""
        return self._backend.scan_calibrate()

    @invalied_validator
    def get_calibrate(self):
        """Gets the calibration value for scanning"""
        return self._backend.scan_get_calibrate()

    @invalied_validator
    def oneshot(self):
        """Gets a photo from camera"""
        return self._backend.scan_oneshot()

    @invalied_validator
    def scanimages(self):
        """Gets a scan image set"""
        return self._backend.scan_images()


class RawTasks(SubTasks):
    def _enter_task(self):
        self.sock = self._backend.begin_raw()

    def quit(self):
        """Quits current task"""
        self._backend.quit_raw_mode()
        self.cleanup()
