# -*- coding: utf-8 -*-
"""

.. moduleauthor:: Yen Feng <yen@flux3dp.com>

"""
from io import BytesIO
import logging
from uuid import UUID
import socket
from time import sleep, time
import threading

from msgpack import packb, unpackb, Unpacker
from PIL import Image

from fluxclient.hw_profile import HW_PROFILE
from fluxclient.encryptor import KeyObject
from fluxclient.device.manager import DeviceManager
from fluxclient.commands.misc import get_or_create_default_key
from fluxclient.robot import FluxRobot, FluxCamera
from fluxclient.sdk import *

logger = logging.getLogger(__name__)


def type_check(instance, type_candidates, err_msg=''):
    if isinstance(instance, type_candidates):
        return
    else:
        raise TypeError('{} type: {} is not supported'.format(err_msg, type(instance)))


class Delta(object):
    """
      Delta is an instance that presents the device in SDK mode, which allows users to control motions of the machine freely.
    """
    def __init__(self, wrapped_socket, client_key, exit_callback=None, blocking=True):
        """
            Creates a Delta ( SDK task ) instance

            :param sockets wrapped_socket: Wrapped sockets
            :param callback exit_callback: Callback when exited
            :param bool blocking: Blocking means python will only send commands when prior commands are finished, otherwise the device will buffer commands.
        """
        super(Delta, self).__init__()

        self._command_index = -1
        self.tool_head_pos = [0, 0, HW_PROFILE["model-1"]["height"]]
        self.motor_pos = {"E1": 0.0, "E2": 0.0, "E3": 0.0}
        self.laser_status = {"L": False, "R": False}
        # [("E1", "E"), ("E2", "S"), ("E3", "U")]
        self.motor_status = {"XYZ": True, "E1": True, "E2": True, "E3": False}
        self.loose_flag = True
        self.headerror_callback = None
        self._camera = None
        self.client_key = client_key
        self.lock = threading.Lock()
        self.command_output = []  # record all the output return from command

        self.control_sock = wrapped_socket
        self.exit_callback = exit_callback
        self.blocking_flag = blocking

        self.head_status = [b'', -2, 0, {}]
        self.serial_status = [b'', 0, 0]
        self.serial_out = []
        self.open_udp_sock()

        self.connected = True

    def __del__(self):
        if self.connected:
            logger.debug('wait for commands to finished')
            while self.atomic_status()[0] - 1 != self._command_index or self.atomic_status()[1] != 0:
                # print(self.atomic_status())
                pass
            logger.debug('wait for udp socket to close')
            self.killthread = True
            self.queue_checker.join()
            sleep(1)

            self._command_index += 1
            self.send_command([CMD_QUIT])
            if self.exit_callback:
                self.exit_callback()
            sleep(1)
            self.connected = False

    @property
    def head_type(self):
        return self.head_status[3].get(b'module', b'N/A')

    @property
    def camera(self):
        if self._camera:
            return self._camera
        else:
            print(self.control_sock.getpeername()[0])
            camera = FluxCamera((self.control_sock.getpeername()[0], 23812), self.client_key)
            self._camera = camera
            return self._camera

    @classmethod
    def connect_delta(cls, target=None, ip=None, client_key=None, password=None, kick=False, blocking=True):
        """
            Creates a Delta instance

            :param str uuid: Device's UUID, optional
            :param str ip: Device's IP, optional
            :param str password: Device's password
            :param bool kick: Force to kick other users
            :param bool blocking: Blocking means python will only send commands when prior commands are finished.
        """
        if client_key:
            if type(client_key) == str:
                client_key = get_or_create_default_key(client_key)
            else:
                type_check(client_key, (str, KeyObject), 'client_key')
        else:
            client_key = get_or_create_default_key()

        if target:
            type_check(target, str, 'target')
            target = UUID(target)

        if ip:
            type_check(ip, str, 'ip')

        if password:
            type_check(password, str, 'password')

        if target:
            manager = DeviceManager.from_uuid(client_key, target)
        elif ip:
            manager = DeviceManager.from_ipaddr(client_key, ip)

        if not manager.authorized:
            manager.authorize_with_password(password)
            if manager.authorized:
                manager.add_trust('sdk key', client_key.public_key_pem.decode())
                logger.warning('[Warning]: adding new key into flux delta')
            else:
                raise RuntimeError("Authorize error")

        if manager.authorized:
            robot = FluxRobot((ip, 23811), client_key)

            st_id = robot.report_play()["st_id"]
            if st_id > 0:
                if st_id not in [64, 66, 128, 130]:
                    robot.abort_play()

                while robot.report_play()["st_id"] in [66, 130]:
                    sleep(0.5)

                robot.quit_play()
            elif st_id < 0:
                robot.kick()

            while robot.report_play()["st_id"] != 0:
                sleep(0.5)
            m_delta = robot.icontrol()  # retuen a Delta object
            m_delta.blocking_flag = blocking
            return m_delta
        else:
            raise RuntimeError("cannot connect to flux delta")

    def open_udp_sock(self):
        local_ipaddr = self.control_sock.getsockname()
        self.send_command([CMD_SYNC, local_ipaddr[0], 22111, b""])
        # init udp socket: for state
        self.status = (1, 0)  # index, queue_left
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(('', 22111))
        buf = self.udp_sock.recv(4096)
        self.killthread = False
        self.queue_checker = threading.Thread(target=self.delta_status)
        self.queue_checker.start()

    def atomic_status(self, new_staus=None):
        """
        Returns the index, queue_left of commands buffer
        """
        self.lock.acquire()

        if new_staus:
            self.status = new_staus
        else:
            new_staus = self.status
        self.lock.release()
        return new_staus

    def get_result(self, index, wait=True):
        """
        Acquires the result of certain index from Delta
        """
        self.lock.acquire()
        if len(self.command_output) < index:
            raise SDKFatalError(self, 'Fatal Error: command index error(index:{}, total:{})'.format(index, len(self.command_output)))
        else:
            if callable(self.command_output[index]):
                if wait:
                    self.lock.release()
                    while callable(self.command_output[index]):
                        # busy waiting for return value
                        pass
                    self.lock.acquire()
                    ret = self.command_output[index]
                else:
                    raise SDKFatalError(self, 'Not ready(index:{})'.format(index))
            elif self.command_output[index] is False:
                if wait:
                    self.lock.release()
                    while self.command_output[index] is False:
                        # print(index, self.command_output)
                        # sleep(0.5)
                        # busy waiting for return value
                        pass
                    self.lock.acquire()
                    ret = self.command_output[index]
                else:
                    raise SDKFatalError(self, 'Not ready(index:{})'.format(index))
            else:
                ret = self.command_output[index]
        self.lock.release()
        return ret

    def delta_status(self):
        """
        Loop thread function that collects and updates delta's status and command's output
        """
        recv_until = -1
        unpacker = Unpacker()
        while True:
            if self.killthread:
                self.udp_sock.close()
                return
            else:
                buf = self.udp_sock.recv(4096)
                payload = unpackb(buf)
                # print(payload)
                if payload[0] == 0:
                    if payload[2] - 1 - payload[3] > recv_until:  # finish_index = next_index - 1 - command_in_queue
                        for i in range(recv_until + 1, payload[2] - 1 - payload[3] + 1):
                            if self.command_output[i]:
                                ret = self.control_sock.recv(4096)
                                unpacker.feed(ret)
                                ret = unpacker.__next__()
                                self.lock.acquire()
                                self.command_output[i] = self.command_output[i](ret)
                                self.lock.release()
                            elif self.command_output[i] is False:
                                self.lock.acquire()
                                self.command_output[i] = None
                                self.lock.release()
                            else:
                                print('         --->>>no way!!', self.command_output[i], self.command_output)

                        recv_until = payload[2] - 1 - payload[3]

                    self.atomic_status((payload[2], payload[3]))
                elif payload[0] == 1:
                    # print(payload)
                    self.lock.acquire()
                    self.head_status = payload[1:]
                    self.head_error = payload[3]
                    if self.head_error > 0:
                        if self.headerror_callback:
                            self.headerror_callback(self.head_error)
                        self.send_command([CMD_CLHE])
                    self.lock.release()
                elif payload[0] == 2:
                    self.lock.acquire()
                    self.serial_status = payload[1:]
                    self.lock.release()

    def send_command(self, command, recv_callback=False):
        self._command_index += 1
        self.control_sock.send(packb(tuple([self._command_index] + command)))

        if recv_callback:
            self.command_output.append(recv_callback)
        elif recv_callback is False:
            self.command_output.append(False)
        else:
            print('no way2!')
        # print('send', self.command_output)
        # print('index', self._command_index)
        return self._command_index

    def get_position(self):
        """
        Gets the current position of toolhead

        :return: coordinate of each tower
        :rtype: (float, float, float)

        >>> f.get_position()
        (0, 0, 280)
        """
        return tuple(self.tool_head_pos)

    def home(self):
        """
        Sets each axis back to home.

        :return: command index and the coordinate where the toolhead originally is
        :rtype: (int, (float, float, float))

        >>> f.home()
        (0, (20.0, 20.0, 20.0))
        >>> f.get_position()
        (0, 0, 280)
        """
        def post_process(ret):
            if ret[0] != CMD_G028 or ret[1] != 0:
                print('ret:', ret)
                raise RuntimeError('command retrun error')
            else:
                return tuple(ret[2])

        self.loose_flag = False
        command_index = self.send_command([CMD_G028], recv_callback=post_process)
        ret = self.get_result(command_index, wait=True)
        if self.blocking_flag:
            return command_index, ret
        else:
            return command_index, None

    def move(self, x=None, y=None, z=None, speed=None, relative=False, **kargs):
        """
        Move the toolhead to (x, y, z)

        :param float x: x coordinate, optional
        :param float y: y coordinate, optional
        :param float z: z coordinate, optional

        :param int speed: moving speed, should be within [20, 8000], optional

        :param bool relative: moving relatively or not, default: False

        :param float E1: position mortor E1 move , optional
        :param float E2: position mortor E2 move , optional
        :param float E3: position mortor E3 move , optional


        :return: position after moving command
        :rtype: (float, float, float)

        :raises ValueError: if trying to the exceed the limitation of machine

        * basic usage

        >>> f.get_position()
        (0.0, 0.0, 280.0)
        >>> f.move(50, 40, 30)
        (50.0, 40.0, 30.0)
        >>> f.move(x=80)  # only sets the coordinate you want
        (80.0, 40.0, 30.0)
        >>> f.move(x=-10, relative=True)  # using relative flag
        (70.0, 40.0, 30.0)
        >>> f.move(z=-100)
        ValueError: Invalid coordinate

        * control motor during moving

        >>> f.move(50, 40, 30, E1=10)
        (50.0, 40.0, 30.0)

        """
        if [x, y, z].count(None) != 3:  # whether move the tool head
            if self.loose_flag:
                raise SDKFatalError(self, 'motor need to home() before moving')
            else:
                tmp_pos = self.tool_head_pos[:]
                for name, v, index in [["X", x, 0], ["Y", y, 1], ["Z", z, 2]]:
                    if isinstance(v, (int, float)):
                        v = float(v)
                        if relative:
                            tmp_pos[index] += v
                        else:
                            tmp_pos[index] = v
                    elif v is None:
                        pass
                    else:
                        raise SDKFatalError(self, "unsupported type({1}) for {0} coordinate".format(name, type(v)))
                        break

                # range check
                if tmp_pos[0] ** 2 + tmp_pos[1] ** 2 > HW_PROFILE["model-1"]["radius"] ** 2 or tmp_pos[2] > HW_PROFILE["model-1"]["height"] or tmp_pos[2] < 0:
                    raise SDKFatalError(self, "Invalid coordinate")
                else:
                    for index, v in enumerate([x, y, z]):
                        if v:
                            self.tool_head_pos[index] = tmp_pos[index]
        # speed
        if isinstance(speed, (int, float)):
            speed = int(speed)
            if speed <= 20 or speed >= 8000:  # TODO
                raise SDKFatalError(self, "move speed:({}) out of range [20, 8000]".format(speed))
        elif speed is None:
            pass
        else:
            raise SDKFatalError(self, "unsupported type({}) for speed".format(type(speed)))

        # dealing with E command
        c = 0
        for name in ['E1', 'E2', 'E3']:
            if name in kargs:
                if isinstance(kargs[name], (int, float)):
                    kargs[name] = float(kargs[name])
                    c += 1
                    if relative:
                        self.motor_pos[name] += kargs[name]
                    else:
                        self.motor_pos[name] = kargs[name]
                else:
                    raise SDKFatalError(self, "unsupported type({1}) for {0} coordinate".format(name, type(kargs[name])))
        if c > 1:
            raise SDKFatalError(self, "too many E command at once")

        # form the dict passing to delta
        command_dict = {}  # {F:int, X:float, Y:float, Z:float, E2:float, E3:float, E3:float}
        for name, value, index in [["X", x, 0], ["Y", y, 1], ["Z", z, 2]]:
            if value:
                command_dict[name] = self.tool_head_pos[index]
        if speed:
            command_dict['F'] = speed

        for name in ['E1', 'E2', 'E3']:
            if name in kargs:
                command_dict[name] = self.motor_pos[name]
        # print(command_dict)

        if command_dict:
            command_index = self.send_command([CMD_G001, command_dict], recv_callback=False)

            # blocking or not
            if self.blocking_flag:
                return command_index, self.get_result(command_index, wait=True)
            else:
                return command_index, None

        return tuple(self.tool_head_pos)

    def get_position_motor(self, motor):
        """
        Gets the current position of specified motor

        :param str motor: name of the motor, should be one of the ``"E1", "E2", "E3"``
        :return: the current position of specified motor
        :rtype: float
        :raises ValueError: raise error if using undifine motor name

        >>> f.get_position_motor("E1")
        12.34

        """
        if motor in self.motor_pos:
            return self.motor_pos[motor]
        else:
            raise SDKFatalError(self, "Invalid motor name {}".format(motor))

    def move_motor(self, E1=None, E2=None, E3=None, speed=None):
        """
        Moves stepper motors

        :param float E1: position to move, optional
        :param float E2: position to move, optional
        :param float E3: position to move, optional
        :param int speed: moving speed, should be within [20, 8000], optional

        >>> f.move_motor(E1=10)
        >>> f.move_motor(E1=10, E2=10)

        """
        motors = {}
        for E, name in [(E1, "E1"), (E2, "E2"), (E3, "E3")]:
            if E:
                motor[name] = E
        if speed:
            motors['speed'] = speed

        self.move(**motors)

    def lock_motor(self):
        """
        Locks all motor.

        >>> f.lock_motor('XYZ')

        """
        command_index = self.send_command([CMD_M017], recv_callback=False)
        if self.blocking_flag:
            return command_index, self.get_result(command_index, wait=True)
        else:
            return command_index, None

    def release_motor(self):
        """
        Releases all motor.

        .. note::

            If you ever released stepper motors, You have to :meth:`home` before calling :meth:`move` them.

        >>> f.release_motor()

        """
        # if motor == 'XYZ':
        self.loose_flag = True
        command_index = self.send_command([CMD_M084], recv_callback=False)
        if self.blocking_flag:
            return command_index, self.get_result(command_index, wait=True)
        else:
            return command_index, None

    def get_position_laser(self, laser):
        """
        Finds out whether the laser is on

        :param str laser: Which laser, should be ``"L"`` or ``"R"``
        :raises ValueError: if the laser parameter is not ``"L"`` or ``"R"``

        >>> f.get_position_laser('L')
        True

        """

        if laser == "L" or laser == "R":
            return self.laser_status[laser]
        else:
            if isinstance(laser, str):
                raise SDKFatalError(self, "Invalid laser name: {}".format(laser))
            else:
                raise SDKFatalError(self, "Invalid argument type")

    def turn_laser(self, laser, on):
        """
        Controls the scanning laser

        :param str laser: Which laser, should be ``"L"`` or ``"R"``
        :param bool on: turn *on* or *off* the laser
        :raises ValueError: if the laser parameter is not ``"L"`` or ``"R"``

        >>> f.turn_laser('L', True)  # Turn on left laser
        True
        >>> f.turn_laser('R', False)  # Turn off right laser
        False

        """

        if laser == "L" or laser == "R":
            self.laser_status[laser] = bool(on)

            flag = 0
            if self.laser_status['L']:
                flag |= 1 << 0
            if self.laser_status['R']:
                flag |= 1 << 1

            command_index = self.send_command([CMD_SLSR, flag], recv_callback=False)
            if self.blocking_flag:
                return command_index, self.get_result(command_index, wait=True)
            else:
                return command_index, None

        elif isinstance(laser, str):
            raise SDKFatalError(self, "Invalid laser name: {}".format(laser))
        else:
            raise SDKFatalError(self, "Invalid argument type")

        return self.laser_status[laser]

    def get_image(self):
        """
        Takes a picture with Delta's built in camera

        :return: a image object
        :rtype: PIL.Image.Image

        """
        storage = {}

        def callback(c, img):
            storage["img"] = img

        self.camera.require_frame()
        while not storage:
            self.camera.feed(callback)

        return Image.open(BytesIO(storage['img']))

    def serial_write(self, buf):
        """
        Writes buffer or string to extension port

        :param str/bytes buf: data sent in extension port
        :rtype: (int, None): (command_index, None)

        >>> f.serial_write(b'PING')  # write bytes
        (1, None)
        >>> f.serial_write('Hello')  # write string
        (2, None)

        """
        if not isinstance(buf, (str, bytes)):
            raise SDKFatalError(self, "Invalid buffer type: {}".format(type(buf)))

        command_index = self.send_command([CMD_THRC, buf], False)
        if self.blocking_flag:
            return command_index, self.get_result(command_index, wait=True)
        else:
            return command_index, None

    def atomic_serial_list(self, param='', mode='e'):
        ret = None
        if mode == 'e':
            self.serial_out.extend(param[1])
        elif mode == 'l':
            self.lock.acquire()
            ret = len(self.serial_out)
            self.lock.release()
        elif mode == 'p':
            self.lock.acquire()
            ret = self.serial_out.pop(0)
            self.lock.release()
        return ret

    def serial_read(self, timeout=0):
        """
        Reads a chunck from extension port
        Returns b'' if timeout happened

        :param int timeout: timeout trying to read data, default 0
        :rtype: bytes: a chunck read from extension port

        >>> delta.serial_read(timeout=10)
        b'PONG'
        """

        if not isinstance(timeout, (int, float)):
            raise SDKFatalError(self, "Wrong timeout type: {}".format(type(timeout)))

        t_s = time()

        if self.atomic_serial_list(mode='l'):
            return self.atomic_serial_list(mode='p')
        else:
            flag = False
            if self.serial_status[2] != 0:
                flag = True
            else:
                while time() - t_s < timeout:
                    if self.serial_status[2] != 0:
                        flag = True
                        break
            if flag:
                command_index = self.send_command([CMD_THRR], self.atomic_serial_list)
                if self.blocking_flag:
                    while self.atomic_serial_list(mode='l') == 0:
                        if time() - t_s > timeout:
                            return b""

                    return self.atomic_serial_list(mode='p')
            else:
                return b""

    def disable_motor(self, motor):
        """
        Enables motor control

        :param str motor: Which motor to disable, should be one of ``"XYZ"``, ``"E1"``, ``"E2"``, ``"E3"``
        :raises TypeError: motor is not str
        :raises ValueError: if the motor's name is not one of ``"XYZ"``, ``"E1"``, ``"E2"``, ``"E3"``

        """
        if motor in ["XYZ", "E1", "E2", "E3"]:
            self.motor_status[motor] = False
        elif isinstance(motor, str):
            raise SDKFatalError(self, "Wrong motor name: {}".format(motor))
        else:
            raise SDKFatalError(self, "Wrong motor type: {}".format(type(motor)))

    def enable_motor(self, motor):
        """
        Disables a motor

        :param str motor: Which motor to enable, should be one of ``"XYZ"``, ``"E1"``, ``"E2"``, ``"E3"``
        :raises TypeError: motor is not str
        :raises ValueError: if the motor's name is not one of ``"XYZ"``, ``"E1"``, ``"E2"``, ``"E3"``

        """
        if motor in ["XYZ", "E1", "E2", "E3"]:
            self.motor_status[motor] = True
        elif isinstance(motor, str):
            raise SDKFatalError(self, "Wrong motor name: {}".format(motor))
        else:
            raise SDKFatalError(self, "Wrong motor type: {}".format(type(motor)))

    def get_head_profile(self):
        """
        Gets the basic toolhead info(immutable data).

        :return: command index and dict consist of toolhead's basic information
        :rtype: (int, dict)

        >>> f.get_head_profile()  # no tool head connected
        (0, {"module": "N/A"})

        >>> f.get_head_profile()  # print head
        (0, {"version": "1.0.8", "module": "EXTRUDER", "id": "203236325346430100240001", "vendor": "FLUX .inc"})

        >>> f.get_head_profile()  # laser head
        (0, {"version": "1.0.3", "module": "LASER", "id":"203236325346430100260004", "vendor": "FLUX .inc"})


        .. note::

            For current toolhead status use :meth:`get_head_status`

        """
        def post_process(ret):
            return {i.decode(): ret[1][i].decode() for i in ret[1]}

        self.send_command([CMD_THPF])
        command_index = self.send_command([CMD_THPF], recv_callback=post_process)
        ret = self.get_result(command_index, wait=True)
        if self.blocking_flag:
            return command_index, ret
        else:
            return command_index, None

    def get_head_status(self):
        """
        Gets the current toolhead status(mutable data).

        :return: dict consist of toolhead's currrent information
        :rtype: dict

        >>> flux.get_head_status()
        {"real temp": [24.8], "target fan": [0.0], "target temp":[nan]}

        .. note::

            For basic toolhead information use :meth:`get_head_profile`

        """
        ret = {}
        for i in self.head_status[3]:
            if type(self.head_status[3][i]) == bytes:
                ret[i.decode()] = self.head_status[3][i].decode()
            else:
                ret[i.decode()] = self.head_status[3][i]
        if self.head_type == b"EXTRUDER":
            ret["real temp"] = ret.pop('rt')
            ret["target temp"] = ret.pop('tt')
            ret["target fan"] = ret.pop('tf')
        return ret

    def set_temp(self, temp, index=0):
        """
        Sets the temperature of print head

        :param number power: The power of print head, should be within [0.0, 200.0]
        :raises TypeError: if temp is not a number
        :raises ValueError: if the temp is not in [0.0, 200.0]
        :raises RuntimeError: if using wrong type of toolhead

        >>> f.set_temp(200)
        """
        if self.head_type == b"EXTRUDER":
            if isinstance(temp, (int, float)):
                if temp <= 200.0 and temp >= 0.0:
                    self.send_command([CMD_M104, index, int(temp)])
                else:
                    raise SDKFatalError(self, "Invalid temperature")
            else:
                raise SDKFatalError(self, "unsupported temp type: {}".format(type(temp)))
        else:
            raise SDKFatalError(self, 'Head error: {}, require head: "EXTRUDER"'.format(self.head_type))

    def set_fan(self, speed, toolhead_index=0):
        """
        Sets the speed of toolhead's fan

        :param number speed: The power of laser head, should be within [0.0, 1.0]
        :raises TypeError: if speed is not a number
        :raises ValueError: if the power is not in [0.0, 1.0]
        :raises RuntimeError: if using wrong type of toolhead

        >>> f.set_fan(0.5)
        """
        if self.head_type == b"EXTRUDER":
            if isinstance(speed, (int, float)):
                speed = float(speed)
                if speed <= 1.0 and speed >= 0.0:
                    print('speed', speed)
                    self.send_command([CMD_M106, toolhead_index, speed])
                else:
                    raise SDKFatalError(self, "Invalid fan speed:{}, should be in [0.0, 1.0]".format(speed))
            else:
                raise SDKFatalError(self, "unsupported speed type: {}".format(type(speed)))
        else:
            raise SDKFatalError(self, 'Head error: {}, require head: "EXTRUDER"'.format(self.head_type.decode()))

    def set_power(self, power):
        """
        Sets the power of laser toolhead

        :param number power: The power of laser head, should be within [0.0, 1.0]
        :raises TypeError: if power is not a number
        :raises ValueError: if the power is not in [0.0, 1.0]
        :raises RuntimeError: if using wrong type of toolhead

        >>> f.set_power(0.5)
        """
        if self.head_type == b"LASER":
            if isinstance(power, (int, float)):
                if power <= 1.0 and power >= 0.0:
                    print('pow', power)
                    self.send_command([CMD_HLSR, power])
                else:
                    raise SDKFatalError(self, "Invalid laser power, should be within [0.0, 1.0]")
            else:
                raise SDKFatalError(self, "unsupported power type: {}".format(type(power)))
        else:
            ret = self.set_head("LASER")
            if not ret:
                raise SDKFatalError(self, 'Head error: {}, require head: "LASER"'.format(self.head_type))

    def set_head(self, head_type):
        """
        Sets the tool head want to use
        :param str head_type: head_type, should be one of 'EXTRUDER', 'LASER', 'N/A'
        """
        if head_type in ['EXTRUDER', 'LASER', 'N/A'] or head_type.startswith('USER'):
            self.send_command([CMD_REQH, head_type], False)
            self.send_command([CMD_BSTH])
            time_s = time()
            while True:  # wait for head to be ready
                if self.head_status[2] == 0:
                    return True
                elif time() - time_s > 5:  # not ready for too long
                    return False
                else:
                    sleep(0.5)
                    continue
        else:
            raise SDKFatalError(self, "Head error: {}".format(self.head_type))

    def set_headerror_callback(self, callback_function):
        """
        Sets the callback function handling tool head error
        :param funciton callback_function: a callable function that take head error code as parameter
        """
        if callable(callback_function):
            self.headerror_callback = callback_function
        else:
            raise SDKFatalError(self, "Callback error: should be callable object")

    def get_fsr(self):
        """
        Gets the current force sensor reading.

        :return: command index and dict consist of each axis' currrent reading
        :rtype: (int, dict)

        >>> flux.get_fsr()
        (0, {'X': 3889.15, 'Y': 3958.45, 'Z': 3715.9})

        """
        def post_process(ret):
            return {i.decode(): ret[1][i] for i in ret[1]}
        command_index = self.send_command([CMD_VALU, 1], post_process)
        if self.blocking_flag:
            return command_index, self.get_result(command_index, wait=True)
        else:
            return command_index, None

    def get_value(self):
        """
        Gets some other sensor's reading.

        :return: command index and dict consist of whether switch is triggered
        :rtype: (int, dict)

        F0: filament sensor 0

        F1: filament sensor 1

        MB: Mainboard Button

        >>> flux.get_value()
        (0, {'F0': True, 'MB': False, 'F1': True})

        """

        def post_process(ret):
            return {i.decode(): ret[1][i] for i in ret[1]}
        command_index = self.send_command([CMD_VALU, 2 | 4 | 8], post_process)
        if self.blocking_flag:
            return command_index, self.get_result(command_index, wait=True)
        else:
            return command_index, None

    def close(self):
        """
        Disconnects from delta
        """
        self.__del__()


class SDKFatalError(Exception):
    def __init__(self, delta, err_msg):
        delta.close()
        self.err_msg = err_msg

    def __repr__(self):
        return self.err_msg

    def __str__(self):
        return self.err_msg
