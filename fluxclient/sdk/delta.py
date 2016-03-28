# -*- coding: utf-8 -*-
"""

.. moduleauthor:: Yen <yen@flux3dp.com>
.. moduleauthor:: John Idle <john@python.invalid>
.. note::

   An example of intersphinx is this: you **cannot** use :mod:`pickle` on this class.


"""
import logging

from fluxclient.hw_profile import HW_PROFILE

from fluxclient.robot.misc import require_robot
from fluxclient.robot import connect_robot
from fluxclient.commands.misc import get_or_create_default_key

logger = logging.getLogger(__name__)


def prepare_robot(endpoint, server_key, client_key):
    def conn_callback(*args):
        sys.stdout.write(".")
        sys.stdout.flush()
        return True
    robot = connect_robot(endpoint, server_key=server_key,
                          client_key=client_key, conn_callback=conn_callback)
    pos = robot.position()
    if pos == "CommandTask":
        robot.begin_scan()
    elif pos == "ScanTask":
        pass
    else:
        raise RuntimeError("Unknow position: %s" % pos)

    return robot


class Delta(object):
    """We use this as a public class example class.

      I don't **know** what's going *on*::

       print(ola)

      hi

    """
    def __init__(self, target):
        super(Delta, self).__init__()

        clientkey = get_or_create_default_key(None)
        endpoint, server_key = require_robot(target, clientkey)
        self.robot = prepare_robot(endpoint, server_key, clientkey)

        self.tool_head_pos = [0, 0, HW_PROFILE["model-1"]["height"]]
        self.motor_pos = {"e0": 0.0, "e1": 0.0, "e2": 0.0}

    def __del__(self):
        pass
        # turn off laser

    def get_position(self):
        """
        Get the current position of tool head

        :return: coordinate of each tower
        :rtype: (float, float, float)

        >>> f.get_position()
        (0, 0, 280)
        """
        return tuple(self.tool_head_pos)

    def home():
        """
        Set each axis back to home.

        :return: the distance each axis goes
        :rtype: (float, float, float)

        """
        pass

    def move(self, x=None, y=None, z=None, speed=None, relative=False, **kargs):
        """
        Move the tool head to (x, y, z)

        :param float x: x coordinate, optional
        :param float y: y coordinate, optional
        :param float z: z coordinate, optional
        :param bool relative: moving relatively or not, default: False

        :param float e0: length mortor e0 move , optional
        :param float e1: length mortor e0 move , optional
        :param float e2: length mortor e0 move , optional


        :return: position after moving command
        :rtype: (float, float, float)

        :raises ValueError: if trying to the exceed the limitation of machine

        * basic usage

        >>> f.get_position()
        (0.0, 0.0, 280.0)
        >>> f.move(50, 40, 30)
        (50.0, 40.0, 30.0)
        >>> f.move(x=80)  # only set the coordinate you want
        (80.0, 40.0, 30.0)
        >>> f.move(x=-10, relative=True)  # using relative flag
        (70.0, 40.0, 30.0)
        >>> f.move(z=-100)
        ValueError: Invalid coordinate

        * control motor during moving

        >>> f.move(50, 40, 30, e0=10)
        (50.0, 40.0, 30.0)

        """
        tmp_pos = self.tool_head_pos[:]
        for name, v, index in [("x", x, 0), ("y", y, 1), ("z", z, 2)]:
            if isinstance(v, (int, float)):
                if relative:
                    tmp_pos[index] += v
                else:
                    tmp_pos[index] = float(v)
            elif v is None:
                pass
            else:
                raise TypeError("unsupported type({1}) for {0} coordinate".format(name, type(v)))
                break
        if tmp_pos[0] ** 2 + tmp_pos[1] ** 2 > HW_PROFILE["model-1"]["radius"] or tmp_pos[2] > HW_PROFILE["model-1"]["height"] or tmp_pos[2] < 0:
            raise ValueError("Invalid coordinate")
        else:
            self.tool_head_pos = tmp_pos

        command = ["X5", "X{}".format(self.tool_head_pos[0]), "Y{}".format(self.tool_head_pos[1]), "Z{}".format(self.tool_head_pos[2])]
        for e, c in [("e0", "E"), ("e1", "S"), ("e2", "U")]:
            if e in kargs:
                if isinstance(kargs[e], (int, float)):
                    command.append("{}{}".format(c, kargs[e]))
                else:
                    raise TypeError("unsupported type({1}) for {0} motor".format(e, type(c)))
                    break
        command = " ".join(command)
        logger.debug(command)
        # robot.makecomamnd.(command)

        return tuple(self.tool_head_pos)

    def get_position_motor(self, motor):
        """
        Get the current position of specified motor

        :param str motor: name of the motor, should be one of the ``"e0", "e1", "e2"``
        :return: the current position of specified motor
        :rtype: float
        :raises ValueError: raise error if using undifine motor name

        >>> f.get_position_motor("e0")
        12.34

        """
        if motor in self.motor_pos:
            return self.motor_pos[motor]
        else:
            raise ValueError("Undifine motor name")

    def move_motor(self, e0=None, e1=None, e2=None):
        """
        Move the motors

        :param float e0: length to move, optional
        :param float e1: length to move, optional
        :param float e2: length to move, optional

        >>> f.move_motor(e0=10)
        >>> f.move_motor(e0=10, e1=10)

        """
        self.move(e0=e0, e1=e1, e2=e2)

    def lock_motor(self, motor):
        """
        Lock the specified motor.

        :param str motor: name of the motor, should be one of the ``"e0", "e1", "e2", "XYZ"``

        >>> f.lock_motor('XYZ')

        """
        # self.robot
        pass

    def release_motor(self):
        """


        Lock the specified motor.


        :param str motor: name of the motor, should be one of the ``"e0", "e1", "e2", "XYZ"``

        .. note::

            If you ever released motor "XYZ", You have to :meth:`home` before calling :meth:`move` them.

        >>> f.release_motor('XYZ')

        """
        if motor == 'XYZ':
            self.loose_flag = True

    def get_position_laser(self, laser):
        """
        Find out whether the laser is on

        :param str laser: Which laser, should be ``"L"`` or ``"R"``
        :raises ValueError: if the laser parameter is not ``"L"`` or ``"R"``

        >>> f.get_position_laser('L')
        True

        """
        if laser == "L":
            pass
        elif laser == "R":
            pass
        else:
            raise ValueError("Wrong laser name")

    def turn_laser(self, laser, on):
        """
        Control the red laser on machine

        :param str laser: Which laser, should be ``"L"`` or ``"R"``
        :param bool on: turn *on* or *off* the laser
        :raises ValueError: if the laser parameter is not ``"L"`` or ``"R"``

        >>> f.turn_laser('L', True)  # Turn on left laser
        True
        >>> f.turn_laser('R', False)  # Turn off right laser
        False

        """

        if laser == "L":
            pass
        elif laser == "R":
            pass
        else:
            raise ValueError("Wrong laser name")
        return bool(on)

    def function(self):
        pass
    # def function(self):
    #     """This function prints hello with a name
    #     function:: format_exception(etype, value, tb[, limit=None])

    #     :param str sender: The person sending the message
    #     :param str recipient: The recipient of the message
    #     :param str message_body: The body of the message
    #     :param priority: The priority of the message, can be a number 1-5
    #     :type priority: integer or None
    #     :return: the message id
    #     :rtype: int
    #     :raises ValueError: if the message_body exceeds 160 characters
    #     :raises TypeError: if the message_body is not a basestring

    #     >>> print_hello_with_name('foo')
    #     Hello, foo

    #     """
    #     pass

if __name__ == '__main__':
    Delta('ffffffffffffffffd81a71e9dcab085c')
