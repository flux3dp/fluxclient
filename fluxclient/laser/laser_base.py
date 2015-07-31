# !/usr/bin/env python3

import os
from math import pi, sin, cos, sqrt


class LaserBase(object):
    """base class for all laser usage calss"""
    def __init__(self):
        self.laser_on = False
        self.focal_l = 11  # focal z coordinate
        self.laser_speed = 600
        self.rotation = 0.0
        self.laser_power = 0

        self.split_thres = 999  # should be some small number
        self.current_x = None
        self.current_y = None
        self.current_z = None

        # speed F= mm/minute

        # machine indicate how you pass gcode into machine
        # choose marlin if you are using printrun
        # self.machine = 'pi'
        if "FLUXHW" in os.environ:
            self.machine = os.environ["FLUXHW"]
        else:
            self.machine = 'pi'

        self.ratio = 1.

    def header(self, header):
        """
        header gcode for laser
        """
        gcode = []

        # header part
        gcode.append(";Flux Laser")
        for i in header.split('\n'):
            gcode.append(";" + i)

        #  gcode.append("M666 X-1.95 Y-0.4 Z-2.1 R97.4 H241.2")
        gcode.append("M666 X-1.95 Y-0.4 Z-2.1 R97.4 H241.2")  # new

        gcode += self.turnOff()
        gcode.append("G28")

        # TODO: this should be in config file
        self.current_x = 0.
        self.current_y = 0.
        self.current_z = 241.20

        gcode.append(";G29")

        gcode.append("G1 F5000 Z" + str(self.focal_l) + "")
        return gcode

    def turnOn(self):
        if self.laser_on:
            return []
        self.laser_on = True
        if self.machine == 'marlin':
            return ["G4 P1", "G4 P1", ] + ["G4 P10", "@X9L%d" % self.laser_power, "G4 P1"]
        elif self.machine == 'pi':
            return ["G4 P1", "G4 P1", ] + ["G4 P10", "HL%d" % self.laser_power, "G4 P1"]

    def turnOff(self):
        if not self.laser_on:
            return []
        self.laser_on = False
        if self.machine == 'marlin':
            return ["G4 P1", "G4 P1", ] + ["G4 P1", "@X9L255", "G4 P1"]
        elif self.machine == 'pi':
            return ["G4 P1", "G4 P1", ] + ["G4 P1", "HL255", "G4 P1"]

    def turnHalf(self):
        self.laser_on = False
        if self.machine == 'marlin':
            return ["G4 P1", "G4 P1", ] + ["G4 P1", "@X9L250", "G4 P1"]
        elif self.machine == 'pi':
            return ["G4 P1", "G4 P1", ] + ["G4 P1", "HL250", "G4 P1"]

    def moveTo(self, x, y, speed=None):
        """
            apply global rotation and scale
            move to position x,y
            if distance need to move is larger than self.split_thres,
            path will split into many different command in order to support emergency stop
        """
        gcode = []

        x2 = (x * cos(self.rotation) - y * sin(self.rotation)) * self.ratio
        y2 = (x * sin(self.rotation) + y * cos(self.rotation)) * self.ratio

        x = x2
        y = y2

        ending = ';Move to'

        if speed == 'draw':
            speed = self.laser_speed
            ending = ';Draw to'
        elif speed == 'move':
            speed = 600
            ending = ';Move to'
        elif not speed:
            speed = 600
            ending = ';Move to'

        # # split path to small step
        # # (vx, vy) : direction vector
        # vx = x - self.current_x
        # vy = y - self.current_y
        # len_v = sqrt(vx ** 2 + vy ** 2)
        # if len_v > self.split_thres:
        #     # convert to unit vector (as the smallest step length)
        #     ux = vx / len_v
        #     uy = vy / len_v
        #     for _ in range(int(len_v / self.split_thres)):  # math.floor of (len_v / self.split_thres)
        #         self.current_x += ux * self.split_thres
        #         self.current_y += uy * self.split_thres
        #         gcode += ["G1 F" + str(speed) + " X" + str(self.current_x) + " Y" + str(self.current_y) + ";Split draw to"]

        self.current_x = x
        self.current_y = y

        return gcode + ["G1 F" + str(speed) + " X" + str(x) + " Y" + str(y) + ending]

    def drawTo(self, x, y, speed='draw'):
        """
            turn on, move to x, y

            draw to position x,y with speed
        """
        gcode = []
        gcode += self.turnOn()
        gcode += self.moveTo(x, y, speed)

        return gcode

    def closeTo(self, x, y, speed=None):
        """
            turn off, move to x, y
        """
        gcode = []
        gcode += self.turnOff()
        gcode += self.moveTo(x, y, speed)
        return gcode

    def to_image(self, buffer_data, img_width, img_height):
        """
        convert buffer_data(bytes) to a 2d array
        """
        int_data = list(buffer_data)
        assert len(int_data) == img_width * img_height, "data length != width * height, %d != %d * %d" % (len(int_data), img_width, img_height)
        image = [int_data[i * img_width: (i + 1) * img_width] for i in range(img_height)]

        return image

    def gcode_generate(self):
        """gcode_generate"""
        gcode = self.header('laser svg')

        return "\n".join(gcode) + "\n"

    def export_to_stream(self, stream, *args):
        stream.write(self.gcode_generate(*args))

    def set_params(self, key, value):
        if key == 'object_height':
            self.focal_l += float(value)
        elif key == 'laser_speed':
            self.laser_speed = float(value) * 60  # mm/s -> mm/min

        elif key == 'power':
            self.laser_power = int((100 - float(value)) / 100 * 255)   # pwm, int
        else:
            raise ValueError('undefine setting key')
