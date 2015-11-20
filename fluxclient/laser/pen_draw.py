import logging
import datetime
import time

from .laser_svg import LaserSvg


class PenDraw(LaserBase, SVGParser):
    """docstring for PenDraw"""
    def __init__(self):
        super(PenDraw, self).__init__()
        self.lift_height = 100
        self.draw_height = 0
        self.turnOn = self.draw
        self.turnOff = self.lift

    def set_lift_height(self, h):
        self.lift_height = h

    def set_draw_height(self, h):
        self.draw_height = h

    def draw(self):
        if self.laser_on is True:
            return []
        self.laser_on = True
        return ["G1 F5000 Z%.5f;down pen" % self.draw_height]
        # return ["M400", "X2O%d;turnOn" % self.draw_power, "G4 P20"]

    def lift(self):
        if self.laser_on is False:
            return []
        self.laser_on = False
        return ["G1 F5000 Z%.5f;lift pen" % self.lift_height]

    def header(self, header):
        """
        header gcode for laser
        """
        gcode = []

        # header part
        gcode.append(";Generate by Flux Studio %s" % (datetime.datetime.fromtimestamp(time.time()).strftime('on %Y-%m-%d at %H:%M:%S')))
        gcode.append(";Pen holder Gcode")
        for i in header.split('\n'):
            gcode.append(";" + i)

        # setting
        gcode += ["X3F3", "X3F2", "X3F1"]

        # home
        gcode.append("G28")

        # move to proper height
        self.laser_on = False
        gcode += ["G1 F5000 Z%.5f;lift pen" % self.lift_height]
        return gcode
