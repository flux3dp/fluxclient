# !/usr/bin/env python3

import logging
import datetime
import time

from fluxclient.laser.laser_svg import LaserSvg
from fluxclient.utils.svg_parser import SVGParser
from fluxclient.hw_profile import HW_PROFILE

logger = logging.getLogger(__name__)


class PenSvg(LaserSvg, SVGParser):
    """docstring for PenSvg"""
    def __init__(self):
        super(PenSvg, self).__init__()
        self.shading = False

        self.ext_metadata['HEAD_TYPE'] = 'N/A'
        self.lift_height = 100
        self.draw_height = 0
        self.nozzle_height = HW_PROFILE['model-1']['nozzle_height']
        # override
        self.turnOn = self.draw
        self.turnOff = self.lift

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
        gcode.append(";Pen Gcode")
        gcode.append(";FLUX. Pen SVG.")

        for i in header.split('\n'):
            gcode.append(";" + i)

        # setting
        gcode += ["X3F3", "X3F2", "X3F1"]

        # move to proper height
        self.laser_on = False
        gcode += ["G1 F5000 Z%.5f;lift pen" % self.lift_height]
        return gcode

    def set_params(self, key, value):
        """
        set parameters for setting
        """
        if key == 'lift_height':
            self.lift_height = float(value) - self.nozzle_height

        elif key == 'draw_height':
            self.draw_height = float(value) - self.nozzle_height

        elif key == 'speed':
            self.laser_speed = float(value) * 60  # mm/s -> mm/min

        elif key == 'one_way':
            self.one_way = int(value) == 1
        else:
            raise ValueError('undefine setting key %s' % key)
