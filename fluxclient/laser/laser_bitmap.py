# !/usr/bin/env python3

from math import pi, sin, cos, degrees
import logging

import numpy as np
from PIL import Image

from fluxclient.laser.laser_base import LaserBase


logger = logging.getLogger(__name__)


class LaserBitmap(LaserBase):
    """
    laser_bitmap class:
      generate gcode base on given images
    """
    def __init__(self):
        super(LaserBitmap, self).__init__()
        self.reset()

    def reset(self):
        """
        reset LaserBitmap class
        """
        self.pixel_per_mm = 16  # sample rate for each point
        self.front_end_radius = 250  # front-end input(250 px as r), doesn't matter ()
        self.ratio = self.radius / self.front_end_radius  # ratio for actually moving head

        self.edges = [0, len(self.image_map), 0, len(self.image_map[0])]  # up, down, left, right bound of the image

        self.rotation = 0  # general rotation for final gcode
        self.laser_on = False  # recording if laser is on

        # threshold, pixel on image_map darker than this will trigger laser, actually no use(only 255 or 0 on image_map)
        self.thres = 255
        self.ratio = 1 / self.pixel_per_mm

    def gcode_generate(self):
        """
        return gcode in string type, use method:export_to_stream to export gcode to stream
        """
        gcode = []
        gcode += self.header('bitmap')

        #row iteration
        abs_shift = len(self.image_map) / 2
        for h in range(0, len(self.image_map)):
            #column iteration
            if h % 2 == 1:
                final_x = 0
                itera = reversed(range(0, len(self.image_map)))
            elif h % 2 == 0:
                itera = range(0, len(self.image_map))
                final_x = len(self.image_map)

            for w in itera:
                if self.image_map[h][w] < self.thres:  # acturally meaningless self.thres=255 and only 0 or 255 on image_map
                    if not self.laser_on:
                        last_i = w
                        gcode += self.closeTo(w - abs_shift, h - abs_shift)
                        gcode += self.turnOn()
                else:
                    if self.laser_on:
                        if abs(w - last_i) < 2:  # Single dot
                            pass
                            gcode += ["G4 P100"]
                        elif final_x > 0:
                            gcode += self.drawTo(w - abs_shift, h - abs_shift)
                        else:
                            gcode += self.drawTo(w - abs_shift, h - abs_shift)
                        gcode += self.turnOff()

            if self.laser_on:
                gcode += self.drawTo(final_x - abs_shift, h - abs_shift)
                gcode += self.turnOff()
        gcode += self.turnOff()
        gcode += ["G28"]
        gcode = "\n".join(gcode) + "\n"
        logger.debug("generate gcode done:%d bytes" % len(gcode))

        return gcode


if __name__ == '__main__':
    a = laser_bitmap()
    logger.info(a)
