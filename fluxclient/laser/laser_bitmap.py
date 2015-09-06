# !/usr/bin/env python3

from math import pi, sin, cos, degrees
import logging

import numpy as np
from PIL import Image

from fluxclient.laser.laser_base import LaserBase


logger = logging.getLogger(__name__)


class LaserBitmap(LaserBase):
    """
    LaserBitmap class:
      generate gcode base on given images
    """
    def __init__(self):
        super(LaserBitmap, self).__init__()
        self.reset()

    def reset(self):
        """
        reset LaserBitmap class
        """
        # threshold, pixel on image_map darker than this will trigger laser, actually no use(only 255 or 0 on image_map)
        self.thres = 255
        self.ratio *= 1 / self.pixel_per_mm

    def gcode_generate(self, res=1):
        """
        return gcode in string type, use method:export_to_stream to export gcode to stream
        """
        gcode = []
        gcode += self.header('FLUX. Laser Bitmap.')

        #row iteration
        abs_shift = len(self.image_map) / 2
        for h in range(0, len(self.image_map)):
            #column iteration
            if h % res != 0:
                continue

            if h % 2 == 0:
                itera = range(0, len(self.image_map))
                final_x = len(self.image_map)
            elif h % 2 == 1:
                final_x = 0
                itera = reversed(range(0, len(self.image_map)))

            for w in itera:
                if w % res != 0:
                    continue
                if self.image_map[h][w] < self.thres:  # acturally meaningless self.thres=255 and only 0 or 255 on image_map
                    if not self.laser_on:
                        last_i = w
                        if final_x != 0:
                            gcode += self.closeTo(w - 0.5 - abs_shift, abs_shift - h)
                        else:
                            gcode += self.closeTo(w + 0.5 - abs_shift, abs_shift - h)
                        gcode += self.turnOn()
                else:
                    if self.laser_on:
                        if abs(w - last_i) < 2:  # Single dot
                            pass
                            gcode += ["G4 P100"]

                        elif final_x != 0:
                            gcode += self.drawTo(w - abs_shift - 0.5, abs_shift - h)
                        else:
                            gcode += self.drawTo(w - abs_shift + 0.5, abs_shift - h)
                        gcode += self.turnOff()

            if self.laser_on:
                gcode += self.drawTo(final_x - abs_shift, abs_shift - h)
                gcode += self.turnOff()

        gcode += self.turnOff()
        gcode += ["G28"]
        gcode = "\n".join(gcode) + "\n"
        logger.debug("generate gcode done:%d bytes" % len(gcode))
        ######################## fake code ####################################
        self.dump('./preview.png')
        #######################################################################
        return gcode


if __name__ == '__main__':
    a = LaserBitmap()
