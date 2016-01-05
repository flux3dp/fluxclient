# !/usr/bin/env python3

from math import pi, sin, cos, degrees
import logging
from os import environ

import numpy as np
from PIL import Image

from .laser_base import LaserBase


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
        # threshold, pixel on image_map darker than this will trigger laser
        self.shading = True
        self.thres = 255
        self.ratio *= 1 / self.pixel_per_mm

    def gcode_generate(self, res=1):
        """
        return gcode in string type
        res: resolution
        use method: export_to_stream to export gcode to a stream
        """
        gcode = []
        gcode += self.header('FLUX. Laser Bitmap.')

        #row iteration
        abs_shift = len(self.image_map) / 2

        # apply threshold in a efficient way
        t = np.vectorize(lambda x: x if x <= self.thres else 255)
        self.image_map = t(self.image_map)
        # self.dump('tmp.png')

        itera_o = list(range(0, len(self.image_map)))
        itera_r = list(reversed(range(0, len(self.image_map))))

        for h in range(0, len(self.image_map)):

            #column iteration
            if h % res != 0:
                continue

            if h % 2 == 0:
                itera = itera_o
                final_x = len(self.image_map)
                abs_shift_x = len(self.image_map) / 2 + 0.5
                # this 0.5 is for fixing tiny difference when iter from "left to right " and "right to left"
            elif h % 2 == 1:
                final_x = 0
                itera = itera_r
                abs_shift_x = len(self.image_map) / 2 - 0.5

            w = 0
            while w < len(itera):
                if w % res != 0:
                    continue
                if self.shading:
                    gcode += self.turnTo(255 - self.image_map[h][itera[w]])
                else:
                    gcode += self.turnTo(255)
                this = self.image_map[h][itera[w]]
                while w < len(itera) and self.image_map[h][itera[w]] == this:
                    w += 1
                if w == len(itera):
                    if self.image_map[h][itera[-1]] != 255:
                        gcode += self.moveTo(final_x - abs_shift_x, abs_shift - h)
                        gcode += self.turnOff()
                    break
                else:
                    if final_x != 0:
                        gcode += self.moveTo(itera[w] - abs_shift_x, abs_shift - h)
                    else:
                        gcode += self.moveTo(itera[w] - abs_shift_x, abs_shift - h)
            gcode += self.turnOff()

        gcode += self.turnOff()
        gcode = "\n".join(gcode) + "\n"
        logger.debug("generate gcode done:%d bytes" % len(gcode))
        ######################## fake code ####################################
        if environ.get("flux_debug") == '1':
            self.dump('./preview.png')
            with open('output.gcode', 'w') as f:
                print(gcode, file=f)
        #######################################################################
        return gcode


if __name__ == '__main__':
    a = LaserBitmap()
