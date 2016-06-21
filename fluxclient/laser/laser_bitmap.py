# !/usr/bin/env python3

from math import pi, sin, cos, degrees
import logging
from os import environ

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
        # threshold, pixel on image_map darker than this will trigger laser
        self.ext_metadata['HEAD_TYPE'] = 'LASER'
        self.shading = True
        self.one_way = True
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

        abs_shift = len(self.image_map) / 2

        # apply threshold in a efficient way
        t = np.vectorize(lambda x: x if x <= self.thres else 255)
        self.image_map = t(self.image_map)

        itera_o = list(range(0, len(self.image_map)))  # iterate left to right
        itera_r = list(reversed(range(0, len(self.image_map))))  # iterate right to left

        #row iteration
        for h in range(0, len(self.image_map)):
            #column iteration
            if h % res != 0:
                continue

            if self.one_way or h & 1 == 0:
                itera = itera_o
                # this 0.5 is for fixing tiny difference between iter from "left to right " and "right to left"
                abs_shift_x = len(self.image_map) / 2 + 0.5
            elif h & 1 == 1:
                itera = itera_r
                abs_shift_x = len(self.image_map) / 2 - 0.5

            final_x = itera[-1]

            w = 0
            back = True
            while w < len(itera):
                if w % res != 0:
                    continue
                # record starting point and the value of this pixel
                w_record = w
                this = self.image_map[h][itera[w]]
                while w < len(itera) and self.image_map[h][itera[w]] == this:
                    w += 1
                if this != 255:
                    if back:
                        back = False
                        # gcode += self.moveTo(itera[w_record] - abs_shift_x, abs_shift - h, speed=8000)
                        gcode += self.moveTo(itera[w_record] - abs_shift_x - 40, abs_shift - h, speed=5000)
                        gcode += self.turnOff()
                        gcode += self.moveTo(itera[w_record] - abs_shift_x, abs_shift - h)

                    else:
                        gcode += self.moveTo(itera[w_record] - abs_shift_x, abs_shift - h)
                    gcode += self.turnTo(255 - this)
                    gcode += self.moveTo(itera[w] - abs_shift_x, abs_shift - h)
                    gcode += self.turnOff()

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
