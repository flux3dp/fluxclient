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
        ######################## fake code ####################################
        self.shading = True
        #######################################################################

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

        # apply threshold in a efficient way

        t = np.vectorize(lambda x: x if x <= self.thres else 255)
        self.image_map = t(self.image_map)
        self.dump('./preview_la.png')

        itera_o = list(range(0, len(self.image_map)))
        itera_r = list(reversed(range(0, len(self.image_map))))

        for h in range(0, len(self.image_map)):
            # print(h, 'h')

            #column iteration
            if h % res != 0:
                continue

            if h % 2 == 0:
                itera = itera_o
                final_x = len(self.image_map)
                abs_shift_x = len(self.image_map) / 2 + 0.5
            elif h % 2 == 1:
                final_x = 0
                itera = itera_r
                abs_shift_x = len(self.image_map) / 2 - 0.5

            w = 0
            # gcode += self.turnTo(255 - self.image_map[h][itera[0]])
            # tmp = self.image_map[h][itera[0]]
            while w < len(itera):
                # print(w)
                if w % res != 0:
                    continue
                # self.image_map[h][itera[w]]
                gcode += self.turnTo(255 - self.image_map[h][itera[w]])
                tmp = self.image_map[h][itera[w]]
                while w < len(itera) and self.image_map[h][itera[w]] == tmp:
                    # print(w)
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
        gcode += ["G28"]
        gcode = "\n".join(gcode) + "\n"
        logger.debug("generate gcode done:%d bytes" % len(gcode))
        ######################## fake code ####################################
        self.dump('./preview.png')
        #######################################################################
        return gcode


if __name__ == '__main__':
    a = LaserBitmap()
