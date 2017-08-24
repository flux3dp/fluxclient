# !/usr/bin/env python3

import numpy as np
import logging

from .laser_middleware import LaserMiddleware

logger = logging.getLogger(__name__)


class LaserBitmap(LaserMiddleware):
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

    def process(self, processor, res=1):
        processor.append_comment("FLUX Laser Bitmap Tool")
        self.turnOff(processor)
        self.moveTo(processor, x=0, y=0, speed=5000,
                    z=self.focal_l + self.obj_height)

        abs_shift = len(self.image_map) / 2
        t = np.vectorize(lambda x: x if x <= self.thres else 255)
        self.image_map = t(self.image_map)

        itera_o = list(range(0, len(self.image_map)))  # iterate left to right
        itera_r = list(reversed(range(0, len(self.image_map))))  # iterate right to left

        # row iteration
        for h in range(0, len(self.image_map)):
            # column iteration
            if h % res != 0:
                continue

            if self.one_way or h & 1 == 0:
                itera = itera_o
                # this 0.5 is for fixing tiny difference between iter from "left to right " and "right to left"
                abs_shift_x = len(self.image_map) / 2 + 0.5
            elif h & 1 == 1:
                itera = itera_r
                abs_shift_x = len(self.image_map) / 2 - 0.5

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
                        self.moveTo(processor, itera[w_record] - abs_shift_x - 40, abs_shift - h, speed=5000)
                        self.turnOff(processor)
                        self.moveTo(processor, itera[w_record] - abs_shift_x, abs_shift - h)

                    else:
                        self.moveTo(processor, itera[w_record] - abs_shift_x, abs_shift - h)
                    self.turnTo(processor, 255 - this)
                    self.moveTo(processor, itera[w] - abs_shift_x, abs_shift - h)
                    self.turnOff(processor)

            self.turnOff(processor)
        self.turnOff(processor)
