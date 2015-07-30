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
        self.radius = 85  # laser max radius = 85mm
        self.front_end_radius = 250  # front-end input(250 px as r), doesn't matter ()
        self.ratio = self.radius / self.front_end_radius  # ratio for actually moving head

        # list holding current image
        self.image_map = [[255 for w in range(self.pixel_per_mm * self.radius * 2)] for h in range(self.pixel_per_mm * self.radius * 2)]
        self.edges = [0, len(self.image_map), 0, len(self.image_map[0])]  # up, down, left, right bound of the image

        self.rotation = 0  # general rotation for final gcode
        self.laser_on = False  # recording if laser is on

        # threshold, pixel on image_map darker than this will trigger laser, actually no use(only 255 or 0 on image_map)
        self.thres = 255
        self.ratio = 1 / self.pixel_per_mm

    def rotate(self, x, y, rotation, cx=0., cy=0.):
        """
        compute new (x, y) after rotate toward (cx, cy)
        """
        vx = (x - cx)
        vy = (y - cy)
        x = cx + vx * cos(rotation) - vy * sin(rotation)
        y = cy + vx * sin(rotation) + vy * cos(rotation)
        return x, y

    def add_image(self, buffer_data, img_width, img_height, x1, y1, x2, y2, rotation, thres=255):
        """
        add image on top of current image i.e self.image_map
          parameters:
            buffer_data: image data in bytes array
            img_width, img_height: trivial
            x1, y1: absolute position of image's top-left corner after rotation
            x2, y2: absolute position of image's button_right corner after rotation
          return:
            None
        """
        pix = Image.frombytes('L', (img_width, img_height), buffer_data)

        logger.debug("Recv (%i, %i) @ [%.2f, %.2f], [%.2f, %.2f] R%.2f" %
                     (img_width, img_height, x1, y1, x2, y2, rotation))

        # image center (rotation center)
        cx = (x1 + x2) / 2.
        cy = (y1 + y2) / 2.

        # compute four original corner
        ox1, oy1 = self.rotate(x1, y1, - rotation, cx, cy)
        ox3, oy3 = self.rotate(x2, y2, -rotation, cx, cy)

        ox2, oy2 = ox1, oy3
        ox4, oy4 = ox3, oy1

        # rotate four corner
        ox1, oy1 = self.rotate(ox1, oy1, rotation, cx, cy)
        ox2, oy2 = self.rotate(ox2, oy2, rotation, cx, cy)
        ox3, oy3 = self.rotate(ox3, oy3, rotation, cx, cy)
        ox4, oy4 = self.rotate(ox4, oy4, rotation, cx, cy)

        # find upper-left corner after rotation(edge)
        gx1 = min(ox1, ox2, ox3, ox4)
        gy1 = max(oy1, oy2, oy3, oy4)  # TODO: change max to min if change coordinate in the future
        gy1_on_map = round((gx1 / self.radius * len(self.image_map) / 2.) + (len(self.image_map) / 2.))
        gx1_on_map = round(-(gy1 / self.radius * len(self.image_map) / 2.) + (len(self.image_map) / 2.))

        gx2 = max(ox1, ox2, ox3, ox4)
        gy2 = min(oy1, oy2, oy3, oy4)  # TODO: change max to min if change coordinate in the future
        gy2_on_map = round((gx2 / self.radius * len(self.image_map) / 2.) + (len(self.image_map) / 2.))
        gx2_on_map = round(-(gy2 / self.radius * len(self.image_map) / 2.) + (len(self.image_map) / 2.))

        # add white frame on each side
        new_pix = Image.new('L', (pix.size[0] + 2, pix.size[1] + 2), 255)
        new_pix.paste(pix, (1, 1))
        new_pix = new_pix.rotate(degrees(rotation), expand=1)
        new_pix = new_pix.resize((gy2_on_map - gy1_on_map, gx2_on_map - gx1_on_map))

        for h in range(new_pix.size[0]):
            # using white frame to find starting and ending index
            for find_s in range(new_pix.size[1]):
                if new_pix.getpixel((h, find_s)) > 0:
                    find_s += 1
                    break
            for find_e in range(new_pix.size[1] - 1, -1, -1):
                if new_pix.getpixel((h, find_e)) > 0:
                    break

            for w in range(find_s, find_e):
                if (gx1_on_map + w - len(self.image_map) / 2.) ** 2 + (gy1_on_map + h - len(self.image_map) / 2.) ** 2 < (len(self.image_map) / 2.) ** 2:
                    if new_pix.getpixel((h, w)) <= thres:
                        self.image_map[gx1_on_map + w][gy1_on_map + h] = 0

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
            itera = range(0, len(self.image_map))
            final_x = len(self.image_map)
            if h % 2 == 1:
                final_x = 0
                itera = reversed(range(0, len(self.image_map)))

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

        gcode += ["G28"]
        gcode = "\n".join(gcode) + "\n"
        logger.debug("generate gcode done:%d bytes" % len(gcode))

        return gcode

    def dump(self, file_name, mode='save'):
        img = np.uint8(np.array(self.image_map))
        img = Image.fromarray(img)
        if mode == 'save':
            img.save(file_name, 'png')
            return
        else:
            # TODO: generate thumbnail for fcode
            pass
        return

if __name__ == '__main__':
    a = laser_bitmap()
    logger.info(a)
