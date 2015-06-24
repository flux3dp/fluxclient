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
        self.pixel_per_mm = 4  # kind of sample rate, for each mm, but not really matter
        self.radius = 85  # laser max radius = 85mm
        self.front_end_radius = 250  # front-end input(250 px as r)
        self.ratio = self.radius / self.front_end_radius  # ratio for actually moving head

        # list holding current image
        self.image_map = [[255 for w in range(self.pixel_per_mm * self.radius * 2)] for h in range(self.pixel_per_mm * self.radius * 2)]
        self.edges = [0, len(self.image_map), 0, len(self.image_map[0])]  # up, down, left, right bound of the image

        self.rotation = 0  # general rotation for final gcode
        self.laser_on = False  # recording if laser is on

        # threshold, pixel on image_map darker than this will trigger laser, actually no use(only 255 or 0 on image_map)
        self.thres = 100

    # def bitmap_moveTo(self, x, y, speed=600):
    #     # TODO fix this shit
    #     x = float(x) / self.pixel_per_mm - self.radius
    #     y = float(len(self.image_map) - y) / self.pixel_per_mm - self.radius
    #     return self.moveTo(x, y, speed)

    # def bitmap_drawTo(self, x, y, speed='draw'):
    #     gcode = []

    #     gcode += self.turnOn()
    #     gcode += self.bitmap_moveTo(x, y, speed)
    #     gcode += self.turnOff()

    #     return gcode

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
        # pix = self.to_image(buffer_data, img_width, img_height)

        logger.debug("Recv (%i, %i) @ [%.2f, %.2f], [%.2f, %.2f] R%.2f" %
                     (img_width, img_height, x1, y1, x2, y2, rotation))

        # protocol fail
        # logger.debug("Corner: [%.2f, %.2f], [%.2f, %.2f]" % (x1, y1, x2, y2))
        # real_width = float(x2 - x1)
        # real_height = float(y1 - y2)

        # rotation center
        cx = (x1 + x2) / 2.
        cy = (y1 + y2) / 2.
        logger.debug('c %f, %f' % (cx, cy))

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
        gx = min(ox1, ox2, ox3, ox4)
        gy = max(oy1, oy2, oy3, oy4)  # TODO: change max to min if change coordinate in the future
        print(gx, gy)
        gy_on_map = round((gx / self.radius * len(self.image_map) / 2.) + (len(self.image_map) / 2.))
        gx_on_map = round(-(gy / self.radius * len(self.image_map) / 2.) + (len(self.image_map) / 2.))
        print(gx_on_map, gy_on_map)

        # logger.info("Corner rotate: [%.2f, %.2f], [%.2f, %.2f]" %
        #             (x1, y1, x2, y2))
        # pix = np.array(pix)

        # add white frame on each side
        new_pix = Image.new('L', (pix.size[0] + 2, pix.size[1] + 2), 255)
        new_pix.paste(pix, (1, 1))
        new_pix = new_pix.rotate(degrees(rotation), expand=1)

        for h in range(new_pix.size[0]):
            # using white frame to find start and end index
            for find_s in range(new_pix.size[1]):
                if new_pix.getpixel((h, find_s)) == 255:
                    find_s += 1
                    break
            for find_e in range(new_pix.size[1] - 1, -1, -1):
                if new_pix.getpixel((h, find_e)) == 255:
                    break
            # print(find_s, find_e, find_e-find_s)

            for w in range(find_s, find_e):
                if (gx_on_map + w - len(self.image_map) / 2.) ** 2 + (gy_on_map + h - len(self.image_map) / 2.) ** 2 < (len(self.image_map) / 2.) ** 2:
                    self.image_map[gx_on_map + w][gy_on_map + h] = new_pix.getpixel((h, w))

                # try:
                #     self.image_map[gx_on_map + w][gy_on_map + h] = new_pix.getpixel((h, w))
                # except:
                #     pass


                # if real_x ** 2 + real_y ** 2 <= self.radius ** 2:
                #     if pix[h][w] < thres:
                #         # [TODO]
                #         # if picture  pixel is small, when mapping to image_map should add more interval points
                #         # but not gonna happen in near future?
                #         x_on_map = int(round(self.radius * self.pixel_per_mm + real_x / self.pixel_per_mm))
                #         y_on_map = int(round(self.radius * self.pixel_per_mm + real_y / self.pixel_per_mm))
                #         # self.image_map[x_on_map][y_on_map] = pix[h][w]
                #         self.image_map[x_on_map][y_on_map] = 0
        # alignment fail when float to int

    def find_edges(self):
        """
        find the edge of 4 sides
        return left-bound, right-bound, up-bound, down-bound
        """
        x1 = -53.03  # = 75/(sqrt(2))  Cyclic quadrilateral
        for i in range(len(self.image_map)):
            if any(j == 0 for j in self.image_map[i]):
                x1 = i
                break

        x2 = 53.03
        for i in range(len(self.image_map) - 1, 0 - 1, -1):
            if any(j == 0 for j in self.image_map[i]):
                x2 = i
                break

        y1 = False
        for j in range(len(self.image_map[0])):
            for i in range(len(self.image_map)):
                if self.image_map[i][j] == 0:
                    y1 = j
                    break
            if y1 is not False:
                break
        if y1 is False:
            y1 = 53.03

        y2 = False
        for j in range(len(self.image_map[0]) - 1, 0 - 1, -1):
            for i in range(len(self.image_map) - 1, 0 - 1, -1):
                if self.image_map[i][j] == 0:
                    y2 = j
                    break
            if y2 is not False:
                break
        if y2 is False:
            y2 = 53.03
        self.edges = [x1, x2, y1, y2]
        logger.info(self.edges)

    def alignment_process(self, times=3):
        gcode = []
        self.find_edges()
        gcode += self.turnHalf()
        for _ in range(times):
            gcode += self.moveTo(self.edges[0], self.edges[2])
            gcode += ["G4 P300"]
            gcode += self.moveTo(self.edges[0], self.edges[3])
            gcode += ["G4 P300"]
            gcode += self.moveTo(self.edges[1], self.edges[3])
            gcode += ["G4 P300"]
            gcode += self.moveTo(self.edges[1], self.edges[2])
            gcode += ["G4 P300"]
        gcode += self.turnOff()

        return gcode

    def export_to_stream(self, stream):
        stream.write(self.gcode_generate())

    def gcode_generate(self):
        gcode = []

        gcode += self.header('bitmap')
        # pix = cv2.imread('S.png')
        # pix = cv2.cvtColor(pix, cv2.COLOR_BGR2GRAY)

        # print pix.shape
        # input()

        # last_i = 0
        # # gcode += ["M104 S200"]
        # gcode += turnOff()
        # gcode += turnHalf()

        # gcode += self.alignment_process()

        #row iteration
        for h in range(0, len(self.image_map)):
            #column iteration
            itera = range(0, len(self.image_map))
            final_x = len(self.image_map)
            if h % 2 == 1:
                final_x = 0
                itera = reversed(range(0, len(self.image_map)))

            for w in itera:
                if self.image_map[h][w] < self.thres:
                    if not self.laser_on:
                        last_i = w
                        gcode += self.moveTo(w, h)
                        gcode += self.turnOn()
                else:
                    if self.laser_on:
                        if abs(w - last_i) < 2:  # Single dot
                            pass
                            gcode += ["G4 P100"]
                        elif final_x > 0:
                            gcode += self.drawTo(w, h)
                        else:
                            gcode += self.drawTo(w, h)
                        gcode += self.turnOff()

            if self.laser_on:
                gcode += self.drawTo(final_x, h)
                gcode += self.turnOff()

        gcode += ["G28"]
        # self.dump('gen.jpg',)
        # return ''
        return "\n".join(gcode) + "\n"

    def dump(self, file_name):
        img = np.uint8(np.array(self.image_map))
        img = Image.fromarray(img)
        img.save(file_name, 'JPEG')
        return

if __name__ == '__main__':
    a = laser_bitmap()
    logger.info(a)
