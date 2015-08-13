# !/usr/bin/env python3
import os
import io
from math import pi, sin, cos, sqrt, degrees

from PIL import Image
import numpy as np


class LaserBase(object):
    """base class for all laser usage calss"""
    def __init__(self):
        self.laser_on = False
        self.focal_l = 3.0  # focal z coordinate

        self.rotation = 0.0

        self.laser_speed = 600
        self.travel_speed = 6000
        self.draw_power = 255  # drawing
        self.fram_power = 230  # indicating
        self.obj_height = 3.21

        self.split_thres = 999  # should be some small number
        self.current_x = None
        self.current_y = None
        self.current_z = None

        self.pixel_per_mm = 16  # sample rate for each point
        self.radius = 85  # laser max radius = 85mm
        # list holding current image
        self.reset_image()

        # speed F= mm/minute

        self.ratio = 1.

    def reset_image(self):
        w = self.pixel_per_mm * self.radius * 2
        self.image_map = np.ones((w, w), np.uint8) * 255
        # self.image_map = [[255 for w in range(self.pixel_per_mm * self.radius * 2)] for h in range(self.pixel_per_mm * self.radius * 2)]

    def header(self, header):
        """
        header gcode for laser
        """
        gcode = []

        # header part
        gcode.append(";Flux Laser")
        for i in header.split('\n'):
            gcode.append(";" + i)

        self.laser_on = True
        gcode += self.turnOff()
        gcode.append("G28")

        # TODO: this should be in config file
        self.current_x = 0.
        self.current_y = 0.
        self.current_z = 241.20

        gcode.append(";G29")

        gcode.append("G1 F5000 Z" + str(self.focal_l + self.obj_height))
        return gcode

    def turnOn(self):
        if self.laser_on:
            return []
        self.laser_on = True
        return ["G4 P1", "G4 P1"] * 16 + ["G4 P10", "HL%d" % self.draw_power, "G4 P1"]

    def turnOff(self):
        if not self.laser_on:
            return []
        self.laser_on = False
        return ["G4 P1", "G4 P1"] * 16 + ["G4 P1", "HL0", "G4 P1"]

    def turnHalf(self):
        self.laser_on = False
        return ["G4 P1", "G4 P1", ] + ["G4 P1", "HL%d" % self.fram_power, "G4 P1"]

    def moveTo(self, x, y, speed=None):
        """
            apply global rotation and scale
            move to position x,y
            if distance need to move is larger than self.split_thres,
            path will split into many different command in order to support emergency stop
        """
        gcode = []

        x2 = (x * cos(self.rotation) - y * sin(self.rotation)) * self.ratio
        y2 = (x * sin(self.rotation) + y * cos(self.rotation)) * self.ratio

        x = x2
        y = y2

        ending = ';Move to'
        if speed is None:
            speed = self.laser_speed

        # NOTE: Deprecated!!
        # # split path to small step
        # # (vx, vy) : direction vector
        # vx = x - self.current_x
        # vy = y - self.current_y
        # len_v = sqrt(vx ** 2 + vy ** 2)
        # if len_v > self.split_thres:
        #     # convert to unit vector (as the smallest step length)
        #     ux = vx / len_v
        #     uy = vy / len_v
        #     for _ in range(int(len_v / self.split_thres)):  # math.floor of (len_v / self.split_thres)
        #         self.current_x += ux * self.split_thres
        #         self.current_y += uy * self.split_thres
        #         gcode += ["G1 F" + str(speed) + " X" + str(self.current_x) + " Y" + str(self.current_y) + ";Split draw to"]

        self.current_x = x
        self.current_y = y

        return gcode + ["G1 F" + str(speed) + " X" + str(x) + " Y" + str(y) + ending]

    def drawTo(self, x, y, speed=None):
        """
            turn on, move to x, y

            draw to position x,y
        """
        gcode = [';draw']
        gcode += self.turnOn()

        if speed is None:
            gcode += self.moveTo(x, y)
        else:
            gcode += self.moveTo(x, y, speed=speed)

        return gcode

    def closeTo(self, x, y, speed=None):
        """
            turn off, move to x, y
        """
        gcode = []
        gcode += self.turnOff()

        if speed is None:
            gcode += self.moveTo(x, y)
        else:
            gcode += self.moveTo(x, y, speed)
        return gcode

    def to_image(self, buffer_data, img_width, img_height):
        """
        convert buffer_data(bytes) to a 2d array
        """
        int_data = list(buffer_data)
        assert len(int_data) == img_width * img_height, "data length != width * height, %d != %d * %d" % (len(int_data), img_width, img_height)
        image = [int_data[i * img_width: (i + 1) * img_width] for i in range(img_height)]

        return image

    def gcode_generate(self):
        """gcode_generate"""
        gcode = self.header('laser svg')

        return "\n".join(gcode) + "\n"

    def export_to_stream(self, stream, *args):
        stream.write(self.gcode_generate(*args))

    def set_params(self, key, value):
        if key == 'object_height':
            self.obj_height = float(value)

        elif key == 'laser_speed':
            self.laser_speed = float(value) * 60  # mm/s -> mm/min

        elif key == 'power':
            self.draw_power = round(float(value) * 255)   # pwm, int
        else:
            raise ValueError('undefine setting key')

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

        # image center (rotation center)
        cx = (x1 + x2) / 2.
        cy = (y1 + y2) / 2.

        # compute four original corner
        ox1, oy1 = self.rotate(x1, y1, -rotation, cx, cy)
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

    def dump(self, file_name, mode='save'):
        img = Image.fromarray(self.image_map)
        if mode == 'save':
            img.save(file_name, 'png')
            return
        elif mode == 'preview':
            img.resize(640, 640)

            b = io.BytesIO()
            img.save(b, 'png')
            image_bytes = b.getvalue()
            return image_bytes

    def get_preview(self):
        return self.dump('', mode='preview')
