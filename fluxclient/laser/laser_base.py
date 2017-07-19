# !/usr/bin/env python3

import sys
from io import BytesIO, StringIO
from math import sin, cos, degrees
from time import time
from datetime import datetime

from PIL import Image
import numpy as np
from fluxclient.fcode.g_to_f import GcodeToFcode
import pkg_resources


class LaserBase(object):
    """base class for all laser usage calss"""
    def __init__(self):
        self.laser_on = False
        self.focal_l = 6.4  # focal z coordinate
        self.focus_by_color = False

        self.laser_speed = 300  # speed F= mm/minute
        self.travel_speed = 1000
        self.draw_power = 255  # drawing
        self.fram_power = 30  # indicating

        self.obj_height = 10.9  # rubber
        self.obj_height = 3.21  # wood
        self.obj_height = 1.7  # pcb
        self.height_offset = 0
        self.current_power = 0
        # self.obj_height = 2.56  # wood?
        # self.obj_height = 0.0  # plate

        self.pixel_per_mm = 10  # sample rate for each point
        self.radius = 150  # laser max radius = 85mm

        # list holding current image
        self.reset_image()

        # warning global setting, don't use theese unless you 100% understand what you are doing
        self.rotation = 0
        self.ratio = 1.

        # ext meta data, used when converting to fcode
        self.ext_metadata = {}
        self.ext_metadata['CORRECTION'] = 'N'
        self.ext_metadata['FILAMENT_DETECT'] = 'N'

    def reset_image(self):
        w = self.pixel_per_mm * self.radius * 2
        self.image_map = np.ones((w, w), np.uint8) * 255

    def header(self, header):
        """
        header gcode for laser
        """
        gcode = []

        # header part
        gcode.append(";Generate by Flux Studio %s" % (datetime.fromtimestamp(time()).strftime('on %Y-%m-%d at %H:%M:%S')))
        gcode.append(";Laser Gcode")
        for i in header.split('\n'):
            gcode.append(";" + i)

        # force close laser
        self.laser_on = True
        gcode += self.turnOff()

        # # setting
        # gcode += ["X3F3", "X3F2", "X3F1"]

        # move to proper height
        gcode.append("G1 F5000 Z%.5f" % (self.focal_l + self.obj_height + self.height_offset))

        return gcode

    def turnOn(self):
        if self.laser_on is True:
            return []
        self.laser_on = True
        self.current_power = 255
        return ["X2O%d;turnOn" % self.draw_power, "G4 P20"]

    def turnOff(self):
        if self.laser_on is False:
            return []
        self.laser_on = False
        self.current_power = 0
        return ["X2O0;turnOff", "G4 P20"]

    def turnTo(self, power=None, wait_sec=20):
        """
        set laser power
        """
        if power is None:
            self.laser_on = True
            if self.current_power == power:
                return []
            self.current_power = self.fram_power
            return ["X2O%d" % self.fram_power, "G4 P20"]

        elif power != 0:
            if self.current_power == power:
                return []
            self.current_power = power
            self.laser_on = True
            return ["X2O%d" % round(power * self.draw_power / 255.0), "G4 P%d" % wait_sec]

        elif power == 0:
            if self.current_power == 0:
                return []
            self.current_power = 0
            return self.turnOff()

    def moveZ(self, z):
        """Generate gcode for moving to z (pos_z)"""
        return ["G1 F%.5f Z%.5f" % (self.laser_speed, z)]


    def moveTo(self, x, y, speed=None, z=None, ending=None):
        """
            apply global "rotation" and "scale"
            move to position x,y
        """

        x2 = (x * cos(self.rotation) - y * sin(self.rotation)) * self.ratio
        y2 = (x * sin(self.rotation) + y * cos(self.rotation)) * self.ratio

        x = x2
        y = y2

        if speed is None:
            speed = self.laser_speed

        if ending is None:
            if self.laser_on:
                ending = ';draw'
            else:
                ending = ';move'

        self.current_x = x
        self.current_y = y
        if z is None:
            return ["G1 F%.5f X%.5f Y%.5f %s" % (speed, x, y, ending)]
        else:
            return ["G1 F%.5f X%.5f Y%.5f Z%.5f %s" % (speed, x, y, z, ending)]

    def drawTo(self, x, y, speed=None, z=None):
        """
            turn on, move to x, y

            draw to position x,y
        """
        gcode = []
        gcode += self.turnOn()

        if speed is None:
            gcode += self.moveTo(x, y, self.laser_speed, z, ending=';draw')
        else:
            gcode += self.moveTo(x, y, speed, z, ending=';draw')

        return gcode

    def closeTo(self, x, y, speed=None, z=None):
        """
            turn off, move to x, y
        """
        gcode = []
        gcode += self.turnOff()

        if speed is None:
            gcode += self.moveTo(x, y, self.travel_speed, z)
        else:
            gcode += self.moveTo(x, y, speed, z)
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
        """Virtual function gcode_generate"""
        raise NotImplementedError('Successor didn\'t implement "gcode_generate" method')

    def export_to_stream(self, stream, *args):
        """export gcode to stream"""
        stream.write(self.gcode_generate(*args))

    def set_params(self, key, value):
        """
        set parameters for setting
        """
        if key == 'object_height':
            self.obj_height = float(value)
        elif key == 'height_offset':
            self.height_offset = float(value)
        elif key == 'laser_speed':
            self.laser_speed = float(value) * 60  # mm/s -> mm/min
        elif key == 'power':
            self.draw_power = (round(float(value) * 255))  # pwm, int
        elif key == 'shading':
            self.shading = (int(value) == 1)
        elif key == 'focus_by_color':
            self.focus_by_color = (int(value) == 1)
        elif key == 'one_way':
            self.one_way = (int(value) == 1)
        else:
            raise ValueError('undefine setting key')

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
        def rotate(x, y, rotation, cx=0., cy=0.):
            """
            compute new (x, y) after rotate toward (cx, cy)
            """
            vx = (x - cx)
            vy = (y - cy)
            x = cx + vx * cos(rotation) - vy * sin(rotation)
            y = cy + vx * sin(rotation) + vy * cos(rotation)
            return x, y

        pix = Image.frombytes('L', (img_width, img_height), buffer_data)
        # pix.save('get.png', 'png')

        # image center (rotation center)
        cx = (x1 + x2) / 2.
        cy = (y1 + y2) / 2.

        # compute four original corner
        ox1, oy1 = rotate(x1, y1, -rotation, cx, cy)
        ox3, oy3 = rotate(x2, y2, -rotation, cx, cy)

        # 1 4
        # 2 3
        ox2, oy2 = ox1, oy3
        ox4, oy4 = ox3, oy1

        pix = pix.resize(tuple(map(lambda x: int(x * self.pixel_per_mm), ((ox3 - ox1), (oy3 - oy1)))))

        # rotate four corner
        ox1, oy1 = rotate(ox1, oy1, rotation, cx, cy)
        ox2, oy2 = rotate(ox2, oy2, rotation, cx, cy)
        ox3, oy3 = rotate(ox3, oy3, rotation, cx, cy)
        ox4, oy4 = rotate(ox4, oy4, rotation, cx, cy)

        # find upper-left corner after rotation(edge)
        gx1 = min(ox1, ox2, ox3, ox4)
        gy1 = max(oy1, oy2, oy3, oy4)

        gy1_on_map = round((gx1 / self.radius * len(self.image_map) / 2.) + (len(self.image_map) / 2.))
        gx1_on_map = round(-(gy1 / self.radius * len(self.image_map) / 2.) + (len(self.image_map) / 2.))

        gx2 = max(ox1, ox2, ox3, ox4)
        gy2 = min(oy1, oy2, oy3, oy4)
        gy2_on_map = round((gx2 / self.radius * len(self.image_map) / 2.) + (len(self.image_map) / 2.))
        gx2_on_map = round(-(gy2 / self.radius * len(self.image_map) / 2.) + (len(self.image_map) / 2.))

        # add white frame on each side
        new_pix = Image.new('L', (pix.size[0] + 2, pix.size[1] + 2), 255)
        new_pix.paste(pix, (1, 1))
        new_pix = new_pix.rotate(degrees(rotation), expand=1)

        for h in range(new_pix.size[1]):
            # using white frame to find starting and ending index
            # find_start, find_end for each row
            flag1 = False  # whether find white frame
            for find_s in range(new_pix.size[0]):
                if new_pix.getpixel((find_s, h)) > 0:  # check this thres?
                    find_s = find_s + 1
                    flag1 = True
                    break

            flag2 = False
            for find_e in range(new_pix.size[0] - 1, -1, -1):
                if new_pix.getpixel((find_e, h)) > 0:
                    find_e = find_e
                    flag2 = True
                    break

            # only one white point in this row(cause by resizing)
            # if find_e < find_s and abs(h - (new_pix.size[1] / 2)) > 1:
            #     find_e = new_pix.size[0]

            # NOTE: flag1 always equal to flag2
            if flag1:  # at least one white point in this row
                for w in range(find_s, find_e):
                    if (gx1_on_map + h - len(self.image_map) / 2.) ** 2 + (gy1_on_map + w - len(self.image_map) / 2.) ** 2 < (len(self.image_map) / 2.) ** 2:
                        if new_pix.getpixel((w, h)) <= thres:
                            if self.shading or self.focus_by_color:
                                self.image_map[gx1_on_map + h][gy1_on_map + w] = new_pix.getpixel((w, h))
                            else:
                                self.image_map[gx1_on_map + h][gy1_on_map + w] = 0

    def dump(self, file_name='', mode='save'):
        """
            dump the image of this laser class
        """
        img = Image.fromarray(self.image_map)
        grid = pkg_resources.resource_filename("fluxclient", "assets/grid.png")
        # magic number just for alignment, don't really important
        delta = 0
        img_background = Image.open(grid).resize((img.size[0] + delta, img.size[1] + delta))
        img_background.paste(img, (delta // 2, delta // 2), img.point(lambda x: 255 if x < 255 else 0))
        img = img_background
        if mode == 'save':
            img.save(file_name, 'png')
            return
        elif mode == 'preview':
            # get the preview (640 * 640) png in bytes
            img = img.resize((640, 640))
            b = BytesIO()
            img.save(b, 'png')
            image_bytes = b.getvalue()
            return image_bytes
        else:
            raise NotImplementedError("unsupport mode {}".format(mode), file=sys.stderr)

    def fcode_generate(self, *args):
        f = StringIO()
        f.write(self.gcode_generate(*args))
        f.seek(0)

        fcode_output = BytesIO()
        m_GcodeToFcode = GcodeToFcode(ext_metadata=self.ext_metadata)
        m_GcodeToFcode.image = self.dump(mode='preview')
        m_GcodeToFcode.md['OBJECT_HEIGHT'] = str(self.obj_height)
        m_GcodeToFcode.md['HEIGHT_OFFSET'] = str(self.height_offset)

        m_GcodeToFcode.process(f, fcode_output)
        return fcode_output.getvalue(), m_GcodeToFcode
