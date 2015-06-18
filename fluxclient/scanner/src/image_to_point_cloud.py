#!/usr/bin/env python3
import struct

import numpy as np

import freeless
import scan_settings


def to_image(buffer_data):
    '''
        convert buffer_data into image -> (numpy.ndarray, uint8)
    '''
    int_data = list(buffer_data)
    img_width = scan_settings.img_width
    img_height = scan_settings.img_height

    assert len(int_data) == img_width * img_height * 3, "data length != width * height, %d != %d * %d" % (len(int_data), img_width, img_height)

    image = [int_data[i * img_width: (i + 1) * img_width] for i in range(img_height)]

    return np.array(image, dtype=np.uint8)


def points_to_bytes(points):
    '''
    convert points to bytes
    input format: [
                                p1[x-coordinate, y-coord, z-coord, b, g, r],
                                p2[x-coordinate, y-coord, z-coord, b, g, r],
                                p3[x-coordinate, y-coord, z-coord, b, g, r],
                                  ...
                 ]
    output format: check https://github.com/flux3dp/fluxghost/wiki/websocket-3dscan-control
    '''
    return b''.join([struct.pack('<ffffff', p[0], p[1], p[2], p[5] / 255., p[4] / 255., p[3] / 255.) for p in points])


class image_to_point_cloud():
    """docstring for image_to_point_cloud"""
    def __init__(self):
        self.reset()

    def reset(self):
        self.points_L = []
        self.fs_L = freeless.freeless(scan_settings.laserX_L, scan_settings.laserZ_L)

        self.points_R = []
        self.fs_R = freeless.freeless(scan_settings.laserX_R, scan_settings.laserZ_R)

        self.step_counter = 0

    def feed(self, buffer_O, buffer_L, buffer_R, step):
        img_O = to_image(buffer_O)
        img_L = to_image(buffer_L)
        img_R = to_image(buffer_R)

        indices_L = self.fs_L.subProcess(img_O, img_L)
        point_L_this = self.fs_L.img_to_points(img_O, img_L, indices_L, step, 'L', clock=True)
        points_L.extend(point_L_this)

        indices_R = self.fs_R.subProcess(img_O, img_R)
        point_R_this = self.fs_R.img_to_points(img_O, img_R, indices_R, step, 'R', clock=True)
        points_R.extend(point_R_this)

        return [points_to_bytes(point_L_this), points_to_bytes(point_R_this)]
