#!/usr/bin/env python3
import struct
import io

import numpy as np
from PIL import Image

import fluxclient.scanner.freeless as freeless
import fluxclient.scanner.scan_settings as scan_settings


class image_to_pc():
    """docstring for image_to_pc"""
    def __init__(self):
        self.reset()

    def reset(self):
        self.points_L = []
        self.fs_L = freeless.freeless(scan_settings.laserX_L, scan_settings.laserZ_L)

        self.points_R = []
        self.fs_R = freeless.freeless(scan_settings.laserX_R, scan_settings.laserZ_R)

        self.step_counter = 0

    def to_image(self, buffer_data):
        '''
            convert buffer_data(bytes readin from jpg) into image -> (numpy.ndarray, uint8)
        '''
        f = io.BytesIO(buffer_data)
        im = Image.open(f)
        im_array = np.array(im)

        # change order from "rgb" to "bgr" <- cv2's order
        im_array[:, :, [0, 2]] = im_array[:, :, [2, 0]]
        return im_array

    def feed(self, buffer_O, buffer_L, buffer_R, step):

        img_O = self.to_image(buffer_O)
        img_L = self.to_image(buffer_L)
        img_R = self.to_image(buffer_R)

        indices_L = self.fs_L.subProcess(img_O, img_L, scan_settings.img_height)
        point_L_this = self.fs_L.img_to_points(img_O, img_L, indices_L, step, 'L', clock=True)
        self.points_L.extend(point_L_this)

        indices_R = self.fs_R.subProcess(img_O, img_R, scan_settings.img_height)
        point_R_this = self.fs_R.img_to_points(img_O, img_R, indices_R, step, 'R', clock=True)
        self.points_R.extend(point_R_this)

        return [self.points_to_bytes(point_L_this), self.points_to_bytes(point_R_this)]

    def points_to_bytes(self, points):
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
        return [struct.pack('<ffffff', p[0], p[1], p[2], p[5] / 255., p[4] / 255., p[3] / 255.) for p in points]
