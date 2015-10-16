#!/usr/bin/env python3
import struct
import io
import sys
import subprocess

import numpy as np
from PIL import Image

import fluxclient.scanner.freeless as freeless
import fluxclient.scanner.scan_settings as scan_settings
from fluxclient.scanner.tools import write_pcd
from fluxclient.hw_profile import HW_PROFILE


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

        self.ref_table = []
        s = 0
        self.steps = 400
        for i in range(self.steps):
            self.ref_table.append(s)
            s = (s + HW_PROFILE['model-1']['step_setting'][self.steps][0]) % self.steps

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
        '''
            feed 3 picture buffer and a step index
            note that this step index is the input
        '''

        img_O = self.to_image(buffer_O)
        img_L = self.to_image(buffer_L)
        img_R = self.to_image(buffer_R)
        step = self.ref_table[step]

        indices_L = self.fs_L.subProcess(img_O, img_L, scan_settings.img_height)
        point_L_this = self.fs_L.img_to_points(img_O, img_L, indices_L, step, 'L', clock=True)
        self.points_L.extend(point_L_this)

        indices_R = self.fs_R.subProcess(img_O, img_R, scan_settings.img_height)
        point_R_this = self.fs_R.img_to_points(img_O, img_R, indices_R, step, 'R', clock=True)
        self.points_R.extend(point_R_this)

        return [self.points_to_bytes(point_L_this), self.points_to_bytes(point_R_this)]
        return [self.points_to_bytes(point_L_this), []]

    def points_to_bytes(self, points):
        '''
        convert points to bytes
        input format: [
                                    p1[x-coordinate, y-coord, z-coord, r, g, b],
                                    p2[x-coordinate, y-coord, z-coord, r, g, b],
                                    p3[x-coordinate, y-coord, z-coord, r, g, b],
                                      ...
                      ]
        output format: check https://github.com/flux3dp/fluxghost/wiki/websocket-3dscan-control
        '''
        return [struct.pack('<ffffff', p[0], p[1], p[2], p[3] / 255., p[4] / 255., p[5] / 255.) for p in points]

    def merge(self):
        """
        merge left and right scanned points
        use Left side as base
        """
        record = [set() for i in range(self.steps)]
        for p in self.points_R:
            record[p[6]].add(p[8])
        self.points_M = self.points_R[:]

        print(len(self.points_M), len(self.points_L), len(self.points_R))
        # print(record)

        for p in self.points_L:
            if not p[8] in record[p[6]]:
                self.points_M.append(p)

        print(len(self.points_M), len(self.points_L), len(self.points_R))


def print_progress(step, total):
    """
    print progress on screen
    """
    left = int((step / total) * 70)
    right = 70 - left
    sys.stdout.write("\r[%s>%s] Step %3i" % ("=" * left, " " * right, step))
    sys.stdout.flush()


def myrange(*args):
    """
    range function that support float
    """
    start = 0.
    sep = 1
    if len(args) == 2:
        start, end = args
    elif len(args) == 3:
        start, end, sep = args
    elif len(args) == 1:
        end = args[0]
    tmp = start
    l = []
    while tmp < end:
        l.append(tmp)
        tmp += sep
    return l


def after(l):
    """
    add fixed size box
    """
    return l
    for i in range(-70, 70, 3):
        for j in range(-70, 70, 3):
            l.append([i, j, 0.0, 255, 0, 0])

    for i in range(-250, 1000):
        l.append([0, 0, i / 10, 0, 255, 0])

    for i in myrange(0, 62, 0.3):
        l.append([-18, -21, i, 255, 0, 0])
        l.append([18, -21, i, 255, 0, 0])
        l.append([-18, 21, i, 255, 0, 0])
        l.append([18, 21, i, 255, 0, 0])

    for i in myrange(-21, 21, 0.3):
        l.append([18, i, 62, 255, 0, 0])
        l.append([18, i, 0, 255, 0, 0])
        l.append([-18, i, 62, 255, 0, 0])
        l.append([-18, i, 0, 255, 0, 0])

    for i in myrange(-18, 18, 0.3):
        l.append([i, 21, 62, 255, 0, 0])
        l.append([i, 21, 0, 255, 0, 0])
        l.append([i, -21, 62, 255, 0, 0])
        l.append([i, -21, 0, 255, 0, 0])
    return l

if __name__ == '__main__':
    m_image_to_pc = image_to_pc()
    img_location = sys.argv[1].rstrip('/')
    print(img_location)

    for i in range(400):
        tmp = [open(img_location + '/' + str(i).zfill(3) + '_' + j + '.jpg', 'rb').read() for j in ['O', 'L', 'R']]
        m_image_to_pc.feed(*tmp, step=i)
        print_progress(i, 400)
    print('')
    output = img_location + '.pcd'
    m_image_to_pc.merge()
    write_pcd(after(m_image_to_pc.points_M), output)
    subprocess.call(['python', '../../../3ds/3ds/PCDViewer/pcd_to_js.py', output], stdout=open('../../../3ds/3ds/PCDViewer/model.js', 'w'))
