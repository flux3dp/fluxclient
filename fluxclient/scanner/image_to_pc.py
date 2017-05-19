#!/usr/bin/env python3

from io import BytesIO
import logging
import struct
import sys
import os

import numpy as np
from PIL import ImageChops, Image

from fluxclient.scanner.tools import write_pcd
from fluxclient.scanner import freeless


logger = logging.getLogger(__name__)


class image_to_pc():
    """docstring for image_to_pc"""
    def __init__(self, steps, scan_settings):
        logger.info('init image2pc')
        self.reset(steps, scan_settings)

    def reset(self, steps, scan_settings):
        self.settings = scan_settings
        self.points_L = []
        self.fs_L = freeless.freeless(self.settings.laserX_L, self.settings.laserZ_L, self.settings)

        self.points_R = []
        self.fs_R = freeless.freeless(self.settings.laserX_R, self.settings.laserZ_R, self.settings)

        self.step_counter = 0
        self.steps = steps

    def to_image(self, buffer_data):
        """
            convert buffer_data(bytes readin from jpg) into image -> (numpy.ndarray, uint8)
        """
        f = BytesIO(buffer_data)
        im = Image.open(f)
        im_array = np.array(im)

        # change order from "rgb" to "bgr" <- cv2's order
        im_array[:, :, [0, 2]] = im_array[:, :, [2, 0]]
        return im_array

    def feed(self, buffer_O, buffer_L, buffer_R, step, l_cab, r_cab):
        """
            feed 3 picture buffer and a step index
            note that this step index is the input
            p1[x-coordinate, y-coord, z-coord, r, g, b, step, x, y]
        """

        img_O = self.to_image(buffer_O)
        img_L = self.to_image(buffer_L)
        img_R = self.to_image(buffer_R)

        path = ""
        if os.path.exists("C:\\DeltaScanResult"):
            path = "C:\\DeltaScanResult"
        if os.path.exists("DeltaScanResult"):
            path = "DeltaScanResult"
        if os.path.exists("/Users/simon/Dev/ScanResult"):
            path = "/Users/simon/Dev/ScanResult"

        # Check Domain
        if path != "":
            im1 = Image.fromarray(img_O)
            b, g, r = im1.split()
            im1 = Image.merge("RGB", (r, g, b))
            im1.save("/Users/simon/Dev/ScanResult/%03d_O.png" % (step))
            im2 = Image.fromarray(img_L)
            b, g, r = im2.split()
            im2 = Image.merge("RGB", (r, g, b))
            im2.save("/Users/simon/Dev/ScanResult/%03d_L.png" % (step))
            im3 = Image.fromarray(img_R)
            b, g, r = im3.split()
            im3 = Image.merge("RGB", (r, g, b))
            im3.save("/Users/simon/Dev/ScanResult/%03d_R.png" % (step))
            im = ImageChops.difference(im2, im1)
            im.save("/Users/simon/Dev/ScanResult/%03d_D.png" % (step))

        indices_L = self.fs_L.subProcess(img_O, img_L, self.settings.img_height)

        indices_L = [[p[0], p[1] + l_cab]for p in indices_L]
        # indices_L = [[i, step] for i in range(self.settings.img_height)]

        point_L_this = self.fs_L.img_to_points(img_O, img_L, indices_L, step, 'L', l_cab, clock=True)
        self.points_L.extend(point_L_this)
        # return [self.points_to_bytes(point_L_this), []]

        indices_R = self.fs_R.subProcess(img_O, img_R, self.settings.img_height)
        indices_R = [[p[0], p[1] + r_cab]for p in indices_R]
        point_R_this = self.fs_R.img_to_points(img_O, img_R, indices_R, step, 'R', r_cab, clock=True)
        self.points_R.extend(point_R_this)
        # return [[], self.points_to_bytes(point_R_this)]
        return [self.points_to_bytes(point_L_this), self.points_to_bytes(point_R_this)]

    def points_to_bytes(self, points):
        """
        convert points to bytes
        input format:

        [[x-coordinate, y-coord, z-coord, r, g, b],
         [x-coordinate, y-coord, z-coord, r, g, b],
         [x-coordinate, y-coord, z-coord, r, g, b]
         ]

        output format: check https://github.com/flux3dp/fluxghost/wiki/websocket-3dscan-control
        """
        return [struct.pack('<ffffff', p[0], p[1], p[2], p[3] / 255., p[4] / 255., p[5] / 255.) for p in points]

    def merge(self):
        """
        merge left and right scanned points
        find which side is brighter, use it as base
        use Left side as base
        """
        s_R = sum(int(p[3]) + p[4] + p[5] for p in self.points_R)
        s_L = sum(int(p[3]) + p[4] + p[5] for p in self.points_L)

        if s_R > s_L:
            base = self.points_R
            add_on = self.points_L
            delta = round(60 / (360 / self.steps))
        else:
            base = self.points_L
            add_on = self.points_R
            delta = -round(60 / (360 / self.steps))

        record = {}
        for p in range(len(base)):
            record[(base[p][6], base[p][8])] = p

        self.points_M = base[:]
        logger.debug("merging base %s, add_on %s", len(base), len(add_on))

        for p in add_on:
            t = (p[6] + delta) % 400, p[8]
            if t in record:
                old_p = base[record[t]]
                base[record[t]][3] = p[3] / 2 + old_p[3] / 2
                base[record[t]][4] = p[4] / 2 + old_p[4] / 2
                base[record[t]][5] = p[5] / 2 + old_p[5] / 2
            else:
                self.points_M.append(p)

        logger.warning('merge done: output self.mpoints_M:%s', len(self.points_M))


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
    for i in range(-250, 1000):
        l.append([0, 0, i / 10, 255, 0, 0])

    for i in range(-70, 70, 3):
        for j in range(-70, 70, 3):
            l.append([i, j, 0.0, 255, 0, 0])

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
    import subprocess
    from fluxclient.scanner.scan_settings import ScanSetting

    m, l, r = 326.0, 320.0, 328.0

    ss = 400
    SS = ScanSetting()
    SS.cab_m, SS.cab_l, SS.cab_r = m, l, r

    SS.LLaserAdjustment = int(m) - (SS.img_width / 2)
    SS.RLaserAdjustment = int(m) - (SS.img_width / 2)
    m_image_to_pc = image_to_pc(ss, SS)
    img_location = sys.argv[1].rstrip('/')
    print(img_location)

    for i in range(ss):
        tmp = [open(img_location + '/' + str(i).zfill(3) + '_' + j + '.jpg', 'rb').read() for j in ['O', 'L', 'R']]
        m_image_to_pc.feed(*tmp, step=i, l_cab=-SS.LLaserAdjustment, r_cab=-SS.RLaserAdjustment)

        print_progress(i, 400)
    print('')
    output = img_location + '.pcd'
    print('start merge')
    m_image_to_pc.merge()
    print('merge done')
    write_pcd(after(m_image_to_pc.points_M), output)
    subprocess.call(['python2', '../../../3ds/3ds/PCDViewer/pcd_to_js.py', output], stdout=open('../../../3ds/3ds/PCDViewer/model.js', 'w'))
