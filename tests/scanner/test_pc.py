#!/usr/bin/env python3

import unittest
import struct
import os

from PIL import Image

parentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.sys.path.insert(0, parentdir)
from fluxclient.scanner import image_to_pc, tools


def images_loader(location, step):
    img_O = Image.open(location + '/' + str(step).zfill(3) + '_O.png')
    buffer_O = []
    for h in range(img_O.size[1]):
        for w in range(img_O.size[0]):
            for i in img_O.getpixel((w, h)):
                buffer_O.append(i)
    buffer_O = b''.join(struct.pack('<B', i) for i in buffer_O)
    img_O.close()

    img_L = Image.open(location + '/' + str(step).zfill(3) + '_L.png')
    buffer_L = []
    for h in range(img_L.size[1]):
        for w in range(img_L.size[0]):
            for i in img_L.getpixel((w, h)):
                buffer_L.append(i)
    buffer_L = b''.join(struct.pack('<B', i) for i in buffer_L)
    img_L.close()

    img_R = Image.open(location + '/' + str(step).zfill(3) + '_R.png')
    buffer_R = []
    for h in range(img_R.size[1]):
        for w in range(img_R.size[0]):
            for i in img_R.getpixel((w, h)):
                buffer_R.append(i)
    buffer_R = b''.join(struct.pack('<B', i) for i in buffer_R)
    img_R.close()

    return buffer_O, buffer_L, buffer_R


class image_to_pcTest(unittest.TestCase):
    pass
    # def test_to_image(self):
    #     buffer_data = b''

    # # TODO
    # def test_points_to_bytes(self):
    #     m_image_to_pc = image_to_pc.image_to_pc()
    #     self.assertEqual(m_image_to_pc.points_to_bytes([[0, 0, 0, 0, 0, 0]]), b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00')
    #     self.assertEqual(m_image_to_pc.points_to_bytes([[99, 99, 99, 255, 255, 255]]), b'\x00\x00\xc6B\x00\x00\xc6B\x00\x00\xc6B\x00\x00\x80?\x00\x00\x80?\x00\x00\x80?')

    # def test_image_to_point_cloud(self):
    #     m_img_to_pc = image_to_pc.image_to_pc()
    #     tmp = []
    #     for step in range(10):
    #         buffer_O, buffer_L, buffer_R = images_loader('data/inso', step)
    #         # tmp.append(m_img_to_pc.feed(buffer_O, buffer_L, buffer_R, step))
    #     # print (tmp)


class scan_toolTest(unittest.TestCase):
    """test scanner/tools.py"""
    def test_dot(self):
        self.assertEqual(tools.dot((0, 0, 0), (1, 2, 3)), 0)
        self.assertEqual(tools.dot((1, 1, 1), (1, 2, 3)), 6)
        self.assertEqual(tools.dot([1, 1, 1], (1, 2, 3)), 6)
        self.assertEqual(tools.dot((1, 0, 0), (0, 0, 1)), 0)

    def test_cross(self):
        self.assertEqual(tools.cross([0, 0, 0], [1, 2, 3], [4, 5, 6]), -3)
        self.assertEqual(tools.cross([7, 8, -1], [12, 6, 3], [-4, 1, 6]), -57)

    def test_normalize(self):
        test = [([0, 0, 0], [0, 0, 0]),
                ([7, 8, -1], [0.6556100681071858, 0.7492686492653552, -0.0936585811581694]),
                ([9, 9, 9], [0.5773502691896257, 0.5773502691896257, 0.5773502691896257]),
                ([1, 1, 1], [0.5773502691896257, 0.5773502691896257, 0.5773502691896257]),
                ([1, 0, 0], [1, 0, 0])]
        for i, j in test:
            tmp = tools.normalize(i)
            for k in range(3):
                self.assertAlmostEqual(tmp[k], j[k])
