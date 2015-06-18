#!/usr/bin/env python3

import unittest
import struct
import os

from PIL import Image

parentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.sys.path.insert(0, parentdir)
import image_to_point_cloud


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
    # def test_to_image(self):
    #     # buffer_data = b''

    def test_points_to_bytes(self):
        self.assertEqual(image_to_point_cloud.points_to_bytes([[0, 0, 0, 0, 0, 0]]), b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00')
        self.assertEqual(image_to_point_cloud.points_to_bytes([[99, 99, 99, 255, 255, 255]]), b'\x00\x00\xc6B\x00\x00\xc6B\x00\x00\xc6B\x00\x00\x80?\x00\x00\x80?\x00\x00\x80?')

    def test_image_to_point_cloud(self):
        m_img_to_pc = image_to_point_cloud.image_to_point_cloud()
        tmp = []
        for step in range(10):
            buffer_O, buffer_L, buffer_R = images_loader('data', step)
            # tmp.append(m_img_to_pc.feed(buffer_O, buffer_L, buffer_R, step))
        # print (tmp)

if __name__ == '__main__':
    unittest.main()
