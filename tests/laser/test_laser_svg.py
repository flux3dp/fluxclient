#!/usr/bin/env python3

import unittest
import struct
import os

from PIL import Image

parentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.sys.path.insert(0, parentdir)


class svgTest(unittest.TestCase):
    def test_pretreat(self):
        m_laser_svg = LaserSvg()
        open()
        m_laser_svg.pretreat()

if __name__ == '__main__':
    unittest.main()
