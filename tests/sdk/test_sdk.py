#!/usr/bin/env python3

import unittest
import struct
import os

parentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.sys.path.insert(0, parentdir)


class sdk_deltaTest(unittest.TestCase):
    """test scanner/tools.py"""
    def test_dot(self):
        pass
