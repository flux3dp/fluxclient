#coding=utf-8
#!/usr/bin/env python3
import pytest
import sys
import os
from time import sleep
import unittest
import random
import string

from fluxclient.printer.stl_slicer import StlSlicer


@pytest.fixture(scope="module", params=["tests/printer/data/cube_ascii.stl", "tests/printer/data/cube.stl"])
def stl_binary(request):
    buf = open(request.param, 'rb').read()
    return buf


@pytest.fixture(scope="module", params=["tests/printer/data/17-18.obj"])
def obj_binary(request):
    buf = open(request.param, 'rb').read()
    return buf


@pytest.fixture(scope="module")
def img_buf(request):
    buf = open('tests/printer/data/worden.jpg', 'rb').read()
    return buf


class TestPrinter:

    def setup(self):
        pass

    def test_reset(self):
        k = ''.join(random.choice(string.printable) for _ in range(50))
        _stl_slicer = StlSlicer(k)
        assert _stl_slicer.slic3r == k

        k = ''.join(random.choice(string.printable) for _ in range(50))
        _stl_slicer.reset(k)
        assert _stl_slicer.slic3r == k

    def test_read_stl(self, stl_binary):
        StlSlicer.read_stl(stl_binary)

    def test_read_obj(self, obj_binary):
        StlSlicer.read_obj(obj_binary)

    def test_upload(self, stl_binary):
        _stl_slicer = StlSlicer('')
        assert _stl_slicer.upload('tmp', b'') is False
        assert _stl_slicer.upload('tmp', b'', 'GG') is False
        assert _stl_slicer.upload('tmp', stl_binary) is True

    def test_upload_obj(self, obj_binary):
        _stl_slicer = StlSlicer('')
        assert _stl_slicer.upload('tmp', obj_binary) is False
        assert _stl_slicer.upload('tmp', obj_binary, 'obj') is True

    def test_duplicate(self, stl_binary):
        _stl_slicer = StlSlicer('')
        _stl_slicer.upload('tmp', stl_binary)
        _stl_slicer.duplicate('tmp', 'tmp2')
        assert 'tmp2' in _stl_slicer.models

    def test_upload_image(self, img_buf):
        _stl_slicer = StlSlicer('')
        _stl_slicer.upload_image(img_buf)

    def test_delete(self, stl_binary):
        _stl_slicer = StlSlicer('')
        _stl_slicer.upload('tmp', stl_binary)

        flag, msg = _stl_slicer.delete('tmp')
        assert 'tmp' not in _stl_slicer.models
        assert 'tmp' not in _stl_slicer.parameter
        assert flag is True

        flag, msg = _stl_slicer.delete('tmp')
        assert flag is False

    def test_advanced_setting(self):
        # normal case
        _stl_slicer = StlSlicer('')
        c1 = """
        extrusion_width = 0.4
        fan_always_on = 0
        fan_below_layer_time = 15
        filament_colour = #FFFFFF
        filament_diameter = 1.75
        fill_angle = 45
        fill_density = 20%
        fill_pattern = honeycomb
        first_layer_acceleration = 0
        first_layer_bed_temperature = 0
        first_layer_extrusion_width = 120%"""
        assert _stl_slicer.advanced_setting(c1) == []

        # error by checking function
        c2 = """
        extrusion_width = 0.4
        fan_always_on = 0
        fan_below_layer_time = 15
        filament_colour = #FFFFFF
        filament_diameter = 1.75
        fill_angle = 45
        fill_density = 20%
        fill_pattern = jefkwjeofiw
        first_layer_acceleration = 0
        first_layer_bed_temperature = 0
        first_layer_extrusion_width = 120%"""
        assert _stl_slicer.advanced_setting(c2)[0][0] == 9

        # errors by checking function
        c3 = """
        extrusion_width = 0.4
        fan_always_on = 0
        fan_below_layer_time = 15
        filament_colour = #FFFFFF
        filament_diameter = 1.75
        fill_angle = 999
        fill_density = 20%
        fill_pattern = jefkwjeofiw
        first_layer_acceleration = 0
        first_layer_bed_temperature = 0
        first_layer_extrusion_width = 120%"""
        assert len(_stl_slicer.advanced_setting(c3)[0]) == 2

        # syntax error
        c4 = """fwehifuhi"""
        assert _stl_slicer.advanced_setting(c4) == [(1, 'syntax error: ' + c4)]

        # comment
        c4 = """# fwehifuhi"""
        assert _stl_slicer.advanced_setting(c4) == []

    def test_set(self):
        _stl_slicer = StlSlicer('')
        assert _stl_slicer.set('tmp', [1, 2, 3]).endswith('not upload yet')

    @unittest.skipIf(not os.path.isfile('../Slic3r/slic3r.pl'), "specify slic3r path in os.environ")
    def test_slicing(self, stl_binary):
        _stl_slicer = StlSlicer('../Slic3r/slic3r.pl')
        _stl_slicer.upload('tmp', stl_binary)
        _stl_slicer.set('tmp', [0, 0, 4.5, 0, 0, 0, 1, 1, 1])
        _stl_slicer.begin_slicing(['tmp'], None, '-f')
        sleep(1)
        from time import time
        t_s = time()
        while time() - t_s < 5:
            a = _stl_slicer.report_slicing()
            if a and a[-1].startswith('{"slice_status": "complete"'):
                break
            sleep(0.5)
        else:
            assert 0, 'slicing timeout'
