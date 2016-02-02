#coding=utf-8
#!/usr/bin/env python3
import pytest
import sys
from time import sleep

from fluxclient.printer.stl_slicer import StlSlicer


@pytest.fixture(scope="module", params=["tests/printer/data/cube_ascii.stl", "tests/printer/data/cube.stl"])
def stl_binary(request):
    buf = open(request.param, 'rb').read()
    return buf


@pytest.fixture(scope="module")
def img_buf(request):
    buf = open('tests/printer/data/worden.jpg', 'rb').read()
    return buf


class TestPrinter:
    # =====fixtures========

    def setup(self):
        pass

    def test_read_stl(self, stl_binary):
        StlSlicer.read_stl(stl_binary)

    def test_upload(self, stl_binary):
        _stl_slicer = StlSlicer('/Applications/Slic3r.app/Contents/MacOS/slic3r')
        _stl_slicer.upload('tmp', stl_binary)

    def test_duplicate(self, stl_binary):
        _stl_slicer = StlSlicer('/Applications/Slic3r.app/Contents/MacOS/slic3r')
        _stl_slicer.upload('tmp', stl_binary)
        _stl_slicer.duplicate('tmp', 'tmp2')

    def test_upload_image(self, img_buf):
        _stl_slicer = StlSlicer('/Applications/Slic3r.app/Contents/MacOS/slic3r')
        _stl_slicer.upload_image(img_buf)

    def test_delete(self, stl_binary):
        _stl_slicer = StlSlicer('/Applications/Slic3r.app/Contents/MacOS/slic3r')
        _stl_slicer.upload('tmp', stl_binary)
        a, b = _stl_slicer.delete('tmp')
        assert a is True
        a, b = _stl_slicer.delete('tmp')
        assert a is False

    def test_set(self, stl_binary):
        pass

    def test_slicing(self, stl_binary):
        sleep(0)
    # def test_preprocess(self, svg_buf):
    #     data, w, h = SVGParser.preprocess(svg_buf)

    #     print(len(data), w, h)
    #     assert w == 618.858
    #     assert h == 542.706

    #     path_data = SVGParser.elements_to_list(ET.fromstring(data))

    #     parms = (120.00000000000001, 105.22916666666667, -60.33, 52.95, 59.67, -52.28, 0)
    #     SVGParser.process(path_data, parms, [-0.5, -0.5, 618.858, 542.706], 85)


# if __name__ == '__main__':
#     pytest.main(" test_svg.py")
# ./
# content of test_tmpdir.py
# def test_needsfiles(tmpdir):
#     print (tmpdir)
#     assert 0
