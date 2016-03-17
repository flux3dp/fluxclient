#coding=utf-8
#!/usr/bin/env python3
import pytest
import sys
import xml.etree.ElementTree as ET

from fluxclient.utils.svg_parser import SVGParser


@pytest.fixture(scope="module")
def svg_buf(request):
    buf = open('tests/laser/data/Achtung.svg', 'rb').read()
    return buf


@pytest.fixture(scope="module")
def clean_svg_buf(request):
    buf = open('tests/laser/data/Achtung.svg', 'rb').read()
    w, d = SVGParser.preprocess(buf)
    data, w, h = d
    return data


class TestSVG:
    # =====fixtures========

    def setup(self):
        pass

    # def teardown(self):
    #     pass
    #     print ("teardown-->")

    # def setup_class(cls):
    #     pass
    #     print ("\n")
    #     print ("setup_class=========>")

    # def teardown_class(cls):
    #     pass
    #     print ("teardown_class=========>")

    # def setup_method(self, method):
    #     pass
    #     print ("setup_method-->>")

    # def teardown_method(self, method):
    #     pass
    #     print ("teardown_method-->>")

    # =====test========

    def test_rect(self):
        s = """<?xml version="1.0" encoding="UTF-8" standalone="no"?>
        <svg id="Layer_3" xmlns="http://www.w3.org/2000/svg" xml:space="preserve" height="550.45" width="627.77" version="1.0" viewBox="0 0 627.769 550.45">
        <rect id="rect"  x="413.94" y="430.79" width="30.747" height="51"/>
        </svg>
        """
        root = ET.fromstring(s)
        node = root.findall('*')[0]
        coordinates = SVGParser.rect(node)

    def test_circle(self):
        s = """<?xml version="1.0" encoding="UTF-8" standalone="no"?>
        <svg id="Layer_3" xmlns="http://www.w3.org/2000/svg" xml:space="preserve" height="550.45" width="627.77" version="1.0" viewBox="0 0 627.769 550.45">
        <circle id="circle2237" cy="430.79" cx="313.94" r="30.747"/>
        </svg>
        """
        root = ET.fromstring(s)
        node = root.findall('*')[0]
        coordinates = SVGParser.circle(node)

    def test_ellipse(self):
        s = """<?xml version="1.0" encoding="UTF-8" standalone="no"?>
        <svg id="Layer_3" xmlns="http://www.w3.org/2000/svg" xml:space="preserve" height="550.45" width="627.77" version="1.0" viewBox="0 0 627.769 550.45">
        <ellipse cx="240" cy="50" rx="220" ry="30" style="fill:yellow" />
        </svg>
        """
        root = ET.fromstring(s)
        node = root.findall('*')[0]
        coordinates = SVGParser.ellipse(node)

    def test_polygon(self):
        s = """<?xml version="1.0" encoding="UTF-8" standalone="no"?>
        <svg id="Layer_3" xmlns="http://www.w3.org/2000/svg" xml:space="preserve" height="550.45" width="627.77" version="1.0" viewBox="0 0 627.769 550.45">
        <polygon id="polygon2233" points="93.977 482.88 533.9 482.88 313.94 101.89" fill="#fff"/>
        </svg>
        """
        root = ET.fromstring(s)
        node = root.findall('*')[0]
        coordinates = SVGParser.polygon(node)

    def test_path(self):
        s = """<?xml version="1.0" encoding="UTF-8" standalone="no"?>
        <svg id="Layer_3" xmlns="http://www.w3.org/2000/svg" xml:space="preserve" height="550.45" width="627.77" version="1.0" viewBox="0 0 627.769 550.45">
        <path id="path2235" d="m291.87 343.36c1.21 11.49 3.21 20.04 6.02 25.66 2.81 5.63 7.82 8.43 15.04 8.43h2.01c7.22 0 12.24-2.8 15.04-8.43 2.81-5.62 4.82-14.17 6.02-25.66l6.42-88.75c1.21-17.3 1.81-29.71 1.81-37.25 0-10.25-2.91-18.25-8.73-23.99-5.53-5.46-13.38-8.59-21.56-8.59s-16.04 3.13-21.57 8.59c-5.81 5.74-8.72 13.74-8.72 23.99 0 7.54 0.6 19.95 1.8 37.25l6.42 88.75z"/>
        </svg>
        """
        root = ET.fromstring(s)
        node = root.findall('*')[0]
        coordinates = SVGParser.path(node)

    def test_line(self):
        s = """<?xml version="1.0" encoding="UTF-8" standalone="no"?>
        <svg id="Layer_3" xmlns="http://www.w3.org/2000/svg" xml:space="preserve" height="550.45" width="627.77" version="1.0" viewBox="0 0 627.769 550.45">
        <line x1="0" y1="0" x2="200" y2="200" style="stroke:rgb(255,0,0);stroke-width:2" />
        </svg>
        """
        root = ET.fromstring(s)
        node = root.findall('*')[0]
        coordinates = SVGParser.line(node)

    def test_polyline(self):
        s = """<?xml version="1.0" encoding="UTF-8" standalone="no"?>
        <svg id="Layer_3" xmlns="http://www.w3.org/2000/svg" xml:space="preserve" height="550.45" width="627.77" version="1.0" viewBox="0 0 627.769 550.45">
        <polyline points="20,20 40,25 60,40 80,120 120,140 200,180" style="fill:none;stroke:black;stroke-width:3" />
        </svg>
        """
        root = ET.fromstring(s)
        node = root.findall('*')[0]
        coordinates = SVGParser.polyline(node)

    def test_warning(self):
        testcase = [[
            """<?xml version="1.0" encoding="utf-8"?>
            <!-- Generator: Adobe Illustrator 19.2.0, SVG Export Plug-In . SVG Version: 6.00 Build 0)  -->
            <svg version="1.1"  xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" x="0px" y="0px"
                 viewBox="0 0 595.3 841.9" style="enable-background:new 0 0 595.3 841.9;" xml:space="preserve">
            <text transform="matrix(1 0 0 1 139 286.9536)" class="st0 st1">Carpenter</text>
            </svg>""", ['TEXT_TAG', 'EMPTY']],

            ["""<?xml version="1.0" encoding="utf-8"?>
            <!-- Generator: Adobe Illustrator 19.2.0, SVG Export Plug-In . SVG Version: 6.00 Build 0)  -->
            <svg version="1.1"  xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" x="0px" y="0px"
                 viewBox="0 0 595.3 841.9" style="enable-background:new 0 0 595.3 841.9;" xml:space="preserve">
            </svg>""", ['EMPTY']],
            ["""<?xml version="1.0" encoding="utf-8"?>
            <!-- Generator: Adobe Illustrator 19.2.0, SVG Export Plug-In . SVG Version: 6.00 Build 0)  -->
            <svg version="1.1"  xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" x="0px" y="0px"
                 viewBox="0 0 595.3 841.9" style="enable-background:new 0 0 595.3 841.9;" xml:space="preserve">
            <text transform="matrix(1 0 0 1 139 286.9536)" class="st0 st1">Carpenter</text>
            <rect fill="none" width="60" height="60" x="0" y="0" stroke="#000" stroke-width="2"  />
            </svg>
            """, ['TEXT_TAG']],
            ["""<?xml version="1.0" encoding="utf-8"?>
            <!-- Generator: Adobe Illustrator 19.2.0, SVG Export Plug-In . SVG Version: 6.00 Build 0)  -->
            <svg version="1.1"  xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink"
                 viewBox="0 0 595.3 841.9" style="enable-background:new 0 0 595.3 841.9;" xml:space="preserve">
            <defs>
                <g id="g1">
                      <rect id="rect1" width="100" height="50" x="10" y="10" fill="#c00"/>
                      <circle id="circle1" cx="30" cy="30" r="10" fill="#00c"/>
                </g>
            </defs>
            </svg>
            """, ['DEFS_TAG', 'EMPTY']],
            ["""<?xml version="1.0" encoding="utf-8"?>
            <!-- Generator: Adobe Illustrator 19.2.0, SVG Export Plug-In . SVG Version: 6.00 Build 0)  -->
            <svg version="1.1"  xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink"
                 viewBox="0 0 595.3 841.9" style="enable-background:new 0 0 595.3 841.9;" xml:space="preserve">
            <clipPath id="b2">
                  <use x="0" y="0" width="200" height="200" xlink:href="#a1Shape" />
                  <use x="0" y="0" width="200" height="200" xlink:href="#a2Shape" />
            </clipPath>
            </svg>
            """, ['CLIP_TAG', "EMPTY"]]
        ]

        for s, w in testcase:
            s = s.encode()
            warning, d = SVGParser.preprocess(s)
            assert set(warning) == set(w)

    def test_preprocess(self, svg_buf):
        warning, d = SVGParser.preprocess(svg_buf)
        data, w, h = d

        print(len(data), w, h)
        assert w == 618.858
        assert h == 542.706

        path_data = SVGParser.elements_to_list(ET.fromstring(data))

        parms = (120.00000000000001, 105.22916666666667, -60.33, 52.95, 59.67, -52.28, 0)
        SVGParser.process(path_data, parms, [-0.5, -0.5, 618.858, 542.706], 85)


# if __name__ == '__main__':
#     pytest.main(" test_svg.py")
# ./
# content of test_tmpdir.py
# def test_needsfiles(tmpdir):
#     print (tmpdir)
#     assert 0
