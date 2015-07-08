# !/usr/bin/env python3

import sys
from math import sin, cos, pi
import xml.etree.ElementTree as ET  # cElementTree is the c implement of ElementTree, much faster and memory friendly

from fluxclient.laser.laser_base import LaserBase


class LaserSvg(LaserBase):
    """docstring for laser_svg"""
    def __init__(self):
        super(LaserSvg, self).__init__()
        self.reset()

    def reset(self):
        self.svgs = []
        self.param = None

    def rect(self, thing):
        '''
        drawing a rectangle
        '''
        # not supporot rx ,ry yet
        gcode = []
        x, y, w, h = float(thing.attrib['x']), float(thing.attrib['y']), float(thing.attrib['width']), float(thing.attrib['height'])
        gcode += self.moveTo(x, y)
        gcode += self.drawTo(x + w, y)
        gcode += self.drawTo(x + w, y + h)
        gcode += self.drawTo(x, y + h)
        gcode += self.drawTo(x, y)
        gcode += self.turnOff()
        return gcode

    def circle(self, thing, sample_n=100):
        '''
        drawing a circle, sample_n indicate the sample rate or the straight line
        '''
        gcode = []
        # center of circle and radius
        cx, cy, r = float(thing.attrib['cx']), float(thing.attrib['cy']), float(thing.attrib['r'])
        gcode += self.moveTo(cx + r, cy)
        for i in range(sample_n + 1):
            theta = 2. * pi * i / sample_n
            gcode += self.drawTo(cx + (r * cos(theta)), cy + (r * sin(theta)))
        gcode += self.turnOff()
        return gcode

    def ellipse(self, thing, sample_n=100):
        '''
        drawing a ellipse, sample_n indicate the sample rate or the straight line
        '''
        gcode = []
        cx, cy, rx, ry = float(thing.attrib['cx']), float(thing.attrib['cy']), float(thing.attrib['rx']), float(thing.attrib['ry'])
        gcode += self.moveTo(cx + rx, cy)
        # The explicit form, reference: https://en.wikipedia.org/wiki/Ellipse
        for i in range(sample_n + 1):
            theta = 2. * pi * i / sample_n
            gcode += self.drawTo(cx + (rx * cos(theta)), cy + (ry * sin(theta)))

        gcode += self.turnOff()
        return gcode

    def polygon(self, thing):
        '''
        drawing a polygon
        '''
        gcode = []
        points = [list(map(float, i.split(','))) for i in thing.attrib['points'].split()]
        gcode += self.moveTo(points[0][0], points[0][1])
        for p in points:
            gcode += self.drawTo(p[0], p[1])
        gcode += self.drawTo(points[0][0], points[0][1])
        gcode += self.turnOff()
        return gcode

    def path(self, thing, sample_n=100):
        '''
        reference:http://www.w3.org/TR/SVG/paths.html
        '''
        gcode = []
        data = []
        head = 0
        # split the path by alphabet
        for i in range(len(thing.attrib['d'])):
            if thing.attrib['d'][i].isalpha():
                data.append(thing.attrib['d'][head:i])
                head = i
        data.append(thing.attrib['d'][head:])

        # clean up the leading space
        if data[0].strip() == '':
            data = data[1:]

        for i in range(len(data)):  # parse each alphabet command
            tmp = [data[i][0]]
            tmp += map(float, data[i][1:].replace(',', ' ').replace('-', ' -').split())
            data[i] = tmp

        for i in data:
            print(i)
            pass

        if data[0][0] == 'M':  # store init x,y for future use
            x_init, y_init = data[0][1], data[0][2]
        else:
            x_init, y_init = 0., 0.

        # variable that use for 'Z' command aka 'close path'
        x, y = x_init, y_init
        Z_flag = False

        x, y = None, None  # current x, y position
        prev_control = None
        for i in data:
            # if i[0] == 'z' or i[0] == 'Z':
            #     prev_control = None
            #     break
            if i[0] == 'M':  # Move to
                x, y = i[1], i[2]
                gcode += self.moveTo(x, y)
                if Z_flag:
                    x_init, y_init = x, y
                    Z_flag = False
                prev_control = None
            elif i[0] == 'm':  # relative move to
                x, y = x + i[1], y + i[2]
                gcode += self.moveTo(x, y)
                if Z_flag:
                    x_init, y_init = x, y
                    Z_flag = False
                prev_control = None
            elif i[0] == 'L':  # Line to
                x, y = i[1], i[2]
                gcode += self.drawTo(x, y)
                prev_control = None
            elif i[0] == 'l':  # relative line to
                x, y = x + i[1], y + i[2]
                gcode += self.drawTo(x, y)
                prev_control = None

            elif i[0] == 'H':  # horizontal line to
                x = i[1]
                gcode += self.drawTo(x, y)
                prev_control = None
            elif i[0] == 'h':  # relative horizontal line to
                x = x + i[1]
                gcode += self.drawTo(x, y)
                prev_control = None

            elif i[0] == 'V':  # vertical lineto
                y = i[1]
                gcode += self.drawTo(x, y)
                prev_control = None
            elif i[0] == 'v':  # relative vertical lineto
                y = y + i[1]
                gcode += self.drawTo(x, y)
                prev_control = None

            elif i[0] == 'Z' or i[0] == 'z':  # close path
                x, y = x_init, y_init
                gcode += self.drawTo(x, y)
                gcode.append(';z')
                Z_flag = True
                prev_control = None

            elif i[0] == 'C' or i[0] == 'c':
                p0 = (x, y)
                if i[0] == 'C':
                    p1 = (i[1], i[2])
                    p2 = (i[3], i[4])
                    p3 = (i[5], i[6])
                elif i[0] == 'c':
                    p1 = (p0[0] + i[1], p0[1] + i[2])
                    p2 = (p0[0] + i[3], p0[1] + i[4])
                    p3 = (p0[0] + i[5], p0[1] + i[6])

                # reference: http://en.wikipedia.org/wiki/B%C3%A9zier_curve
                for j in range(sample_n + 1):
                    t = j / float(sample_n)
                    t_ = 1 - t
                    tmp_x = p0[0] * (t_ ** 3) + 3 * p1[0] * t * (t_ ** 2) + 3 * p2[0] * (t ** 2) * t_ + p3[0] * (t ** 3)
                    tmp_y = p0[1] * (t_ ** 3) + 3 * p1[1] * t * (t_ ** 2) + 3 * p2[1] * (t ** 2) * t_ + p3[1] * (t ** 3)
                    gcode += self.drawTo(tmp_x, tmp_y)

                x, y = p3[0], p3[1]
                prev_control = (p2[0], p2[1])

            elif i[0] == 'S' or i[0] == 's':
                p0 = (x, y)
                if prev_control is not None:
                    p1 = 2 * x - prev_control[0], 2 * y - prev_control[1]
                else:
                    p1 = (x, y)
                if i[0] == 'S':
                    p2 = (i[1], i[2])
                    p3 = (i[3], i[4])
                    # p3 = (i[5], i[6])
                elif i[0] == 'S':
                    p2 = (p0[0] + i[1], p0[1] + i[2])
                    p3 = (p0[0] + i[3], p0[1] + i[4])
                    # p3 = (p0[0] + i[5], p0[1] + i[6])
                for j in range(sample_n + 1):
                    t = j / float(sample_n)
                    t_ = 1 - t
                    tmp_x = p0[0] * (t_ ** 3) + 3 * p1[0] * t * (t_ ** 2) + 3 * p2[0] * (t ** 2) * t_ + p3[0] * (t ** 3)
                    tmp_y = p0[1] * (t_ ** 3) + 3 * p1[1] * t * (t_ ** 2) + 3 * p2[1] * (t ** 2) * t_ + p3[1] * (t ** 3)
                    gcode += self.drawTo(tmp_x, tmp_y)

                x, y = p3[0], p3[1]
                prev_control = (p2[0], p2[1])

            else:
                raise ValueError('Undefine path command \'%s\'' % (i[0]))
        return gcode

    def check_grey_scale(self, color):
        return False

    def add_image(self, svg_data):
        # may appear different language when readin a file in binary mode
        # and be careful about the '\n', '\r\n' issue
        svg_data = svg_data.decode('utf8')
        root = ET.fromstring(svg_data)
        self.svgs.append(root)

    def gcode_generate(self):
        gcode = []
        gcode += self.header('laser svg')
        for svg in self.svgs:
            # find the tag-header for each child element
            # because will look like this when parsing the data -> '{http://www.w3.org/2000yo/svg}polygon'
            # or should i use if [polygon, circle, ... ] in thing.tag instead?
            header = svg.tag[:svg.tag.find('}svg') + 1]

            debt = []  # things that should be fill in later, not support yet~
            for thing in svg.iter():
                # deal with different svg graph
                if thing.tag == header + 'rect':
                    gcode += self.rect(thing)
                elif thing.tag == header + 'circle':
                    gcode += self.circle(thing)
                elif thing.tag == header + 'ellipse':
                    gcode += self.ellipse(thing)
                elif thing.tag == header + 'polygon':
                    gcode += self.polygon(thing)
                elif thing.tag == header + 'path':
                    gcode += self.path(thing)

                if self.check_grey_scale(thing.attrib.get('fill')):  # things that should be fill in later, not support yet~
                    debt.append(thing)

            for thing in debt:
                pass
        tmp = []
        for i in gcode:
            if i[:2] == 'G1':
                tmp.append(i)
        return "\n".join(tmp) + "\n"
        return "\n".join(gcode) + "\n"

if __name__ == '__main__':
    m_laser_svg = LaserSvg()

    # filename = ('Rust.svg')
    # filename = ('HTML5_LOGO.svg')
    # filename = ('responsive-design.svg')
    filename = sys.argv[1]
    with open(filename, 'rb') as f:
        m_laser_svg.add_image(f.read())
        print (m_laser_svg.gcode_generate())
