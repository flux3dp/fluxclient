# !/usr/bin/env python3
try:
    import xml.etree.cElementTree as ET
except ImportError:
    import xml.etree.ElementTree as ET
from math import sin, cos, pi
import sys

from fluxclient.laser.laser_base import LaserBase


class LaserSvg(LaserBase):
    """docstring for laser_svg"""
    def __init__(self):
        super(laser_svg, self).__init__()

    def rect(self, thing):
        'not supporot rx ,ry yet'
        gcode = []
        gcode += self.moveto(float(thing.attrib['x']), float(thing.attrib['y']))
        gcode += self.drawto(float(thing.attrib['x']) + float(thing.attrib['width']), float(thing.attrib['y']))
        gcode += self.drawto(float(thing.attrib['x']) + float(thing.attrib['width']), float(thing.attrib['y']) + float(thing.attrib['height']))
        gcode += self.drawto(float(thing.attrib['x']), float(thing.attrib['y']) + float(thing.attrib['height']))
        gcode += self.drawto(float(thing.attrib['x']), float(thing.attrib['y']))
        gcode += self.turnOff()
        return gcode

    def circle(self, thing, sample_n=100):
        gcode = []
        cx, cy, r = float(thing.attrib['cx']), float(thing.attrib['cy']), float(thing.attrib['r'])
        gcode += self.moveto(cx + r, cy)
        for i in range(sample_n + 1):
            gcode += self.drawto(cx + (r * cos(i * 2. * pi / sample_n)), cy + (r * sin(i * 2. * pi / sample_n)))
        gcode += self.turnOff()
        return gcode

    def ellipse(self, thing, sample_n=100):
        # cx = "100" cy = "75" rx="67" ry="44"
        gcode = []
        cx, cy, rx, ry = float(thing.attrib['cx']), float(thing.attrib['cy']), float(thing.attrib['rx']), float(thing.attrib['ry'])
        gcode += self.moveto(cx + rx, cy)

        for i in range(sample_n + 1):
            gcode += self.drawto(cx + (rx * cos(i * 2. * pi / sample_n)), cy + (ry * sin(i * 2. * pi / sample_n)))

        gcode += self.turnOff()
        return gcode

    def polygon(self, thing):
        gcode = []
        points = [list(map(float, i.split(','))) for i in thing.attrib['points'].split()]
        gcode += moveto(points[0][0], points[0][1])
        for p in points:
            gcode += self.drawto(p[0], p[1])
        gcode += self.drawto(points[0][0], points[0][1])
        gcode += self.turnOff()
        return gcode

    def path(self, thing):
        # print thing.attrib['d']
        gcode = []
        head = 0
        data = []
        for i in range(len(thing.attrib['d'])):  # split for each alpha
            if thing.attrib['d'][i].isalpha():
                data.append(thing.attrib['d'][head:i])
                head = i
        data.append(thing.attrib['d'][head:])

        if data[0] == '':
            data = data[1:]

        for i in range(len(data)):  # parse each alphabet command
            tmp = [data[i][0]]
            tmp += map(float, data[i][1:].replace(',', ' ').replace('-', ' -').split())
            data[i] = tmp
            # print data[i]

        if data[0][0] == 'M':  # store init x,y for future use
            x_init, y_init = data[0][1], data[0][2]
        else:
            x_init, y_init = 0., 0.

        x, y = x_init, y_init
        Z_flag = False
        for i in data:
            if i[0] == 'M':  # Move to
                x, y = i[1], i[2]
                gcode += self.moveto(x, y)
                if Z_flag:
                    x_init, y_init = x, y
                    Z_flag = False

            elif i[0] == 'm':
                x, y = x + i[1], y + i[2]
                gcode += self.moveto(x, y)
                if Z_flag:
                    x_init, y_init = x, y
                    Z_flag = False

            elif i[0] == 'L':  # Line to
                x, y = i[1], i[2]
                gcode += self.drawto(x, y)
            elif i[0] == 'l':
                x, y = x + i[1], y + i[2]
                gcode += self.drawto(x, y)

            elif i[0] == 'H':  # horizontal line to
                x = i[1]
                gcode += self.drawto(x, y)
            elif i[0] == 'h':
                x = x + i[1]
                gcode += self.drawto(x, y)

            elif i[0] == 'V':  # vertical lineto
                y = i[1]
                gcode += self.drawto(x, y)
            elif i[0] == 'v':
                y = y + i[1]
                gcode += self.drawto(x, y)

            elif i[0] == 'Z' or i[0] == 'z':  # close path
                x, y = x_init, y_init
                gcode += self.drawto(x, y)
                gcode.append(';z')
                Z_flag = True

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
                # ref
                # http://en.wikipedia.org/wiki/B%C3%A9zier_curve
                sample_n = 100
                for j in range(sample_n + 1):
                    t = j / float(sample_n)
                    t_ = 1 - t
                    tmp_x = p0[0] * (t_ ** 3) + 3 * p1[0] * t * (t_ ** 2) + 3 * p2[0] * (t ** 2) * t_ + p3[0] * (t ** 3)
                    tmp_y = p0[1] * (t_ ** 3) + 3 * p1[1] * t * (t_ ** 2) + 3 * p2[1] * (t ** 2) * t_ + p3[1] * (t ** 3)
                    gcode += self.drawto(tmp_x, tmp_y)
                x, y = p3[0], p3[1]
            return gcode

    def check_grey_scale(self, color):
        # print (color)
        return False

    def parse_svg(self, filename):
        gcode = []
        tree = ET.parse(filename)
        # should modify io part
        root = tree.getroot()
        header = root.tag[:root.tag.find('}svg') + 1]  # '{'
        debt = []  # things that should be fill in later
        for thing in root.iter():
            # print (thing.tag)
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

            if check_grey_scale(thing.attrib.get('fill')):
                debt.append(thing)

        for thing in debt:
            pass
        return "\n".join(gcode) + "\n"

if __name__ == '__main__':
    m_laser_svg = laser_svg()
    # main(sys.argv[1])
    # main('Rust.svg')
    # main('HTML5_LOGO.svg')

    # main('responsive-design.svg')
