# !/usr/bin/env python3

import sys
from math import sin, cos, pi, radians, sqrt, acos, copysign
import xml.etree.ElementTree as ET  # cElementTree is the c implement of ElementTree, much faster and memory friendly

from fluxclient.laser.laser_base import LaserBase


class LaserSvg(LaserBase):
    """docstring for laser_svg"""
    def __init__(self):
        super(LaserSvg, self).__init__()
        self.reset()

    def reset(self):
        self.svgs = {}
        self.ready_svgs = {}
        self.param = None

    def rect(self, thing):
        '''
        drawing a rectangle
        '''
        # not support rx, ry yet
        gcode = []
        x, y, w, h = float(thing.attrib['x']), float(thing.attrib['y']), float(thing.attrib['width']), float(thing.attrib['height'])

        gcode.append((x, y))
        gcode.append((x + w, y))
        gcode.append((x + w, y + h))
        gcode.append((x, y + h))
        gcode.append((x, y))
        return [gcode]

    def circle(self, thing, sample_n=100):
        '''
        drawing a circle, sample_n indicate the sample rate or the straight line
        '''
        gcode = []
        # center of circle and radius
        cx, cy, r = float(thing.attrib['cx']), float(thing.attrib['cy']), float(thing.attrib['r'])
        gcode.append((cx + r, cy))
        for i in range(sample_n + 1):
            theta = 2. * pi * i / sample_n
            gcode.append((cx + (r * cos(theta)), cy + (r * sin(theta))))
        return [gcode]

    def ellipse(self, thing, sample_n=100):
        '''
        drawing a ellipse, sample_n indicate the sample rate or the straight line
        '''
        gcode = []

        cx, cy, rx, ry = float(thing.attrib['cx']), float(thing.attrib['cy']), float(thing.attrib['rx']), float(thing.attrib['ry'])
        gcode.append((cx + rx, cy))
        # The explicit form of ellipse, reference: https://en.wikipedia.org/wiki/Ellipse
        for i in range(sample_n + 1):
            theta = 2. * pi * i / sample_n
            gcode.append((cx + (rx * cos(theta)), cy + (ry * sin(theta))))
        return [gcode]

    def polygon(self, thing):
        '''
        drawing a polygon
        '''
        gcode = []
        points = [list(map(float, i.split(','))) for i in thing.attrib['points'].split()]
        gcode.append((points[0][0], points[0][1]))
        for p in points:
            gcode.append((p[0], p[1]))
        gcode.append((points[0][0], points[0][1]))
        return [gcode]

    def path(self, thing, sample_n=100):
        '''
        reference:http://www.w3.org/TR/SVG/paths.html
        command supported: M, m, L, l, H, h, V, v, Z, z, C, c, S, s, Q, q, T, t, A, a
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
            if i[0] in 'Mm':
                if i[0] == 'M':  # absolute move to
                    x, y = i[1], i[2]
                elif i[0] == 'm':  # relative move to
                    x, y = x + i[1], y + i[2]
                gcode.append(('\n', '\n'))
                gcode.append((x, y))

                if Z_flag:  # store data if z command appear in the future
                    x_init, y_init = x, y
                    Z_flag = False
                prev_control = None

            elif i[0] in 'LlHhVv':
                if i[0] == 'L':  # absolute Line to
                    x, y = i[1], i[2]
                elif i[0] == 'l':  # relative line to
                    x, y = x + i[1], y + i[2]
                elif i[0] == 'H':  # horizontal line to
                    x = i[1]
                elif i[0] == 'h':  # relative horizontal line to
                    x = x + i[1]
                elif i[0] == 'V':  # vertical lineto
                    y = i[1]
                elif i[0] == 'v':  # relative vertical lineto
                    y = y + i[1]

                gcode.append((x, y))
                prev_control = None

            elif i[0] == 'Z' or i[0] == 'z':  # close path
                x, y = x_init, y_init
                gcode.append((x, y))
                Z_flag = True
                prev_control = None

            # reference: http://en.wikipedia.org/wiki/B%C3%A9zier_curve
            elif i[0] in 'CcSs':
                p0 = (x, y)
                if i[0] == 'C':
                    p1 = (i[1], i[2])
                    p2 = (i[3], i[4])
                    p3 = (i[5], i[6])
                elif i[0] == 'c':
                    p1 = (p0[0] + i[1], p0[1] + i[2])
                    p2 = (p0[0] + i[3], p0[1] + i[4])
                    p3 = (p0[0] + i[5], p0[1] + i[6])

                elif i[0] in 'Ss':
                    if prev_control is not None and prev_control[0] is not None:
                        p1 = 2 * x - prev_control[0][0], 2 * y - prev_control[0][1]
                    else:
                        p1 = (x, y)

                    if i[0] == 'S':
                        p2 = (i[1], i[2])
                        p3 = (i[3], i[4])
                    elif i[0] == 's':
                        p2 = (p0[0] + i[1], p0[1] + i[2])
                        p3 = (p0[0] + i[3], p0[1] + i[4])

                for j in range(sample_n + 1):
                    t = j / float(sample_n)
                    t_ = 1 - t
                    tmp_x = p0[0] * (t_ ** 3) + 3 * p1[0] * t * (t_ ** 2) + 3 * p2[0] * (t ** 2) * t_ + p3[0] * (t ** 3)
                    tmp_y = p0[1] * (t_ ** 3) + 3 * p1[1] * t * (t_ ** 2) + 3 * p2[1] * (t ** 2) * t_ + p3[1] * (t ** 3)
                    gcode.append((tmp_x, tmp_y))

                x, y = p3[0], p3[1]
                prev_control = [(p2[0], p2[1]), None]

            elif i[0] in 'QqTt':
                p0 = (x, y)
                if i[0] == 'Q':
                    p1 = (i[1], i[2])
                    p2 = (i[3], i[4])
                elif i[0] == 'q':
                    p1 = (p0[0] + i[1], p0[1] + i[2])
                    p2 = (p0[0] + i[3], p0[1] + i[4])

                elif i[0] in 'Tt':
                    if prev_control is not None and prev_control[1] is not None:
                        p1 = 2 * x - prev_control[1][0], 2 * y - prev_control[1][1]
                    else:
                        p1 = (x, y)

                    if i[0] == 'T':
                        p2 = (i[1], i[2])
                    elif i[0] == 't':
                        p2 = (p0[0] + i[1], p0[1] + i[2])

                for j in range(sample_n + 1):
                    t = j / float(sample_n)
                    t_ = 1 - t
                    tmp_x = p0[0] * (t_ ** 2) + 2 * t_ * t * p1[0] + (t ** 2) * p2[0]
                    tmp_y = p0[1] * (t_ ** 2) + 2 * t_ * t * p1[1] + (t ** 2) * p2[1]
                    gcode.append((tmp_x, tmp_y))

                x, y = p2[0], p2[1]
                prev_control = [None, (p1[0], p1[1])]
            elif i[0] in 'Aa':
                # implementation reference: http://www.w3.org/TR/SVG/implnote.html#ArcImplementationNotes

                rx, ry = abs(i[1]), abs(i[2])
                x_axis_rotation = i[3]  # degrees
                phi = radians(x_axis_rotation)
                # None zero means true
                large_arc_flag = i[4] != 0
                sweep_flag = i[5] != 0
                if i[0] == 'A':
                    end_x, end_y = i[6], i[7]
                elif i[0] == 'a':
                    end_x, end_y = x + i[6], y + i[7]

                if x == end_x and y == end_y:
                    # nothing happend
                    pass
                elif rx == 0 or ry == 0:
                    # line to end point
                    x, y = end_x, end_y
                    gcode.append((x, y))
                else:
                    # see F.6.5
                    cp = cos(phi)
                    sp = sin(phi)

                    # F.6.5.1
                    x_prime = cp * (x - end_x) / 2. + sp * (y - end_y) / 2.
                    y_prime = (-sp) * (x - end_x) / 2. + cp * (y - end_y) / 2.

                    # F.6.6.2
                    cap = (x_prime ** 2) / (rx ** 2) + (y_prime ** 2) / (ry ** 2)
                    if cap > 1:
                        rx = sqrt(cap) * rx
                        ry = sqrt(cap) * ry

                    # F.6.5.2
                    tmp = (rx ** 2 * ry ** 2 - rx ** 2 * y_prime ** 2 - ry ** 2 * x_prime ** 2) / (rx ** 2 * y_prime ** 2 + ry ** 2 * x_prime ** 2)
                    if abs(tmp) < 1e-10:  # python math overflow error
                        tmp = 0
                    tmp = sqrt(tmp)
                    if large_arc_flag == sweep_flag:
                        tmp *= -1
                    cx_prime = tmp * rx * y_prime / ry
                    cy_prime = tmp * (-1) * ry * x_prime / rx

                    # F.6.5.3
                    cx = cp * cx_prime + (-sp) * cy_prime + ((x + end_x) / 2)
                    cy = sp * cx_prime + cp * cy_prime + ((y + end_y) / 2)

                    # F.6.5.4 - F.6.5.6
                    angle_between = lambda v1, v2: copysign(acos((v1[0] * v2[0] + v1[1] * v2[1]) / sqrt(v1[0] ** 2 + v1[1] ** 2) / sqrt(v2[0] ** 2 + v2[1] ** 2)), v1[0] * v2[1] - v1[1] * v2[0])
                    tmp_v = (x_prime - cx_prime) / rx, y_prime - cy_prime / ry
                    theta_1 = angle_between((1, 0), tmp_v)
                    delta_theta = angle_between(tmp_v, ((-x_prime - cx_prime) / rx, (-y_prime - cy_prime) / ry))

                    # might be buggy!!??
                    if sweep_flag and delta_theta < 0:
                        delta_theta += 2 * pi
                    elif not sweep_flag and delta_theta > 0:
                        delta_theta -= 2 * pi

                    for j in range(sample_n + 1):
                        t = j / float(sample_n)
                        theta = theta_1 + t * delta_theta

                        x = cp * rx * cos(theta) + (-sp) * ry * sin(theta) + cx
                        y = sp * rx * cos(theta) + cp * ry * sin(theta) + cy
                        gcode.append((x, y))

                x, y = end_x, end_y
                gcode.append((x, y))
                prev_control = None
            else:
                raise ValueError('Undefine path command \'%s\'' % (i[0]))
        return [gcode]

    def add_image(self, svg_data):

        # may appear different language when readin a file in binary mode
        # and be careful about the '\n', '\r\n' issue
        svg_data = svg_data.decode('utf8')
        root = ET.fromstring(svg_data)
        self.svgs.append(root)

    def elements_to_list(self, svg):
        """
        return list-in-list indicate the coordinate the laser should go through
        each element in one list
        in each list '\n' means it's a move to command not a line to
        """
        # find the tag-header for each child element
        # because will look like this when parsing the data -> '{http://www.w3.org/2000/svg}polygon'
        # or should i use if [polygon, circle, ... ] in thing.tag instead?
        header = svg.tag[:svg.tag.find('}svg') + 1]
        gcode = []
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
        return gcode

    def compute(self, buf, name, params):
        self.ready_svgs[name] = [buf] + params

    def preprocess(self, buf, name):
        """
        preprocess the svg file
        make it only with black storke and element's frame
        and compute smallest viewBox
        """
        # note that there may be different language when read in a file in binary mode
        # and be careful about the '\n', '\r\n' issue
        svg_data = buf.decode('utf8')
        root = ET.fromstring(svg_data)
        tree = ET.ElementTree()
        tree._setroot(root)
        newtree = ET.ElementTree()
        header = root.tag[:root.tag.find('}svg') + 1]

        for i in range(len(root)):
            thing = root[i]
            tmp_thing = ET.Element(thing.tag)
            # tmp_thing.tag = thing.tag
            tmp_thing.attrib['fill'] = 'None'
            tmp_thing.attrib['stroke'] = '#000000'
            tmp_thing.tail = '\n'
            if thing.tag == header + 'rect':
                tmp_thing.attrib['x'] = thing.attrib['x']
                tmp_thing.attrib['y'] = thing.attrib['y']
                tmp_thing.attrib['height'] = thing.attrib['height']
                tmp_thing.attrib['width'] = thing.attrib['width']
                root[i] = tmp_thing
            elif thing.tag == header + 'circle':
                tmp_thing.attrib['cx'] = thing.attrib['cx']
                tmp_thing.attrib['cy'] = thing.attrib['cy']
                tmp_thing.attrib['r'] = thing.attrib['r']
                root[i] = tmp_thing
            elif thing.tag == header + 'ellipse':
                tmp_thing.attrib['cx'] = thing.attrib['cx']
                tmp_thing.attrib['cy'] = thing.attrib['cy']
                tmp_thing.attrib['rx'] = thing.attrib['rx']
                tmp_thing.attrib['ry'] = thing.attrib['ry']
                root[i] = tmp_thing
            elif thing.tag == header + 'polygon':
                tmp_thing.attrib['points'] = thing.attrib['points']
                root[i] = tmp_thing
            elif thing.tag == header + 'path':
                tmp_thing.attrib['d'] = thing.attrib['d']
                root[i] = tmp_thing

        path_data = self.elements_to_list(root)

        min_x, min_y, max_x, max_y = float('inf'), float('inf'), 0., 0.
        for path in path_data:
            for x, y in path:
                if x != '\n':  # moving command
                    if x < min_x:
                        min_x = x
                    if x > max_x:
                        max_x = x
                    if y < min_y:
                        min_y = y
                    if y > max_y:
                        max_y = y
                else:
                    pass

        viewBox = root.attrib.get('viewBox', (0, 0, float('inf'), float('inf')))
        if type(viewBox) == str:
            viewBox = list(map(float, viewBox.replace(',', ' ').split()))
        viewBox = [max(viewBox[0], min_x), max(viewBox[1], min_y), min(viewBox[2], max_x), min(viewBox[3], max_y)]
        viewBox[2] = viewBox[2] - viewBox[0]
        viewBox[3] = viewBox[3] - viewBox[1]
        root.attrib['viewBox'] = " ".join(map(str, viewBox))
        # theese are optional
        root.attrib['width'] = str(viewBox[2])
        root.attrib['height'] = str(viewBox[3])
        root.attrib['style'] = "border:1px solid #ff0000;"

        self.svgs[name] = ET.tostring(root)  # type: bytes
        # tree.write('preprocess.svg')

    def process(self, path_data, params, viewBox):
        """
        actually compute the path that should appear on object
        including delete things out of viewBox
        scale, locate and rotate all the points
        """
        # viewBox: put all the points in viewBox
        # scale: go through and find x, y
        # transform: rotate the points
        w, h, x1_real, y1_real, x2_real, y2_real, rotation = params
        dis = lambda x, y: (x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2
        for path in range(len(path_data)):
            new_path = []
            for p in range(len(path_data[path]) - 1):
                x1, y1 = path_data[path][p]
                x2, y2 = path_data[path][p + 1]
                if x1 == '\n' or x2 == '\n':
                    new_path.append([x1, y1])
                    new_path.append([x2, y2])
                    continue
                out = 0
                if x1 < viewBox[0] or x1 > viewBox[0] + viewBox[2] or y1 < viewBox[1] or y1 > viewBox[1] + viewBox[3]:
                    out += 1
                if x2 < viewBox[0] or x2 > viewBox[0] + viewBox[2] or y2 < viewBox[1] or y2 > viewBox[1] + viewBox[3]:
                    out += 2

                if out == 0:
                    new_path.append([x1, y1])
                    new_path.append([x2, y2])
                else:
                    if y1 == y2:
                        # horizontal line
                        x_candidate = sorted([x1, x2, viewBox[0], viewBox[0] + viewBox[2]])
                        if x1 < x2:
                            new_path.append([x_candidate[1], y1])
                            new_path.append([x_candidate[2], y1])
                        else:
                            new_path.append([x_candidate[2], y1])
                            new_path.append([x_candidate[1], y1])

                        pass
                    elif x1 == x2:
                        # vertical line
                        y_candidate = sorted([y1, y2, viewBox[1], viewBox[1] + viewBox[3]])
                        if y1 < y2:
                            new_path.append([x1, y_candidate[1]])
                            new_path.append([x1, y_candidate[2]])
                        else:
                            new_path.append([x1, y_candidate[2]])
                            new_path.append([x1, y_candidate[1]])

                    # y = ax + b
                    a = (y1 - y2) / (x1 - x2)
                    b = y1 - (a * x1)

                    candidate = []
                    if a * x1 + b > viewBox[0] and a * x1 + b < viewBox[0] + viewBox[2]:
                        candidate.append([x1, a * x1 + b])
                    if a * x2 + b > viewBox[0] and a * x2 + b < viewBox[0] + viewBox[2]:
                        candidate.append([x2, a * x2 + b])
                    if (y1 - b) / a > viewBox[1] and (y1 - b) / a > viewBox[1] + viewBox[3]:
                        candidate.append([(y1 - b) / a, y1])
                    if (y2 - b) / a > viewBox[1] and (y2 - b) / a > viewBox[1] + viewBox[3]:
                        candidate.append([(y2 - b) / a, y2])
                    if len(candidate) == 1:
                        # one cross point, so need to find out which point
                        if out == 1:
                            new_path.append(['\n', '\n'])
                            new_path.append(candidate[0])
                            new_path.append([x2, y2])
                        elif out == 2:
                            new_path.append([x1, y1])
                            new_path.append(candidate[0])
                            new_path.append(['\n', '\n'])

                    elif len(candidate) == 2:
                        new_path.append(['\n', '\n'])
                        if dis(candidate[0], (x1, y1)) < dis(candidate[0], (x2, y2)):
                            new_path.append(candidate[0])
                            new_path.append(candidate[1])
                        else:  # needless because it's not gonna happene?
                            new_path.append(candidate[1])
                            new_path.append(candidate[0])

                    elif len(candidate) >= 3:
                        # a line through a squre can only have less than 2 crossover
                        # this statement could only be true when they meet in corner
                        new_path.append(['\n', '\n'])
                        M = max(dis(candidate[0], candidate[1]), dis(candidate[0], candidate[2]), dis(candidate[1], candidate[2]))
                        if dis(candidate[0], candidate[1]) == M:
                            new_path.append(candidate[0])
                            new_path.append(candidate[1])
                        elif dis(candidate[0], candidate[2]) == M:
                            new_path.append(candidate[0])
                            new_path.append(candidate[2])
                        elif dis(candidate[1], candidate[2]) == M:
                            new_path.append(candidate[1])
                            new_path.append(candidate[2])

            # delete redundant points
            tmp_new_path = [new_path[0]]
            for i in new_path:
                if tmp_new_path[-1] != i:
                    tmp_new_path.append(i)
            new_path = tmp_new_path

            # transformation
            vx = [w, 0]
            vx = [(vx[0] * cos(rotation) - vx[1] * sin(rotation)), (vx[0] * sin(rotation) + vx[1] * cos(rotation))]

            vy = [0, -h]
            vy = [(vy[0] * cos(rotation) - vy[1] * sin(rotation)), (vy[0] * sin(rotation) + vy[1] * cos(rotation))]

            for i in range(len(new_path)):
                if new_path[i][0] != '\n':
                    new_path[i][0] -= viewBox[0]
                    new_path[i][1] -= viewBox[1]

                    new_path[i][0] /= viewBox[2]
                    new_path[i][1] /= viewBox[3]

                    x = x1_real + new_path[i][0] * vx[0] + new_path[i][1] * vy[0]
                    y = y1_real + new_path[i][0] * vx[1] + new_path[i][1] * vy[1]
                    new_path[i] = [x, y]
                else:
                    pass
            path_data[path] = new_path
        return path_data

    def gcode_generate(self):
        gcode = []
        gcode += self.header('laser svg')

        for i in self.ready_svgs.values():  # might use 'name' as key instead in the future?
            svg_data = i[0].decode('utf8')
            root = ET.fromstring(svg_data)
            viewBox = list(map(float, root.attrib['viewBox'].replace(',', ' ').split()))

            path_data = self.elements_to_list(root)

            path_data = self.process(path_data, i[1:], viewBox)

            # TODO: y = -y
            for each_path in path_data:
                # move to the first place
                moveTo = True
                for x, y in each_path:
                    if x != '\n':
                        if not moveTo:
                            gcode += self.drawTo(x, y)
                        else:
                            gcode += self.moveTo(x, y)
                            moveTo = False
                    else:
                        moveTo = True
                        continue

        ################ fake code ##############
        tmp = []
        for i in gcode:
            if i[:2] == 'G1':
                tmp.append(i)
        return "\n".join(tmp) + "\n"
        ##########################################

        return "\n".join(gcode) + "\n"

if __name__ == '__main__':
    m_laser_svg = LaserSvg()

    # filename = ('Rust.svg')
    # filename = ('HTML5_LOGO.svg')
    # filename = ('responsive-design.svg')
    filename = sys.argv[1]
    with open(filename, 'rb') as f:
        m_laser_svg.preprocess(f.read(), filename)
        # print (m_laser_svg.gcode_generate())
