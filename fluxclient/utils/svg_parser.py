import xml.etree.ElementTree as ET
from re import split
from os import environ
from math import sin, cos, pi, radians, sqrt, acos, copysign


class SVGParser(object):
    """docstring for SVGParser"""
    def __init__(self):
        super(SVGParser, self).__init__()

    @staticmethod
    def rect(node):
        '''
        drawing a rectangle
        '''
        # not support rx, ry yet
        coordinate = []
        x, y, w, h = float(node.attrib['x']), float(node.attrib['y']), float(node.attrib['width']), float(node.attrib['height'])

        coordinate.append((x, y))
        coordinate.append((x + w, y))
        coordinate.append((x + w, y + h))
        coordinate.append((x, y + h))
        coordinate.append((x, y))
        return [coordinate]

    @staticmethod
    def line(node):
        '''
        drawing a Line
        '''
        coordinate = []
        x1, y1, x2, y2 = float(node.attrib['x1']), float(node.attrib['y1']), float(node.attrib['x2']), float(node.attrib['y2'])

        coordinate.append((x1, y1))
        coordinate.append((x2, y2))
        return [coordinate]

    @staticmethod
    def polyline(node):
        '''
        drawing a polyline
        '''
        coordinate = []
        points = split('[^0-9.-]+', node.attrib['points'])  # split into numbers
        points = filter(lambda x: x != "", points)
        points = list(map(float, points))

        if len(points) % 2 == 1:  # odd number of coordinate shouldn't be here
            points.pop()

        for i in range(len(points) // 2):
            coordinate.append((points[i * 2], points[i * 2 + 1]))
        return [coordinate]

    @staticmethod
    def circle(node, sample_n=100):
        '''
        drawing a circle, sample_n indicate the sample rate or the straight line
        '''
        coordinate = []
        # center of circle and radius
        cx, cy, r = float(node.attrib['cx']), float(node.attrib['cy']), float(node.attrib['r'])
        coordinate.append((cx + r, cy))
        for i in range(sample_n + 1):
            theta = 2. * pi * i / sample_n
            coordinate.append((cx + (r * cos(theta)), cy + (r * sin(theta))))
        return [coordinate]

    @staticmethod
    def ellipse(node, sample_n=100):
        '''
        drawing a ellipse, sample_n indicate the sample rate or the straight line
        '''
        coordinate = []

        cx, cy, rx, ry = float(node.attrib['cx']), float(node.attrib['cy']), float(node.attrib['rx']), float(node.attrib['ry'])
        coordinate.append((cx + rx, cy))
        # The explicit form of ellipse, reference: https://en.wikipedia.org/wiki/Ellipse
        for i in range(sample_n + 1):
            theta = 2. * pi * i / sample_n
            coordinate.append((cx + (rx * cos(theta)), cy + (ry * sin(theta))))
        return [coordinate]

    @staticmethod
    def polygon(node):
        '''
        drawing a polygon
        '''
        coordinate = []
        points = split('[^0-9.-]+', node.attrib['points'])  # split into numbers
        points = filter(lambda x: x != "", points)
        points = list(map(float, points))

        if len(points) % 2 == 1:  # odd number of coordinate shouldn't be here
            points.pop()

        if len(points) >= 6:  # polygon need more than 3 points
            for i in range(len(points) // 2):
                coordinate.append((points[i * 2], points[i * 2 + 1]))
            coordinate.append((points[0], points[1]))  # go back to the head
        return [coordinate]

    @staticmethod
    def path(node, sample_n=100):
        '''
        reference:http://www.w3.org/TR/SVG/paths.html
        command supported: M, m, L, l, H, h, V, v, Z, z, C, c, S, s, Q, q, T, t, A, a
        '''
        def angle_between(v1, v2):
            angle = (v1[0] * v2[0] + v1[1] * v2[1]) / sqrt(v1[0] ** 2 + v1[1] ** 2) / sqrt(v2[0] ** 2 + v2[1] ** 2)
            if abs(angle) - 1 < 1e-10:  # precision problem will cause error on Domain of a function
                angle = round(angle)
            return copysign(acos(angle), v1[0] * v2[1] - v1[1] * v2[0])

        coordinate = []
        data = []
        head = 0
        # split the path by alphabet
        for i in range(len(node.attrib['d'])):
            if node.attrib['d'][i].isalpha():
                data.append(node.attrib['d'][head:i])
                head = i
        data.append(node.attrib['d'][head:])

        # clean up the leading space
        if data[0].strip() == '':
            data = data[1:]

        # parse each alphabet command
        for i in range(len(data)):
            tmp = [data[i][0]]
            tmp += map(float, data[i][1:].replace(',', ' ').replace('-', ' -').split())
            data[i] = tmp

        if len(data) == 0:
            return [[]]

        # there can be several set of parameter using same command ex: L12,34,56,78 -> L12,34 L56,78
        parameter_list = {'M': 2, 'm': 2, 'L': 2, 'l': 2, 'H': 1, 'h': 1, 'V': 1, 'v': 1, 'Z': 0, 'z': 0, 'C': 6, 'c': 6, 'S': 4, 's': 4, 'Q': 4, 'q': 4, 'T': 2, 't': 2, 'A': 7, 'a': 7}
        tmp = []
        for i in range(len(data)):
            l = len(data[i]) - 1
            if l == parameter_list[data[i][0]]:
                tmp.append(data[i])
            else:
                counter = 1
                while counter < l:
                    tmp.append([data[i][0]] + data[i][counter:counter + parameter_list[data[i][0]]])
                    counter += parameter_list[data[i][0]]
        data = tmp

        if data[0][0] == 'M':  # store init x,y for future use
            x_init, y_init = data[0][1], data[0][2]
        else:
            x_init, y_init = 0., 0.

        # variable that use for 'Z' command aka 'close path'
        Z_flag = True

        x, y = None, None  # current x, y position
        prev_control = None  # some command need to use previous points as control point

        for i in data:
            if i[0] in 'Mm':

                if i[0] == 'M':  # absolute move to
                    x, y = i[1], i[2]
                elif i[0] == 'm':  # relative move to
                    if x is None and y is None:
                        x, y = i[1], i[2]
                    else:
                        x, y = x + i[1], y + i[2]
                coordinate.append(('\n', '\n'))
                coordinate.append((x, y))

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

                coordinate.append((x, y))
                prev_control = None

            elif i[0] == 'Z' or i[0] == 'z':  # close path
                x, y = x_init, y_init
                coordinate.append((x, y))
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
                    coordinate.append((tmp_x, tmp_y))

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
                    coordinate.append((tmp_x, tmp_y))

                x, y = p2[0], p2[1]
                prev_control = [None, (p1[0], p1[1])]
            elif i[0] in 'Aa':
                # implementation reference: http://www.w3.org/TR/SVG/implnote.html#ArcImplementationNotes
                rx, ry = abs(i[1]), abs(i[2])
                phi = radians(i[3])
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
                    coordinate.append((x, y))
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
                    tmp_v = ((x_prime - cx_prime) / rx, (y_prime - cy_prime) / ry)
                    theta_1 = angle_between((1, 0), tmp_v)
                    delta_theta = angle_between(tmp_v, ((-x_prime - cx_prime) / rx, (-y_prime - cy_prime) / ry))
                    # might be buggy!!??
                    if not sweep_flag and delta_theta > 0:
                        delta_theta -= 2 * pi
                    elif sweep_flag and delta_theta < 0:
                        delta_theta += 2 * pi

                    for j in range(sample_n + 1):
                        t = j / float(sample_n)
                        theta = theta_1 + t * delta_theta

                        x = cp * rx * cos(theta) + (-sp) * ry * sin(theta) + cx
                        y = sp * rx * cos(theta) + cp * ry * sin(theta) + cy
                        coordinate.append((x, y))

                x, y = end_x, end_y
                coordinate.append((x, y))
                prev_control = None
            else:
                raise ValueError('Undefine path command \'%s\'' % (i[0]))
        return [coordinate]

    @staticmethod
    def elements_to_list(svg):
        """
        [svg]: root generate from ET.fromstring()
        parse svg(readin as root, from ElementTree)
        return list-in-list indicating the coordinate the laser should go through
        each element in one list
        in each list, char '\n' means that's a move-to command not a line-to
        """
        # find the tag-header for each child element
        # because it will look like this when parsing the data -> '{http://www.w3.org/2000/svg}polygon'
        # or should i use if [polygon, circle, ... ] in node.tag instead?
        header = svg.tag[:svg.tag.find('}svg') + 1]
        coordinates = []
        for node in svg.iter():
            # deal with different svg graph
            if node.tag == header + 'rect':
                coordinates += SVGParser.rect(node)
            elif node.tag == header + 'circle':
                coordinates += SVGParser.circle(node)
            elif node.tag == header + 'ellipse':
                coordinates += SVGParser.ellipse(node)
            elif node.tag == header + 'polygon':
                coordinates += SVGParser.polygon(node)
            elif node.tag == header + 'path':
                coordinates += SVGParser.path(node)
            elif node.tag == header + 'line':
                coordinates += SVGParser.line(node)
            elif node.tag == header + 'polyline':
                coordinates += SVGParser.polyline(node)
        return coordinates

    @staticmethod
    def preprocess(buf):
        """
        [buf]
        preprocess the svg file
        make it only with black storke and elements' frame
        and compute smallest viewBox
        """
        # note that there may be different language when read in a file in binary mode
        # and be careful dealing with '\n', '\r\n' issue
        warning = []
        root = ET.fromstring(buf.decode('utf8'))
        tree = ET.ElementTree()
        tree._setroot(root)
        header = root.tag[:root.tag.find('}svg') + 1]

        for i in root.iter():
            thing = i
            tmp_thing = ET.Element(thing.tag)
            # tmp_thing.tag = thing.tag
            tmp_thing.attrib['fill'] = 'None'
            tmp_thing.attrib['stroke'] = '#000000'
            tmp_thing.tail = '\n'
            if thing.tag == header + 'rect':
                tmp_thing.attrib['x'] = thing.attrib.get('x', '0')
                tmp_thing.attrib['y'] = thing.attrib.get('y', '0')
                tmp_thing.attrib['height'] = thing.attrib['height']
                tmp_thing.attrib['width'] = thing.attrib['width']
                i.attrib = tmp_thing.attrib
            elif thing.tag == header + 'circle':
                tmp_thing.attrib['cx'] = thing.attrib['cx']
                tmp_thing.attrib['cy'] = thing.attrib['cy']
                tmp_thing.attrib['r'] = thing.attrib['r']
                i.attrib = tmp_thing.attrib
            elif thing.tag == header + 'ellipse':
                tmp_thing.attrib['cx'] = thing.attrib['cx']
                tmp_thing.attrib['cy'] = thing.attrib['cy']
                tmp_thing.attrib['rx'] = thing.attrib['rx']
                tmp_thing.attrib['ry'] = thing.attrib['ry']
                i.attrib = tmp_thing.attrib
            elif thing.tag == header + 'line':
                tmp_thing.attrib['x1'] = thing.attrib['x1']
                tmp_thing.attrib['y1'] = thing.attrib['y1']
                tmp_thing.attrib['x2'] = thing.attrib['x2']
                tmp_thing.attrib['y2'] = thing.attrib['y2']
                i.attrib = tmp_thing.attrib
            elif thing.tag == header + 'polygon':
                tmp_thing.attrib['points'] = thing.attrib['points']
                i.attrib = tmp_thing.attrib
            elif thing.tag == header + 'polyline':
                tmp_thing.attrib['points'] = thing.attrib['points']
                i.attrib = tmp_thing.attrib
            elif thing.tag == header + 'path':
                tmp_thing.attrib['d'] = thing.attrib['d']
                i.attrib = tmp_thing.attrib
            elif thing.tag == header + 'style':
                i.tag = 'delete'
            elif thing.tag == header + 'text':
                warning.append('TEXT_TAG')
                i.tag = 'delete'
            else:
                i.attrib = {}

        for node_need_delete in root.findall('delete'):
            root.remove(node_need_delete)

        path_data = SVGParser.elements_to_list(root)

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
            # viewBox
            viewBox = list(map(float, viewBox.replace(',', ' ').split()))
            viewBox[2] = viewBox[0] + viewBox[2]
            viewBox[3] = viewBox[1] + viewBox[3]
        viewBox = [max(viewBox[0], min_x), max(viewBox[1], min_y), min(viewBox[2], max_x), min(viewBox[3], max_y)]

        viewBox[2] = viewBox[2] - viewBox[0]
        viewBox[3] = viewBox[3] - viewBox[1]

        if any(i == float('inf') or i == float('-inf') for i in viewBox):
            print(viewBox)
            viewBox[2] = 0
            viewBox[3] = 0
            warning.append('FAIL_PARSING')
        else:

            root.attrib = {}

            root.attrib['width'] = str(viewBox[2])
            root.attrib['height'] = str(viewBox[3])

            # stroke strip problem fake solution
            # real solution ref: http://stackoverflow.com/questions/7241393/can-you-control-how-an-svgs-stroke-width-is-drawn
            viewBox[0] -= .5
            viewBox[1] -= .5
            viewBox[2] += 1
            viewBox[3] += 1

            root.attrib['viewBox'] = " ".join(map(str, viewBox))

        ################ fake code ##############
        if environ.get("flux_debug") == '1':
            tree.write('preprocess.svg')
        ########################################

        return warning, [ET.tostring(root), viewBox[2], viewBox[3]]  # ET.tostring type: bytes

    @staticmethod
    def process(path_data, params, viewBox, radius):
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
                out = 0  # flag show how the point's are out of viewBox, using binary encoding
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
                        if y1 >= viewBox[1] and y1 <= viewBox[1] + viewBox[3]:

                            x_candidate = sorted([x1, x2, viewBox[0], viewBox[0] + viewBox[2]])

                            if x1 < x2:
                                new_path.append([x_candidate[1], y1])
                                new_path.append([x_candidate[2], y1])
                            else:
                                new_path.append([x_candidate[2], y1])
                                new_path.append([x_candidate[1], y1])

                    elif x1 == x2:
                        # vertical line
                        if x1 >= viewBox[0] and x1 <= viewBox[0] + viewBox[2]:
                            y_candidate = sorted([y1, y2, viewBox[1], viewBox[1] + viewBox[3]])

                            if y1 < y2:
                                new_path.append([x1, y_candidate[1]])
                                new_path.append([x1, y_candidate[2]])
                            else:
                                new_path.append([x1, y_candidate[2]])
                                new_path.append([x1, y_candidate[1]])

                    # y = ax + b
                    else:
                        # get the line that go through (x1, y1), (x2, y2)
                        a = (y1 - y2) / (x1 - x2)
                        b = y1 - (a * x1)

                        candidate = []
                        for tmp_x in [viewBox[0], viewBox[0] + viewBox[2]]:
                            if a * tmp_x + b > viewBox[1] and a * tmp_x + b < viewBox[1] + viewBox[3]:
                                # check whether it's internal point of (x1, y1), (x2, y2)
                                if (tmp_x <= x1 and tmp_x >= x2) or (tmp_x >= x1 and tmp_x <= x2):
                                    candidate.append([tmp_x, a * tmp_x + b])

                        for tmp_y in [viewBox[1], viewBox[1] + viewBox[3]]:
                            if (tmp_y - b) / a > viewBox[0] and (tmp_y - b) / a < viewBox[0] + viewBox[2]:
                                # check whether it's internal point of (x1, y1), (x2, y2)
                                if (tmp_y <= y1 and tmp_y >= y2) or (tmp_y >= y1 and tmp_y <= y2):
                                    candidate.append([(tmp_y - b) / a, tmp_y])

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
                            elif out == 3:
                                raise ValueError('out == 3, happened only if math is broke')
                                pass

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
            if len(new_path) > 0:
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

            # make points into real world coordinate
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
            # make every points inside the boundary circle -> (cx, cy, r) = (0, 0, radius)
            in_path = []
            for i in range(1, len(new_path)):
                if new_path[i - 1][0] == '\n':
                    fake_x, fake_y = new_path[i]
                else:
                    fake_x, fake_y = new_path[i - 1]  # record where head should be as if there's no boundary
                if new_path[i][0] != '\n':
                    flag = 0
                    if new_path[i][0] ** 2 + new_path[i][1] ** 2 > radius ** 2:
                        flag += 1
                    if fake_x ** 2 + fake_y ** 2 > radius ** 2:
                        flag += 2

                    if flag == 0:  # both inside the circle
                        in_path.append(new_path[i - 1])
                        in_path.append(new_path[i])
                    else:
                        # find the intersection point between vector a->b and circle
                        # a = (x1, y1), b = (x2, y2), circle = (0, 0, r)
                        # (x1 + t*(x2-x1))^2 + (y1 + t(y2-y1))^2 = r^2
                        # solve t, and find the proper sign for it
                        x1, y1 = fake_x, fake_y
                        x2, y2 = new_path[i]
                        if x1 == x2 and y1 == y2:
                            continue
                        a = (x2 - x1) ** 2 + (y2 - y1) ** 2
                        b = 2 * ((x1 * x2) - (x1 ** 2) + (y1 * y2) - (y1 ** 2))
                        c = (x1 ** 2) + (y1 ** 2) - (radius ** 2)

                        if (b ** 2) - (4 * a * c) >= 0:  # solvable
                            t_p = (-b + sqrt((b ** 2) - (4 * a * c))) / (2 * a)
                            t_n = (-b - sqrt((b ** 2) - (4 * a * c))) / (2 * a)
                            v = [x2 - x1, y2 - y1]
                            if flag == 1:  # in to out
                                in_path.append([fake_x, fake_y])
                                t = t_p if abs(t_p) < 1 else t_n  # must be inner division point
                                in_path.append([fake_x + t * v[0], fake_y + t * v[1]])
                                in_path.append(['\n', '\n'])
                            elif flag == 2:  # out to in
                                t = t_p if abs(t_p) < 1 else t_n  # must be inner division point
                                in_path.append(['\n', '\n'])
                                in_path.append([fake_x + t * v[0], fake_y + t * v[1]])
                                in_path.append([new_path[i][0], new_path[i][1]])
                            elif flag == 3:  # both out
                                if abs(t_p) < 1 and abs(t_n) < 1:  # must be inner division point
                                    in_path.append(['\n', '\n'])
                                    in_path.append([fake_x + t_n * v[0], fake_y + t_n * v[1]])
                                    in_path.append([fake_x + t_p * v[0], fake_y + t_p * v[1]])
                                    in_path.append(['\n', '\n'])
                        else:  # insoluble
                            pass
                else:
                    in_path.append(new_path[i])

            # delete redundant points
            if len(in_path) > 0:
                tmp = [in_path[0]]
                for i in in_path:
                    if tmp[-1] != i:
                        tmp.append(i)
                in_path = tmp

            path_data[path] = in_path
        return path_data
