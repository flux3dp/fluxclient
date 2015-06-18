#!/usr/bin/env python

import math
import struct
import sys
import os
from time import time
from datetime import datetime


from datetime import datetime
name = datetime.fromtimestamp(time()).strftime('%Y-%m-%d-%H-%M-%S/')  # name
scan_step = 600  # steps
camera_port = 0
# serial_port = 'COM9'
serial_port = '/dev/tty.usbmodem1411'
serial_port = '/dev/cu.usbmodem14141'
throw = 3
sleeping_time = 0.07  # wait for some time to make sure hardware reacted
# sleeping_time = 0  # wait for some time to make sure hardware reacted
real_world = 1
theta_a = math.pi / 6 * 2 / 2  # rad between center and laser
img_width = 1024
img_height = 768
store_img = True  # whether or not store the img


sensorWidth = 3.67
sensorHeight = 2.74
focalLength = 3.6

cameraX = 0.0
cameraY = 170.0
cameraZ = 90

laserX_L = -52.0
laserY_L = 77.0
laserZ_L = 133

laserX_R = 52.0
laserY_R = 77.0
laserZ_R = 133

# cameraX = 0.0
# cameraY = 120.0
# cameraZ = 150.0

# laserX_L = -80.0
# laserY_L = 120.0
# laserZ_L = 120

# laserX_R = 80.0
# laserY_R = 120.0
# laserZ_R = 120


if "-n" in sys.argv:
    name = "./" + sys.argv[sys.argv.index("-n") + 1] + "/"
    if not os.path.exists(name):
        os.mkdir(name)
    # print "store into"+name


def cutter(indices, mode, s, e=None):
    '''
      not finished yet
    '''
    if mode == 'radius':
        if e is None:
            e = 2147483647
        indices = filter(lambda x: x[1] > s and x[1] < e, indices)

    elif mode == 'x':
        pass
    elif mode == 'y':
        pass
    elif mode == 'z':
        if e is None:
            e = 2147483647
        indices = filter(lambda x: x[0] > s and x[0] < e, indices)

    else:
        print >>sys.stderr, 'wrong mode'
    return indices


def normalize(v):

    l = v[0] * v[0] + v[1] * v[1] + v[2] * v[2]
    l = math.sqrt(l)

    v[0] /= l
    v[1] /= l
    v[2] /= l

    return v


def img_to_points(img_o, img_red, indices, step, side, clock=False):
    '''
      from img and coordinate to pointclooud
      img_o, img_red: original img and lasered img
      indices:[[row, col], [row, col], [row, col], ... ] location that laser at on img
      step : which step it is, should always < scan_step
      side : side of laser, can only be 'L' or 'R'
      clock : how plate rotate, default False assuming to be anticlockwise, change to True if it's not the case
    '''
    points = []
    if clock is False:
        step = - step

    if side == 'L':
        rad = math.pi * 2 * step / scan_step - theta_a
    elif side == 'R':
        rad = math.pi * 2 * step / scan_step + theta_a

    C = math.cos(rad)
    S = math.sin(rad)

    for y, x in (indices):
        if side == 'L':
            R = (0.5 * img_width - x) / math.sin(theta_a)
        elif side == 'R':
            R = (x - 0.5 * img_width) / math.sin(theta_a)

        points.append((R * C,
                       R * S,
                       # (img_height - y) * real_world,
                       (img_height - y + 0.5 * ((step % 4) - 2)
                        * (step % 2)) * real_world,
                       img_o[y][x][0], img_o[y][x][1], img_o[y][x][2]
                       )
                      )
    return points


def pre_cut(img, x=0, y=0, w=None, h=None):
    return img[y: y + h, x: x + w]  # x, y, w, h


def point_dis_sq(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[1] - b[1]) ** 2


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def check_tri(tri, thres=25):
    '''
      check if a triangle is valid
      return True if each edge is smaller than thres, False otherwise
      tri = [[x, y, z], [x, y, z], [x, y, z]]
    '''
    thres = thres ** 2
    if (tri[0][0] - tri[1][0]) ** 2 + (tri[0][1] - tri[1][1]) ** 2 + (tri[0][2] - tri[1][2]) ** 2 > thres:
        return False
    elif (tri[0][0] - tri[2][0]) ** 2 + (tri[0][1] - tri[2][1]) ** 2 + (tri[0][2] - tri[2][2]) ** 2 > thres:
        return False
    elif (tri[1][0] - tri[2][0]) ** 2 + (tri[1][1] - tri[2][1]) ** 2 + (tri[1][2] - tri[2][2]) ** 2 > thres:
        return False
    else:
        return True


def normal(tri):
    '''
      compute normal of a triangle
      tri = [[x, y, z], [x, y, z], [x, y, z]]
    '''
    return (
        (tri[1][1] - tri[1][0]) * (tri[2][2] - tri[2][0]) -
        (tri[1][2] - tri[1][0]) * (tri[2][1] - tri[2][0]),
        (tri[2][1] - tri[2][0]) * (tri[0][2] - tri[0][0]) -
        (tri[0][1] - tri[0][0]) * (tri[2][2] - tri[2][0]),
        (tri[0][1] - tri[0][0]) * (tri[1][2] - tri[1][0]) -
        (tri[0][2] - tri[0][0]) * (tri[1][1] - tri[1][0])

    )


def pcd_write(points, file_name="model.pcd"):
    '''
      write a pointclooud as .pcd file, in binary mode or ascii mode. (compact or readable)
      points should look like this:
                            [
                              p1[x-coordinate, y-coord, z-coord, b, g, r],
                              p2[x-coordinate, y-coord, z-coord, b, g, r],
                              p3[x-coordinate, y-coord, z-coord, b, g, r],
                                ...
                            ]
                            (coordinate and color data for each point, in a list of points for a pointclooud)
    '''
    with open(file_name, 'w') as f:
        # write Header
        print >> f, '# .PCD v.7 - Point Cloud Data file format'
        print >> f, 'VERSION .7'
        print >> f, 'FIELDS x y z rgb'
        print >> f, 'SIZE 4 4 4 4'  # temp
        print >> f, 'TYPE F F F F'
        print >> f, 'COUNT 1 1 1 1'
        print >> f, 'WIDTH %d' % len(points)
        print >> f, 'HEIGHT 1'
        print >> f, 'VIEWPOINT 0 0 0 10 0 0 0'
        print >> f, 'POINTS %d' % len(points)
        print >> f, 'DATA ascii'
        for i in points:
            # print i
            try:
                print >> f, '%f %f %f %f' % (
                    i[0], i[1], i[2], i[3] + (i[4]) * 256 + (i[5]) * 256 * 256)
            except:
                pass
                # print i, points.index(i)

    print ('write', len(points), 'points into ' + file_name)


def write_stl(tri, output='model.stl', mode='binary'):
    '''
      write a 3d model as stl file, in binary mode or ascii mode. (compact or readable)
      tri should look like this:
                            [
                            t1[p1[x, y, z], p2[x, y, z], p3[x, y, z]],
                            t2[p1[x, y, z], p2[x, y, z], p3[x, y, z]],
                            t3[p1[x, y, z], p2[x, y, z], p3[x, y, z]],
                              ...
                            ]
                            (xyz each point, 3 points for each triangle, triangles for a model)
    '''
    if mode == 'binary':
        with open(output, 'wb') as outstl:
            Header = 'FLUX 3d printer: flux3dp.com, 2015'

            for i in range(80):
                if i < len(Header):
                    outstl.write(struct.pack('c', Header[i]))
                else:
                    outstl.write(struct.pack("c", ' '))

            outstl.write(struct.pack("I", len(tri)))
            for i in tri:
                outstl.write(struct.pack("fff", 0, 0, 0))
                for j in i:
                    for k in j:
                        outstl.write(struct.pack("f", k))
                for j in range(2):
                    outstl.write(struct.pack("?", False))

    elif mode == 'ascii':
        with open(output, 'w') as outstl:
            print >>outstl, 'solid ascii'
            for i in tri:
                # tmp = normal(i)
                # print >>outstl, ' facet normal ',tmp[0],tmp[1],tmp[2]
                print >>outstl, ' facet normal 0 0 0'

                print >>outstl, '  outer loop'
                for j in i:
                    print >>outstl, '   vertex', j[0], j[1], j[2]
                print >>outstl, '  endloop'
                print >>outstl, ' endfacet'
            print >>outstl, 'endsolid'

    else:
        print >> sys.err, 'mode error, mode could only be \'binary\' or \'ascii\''
        return

    print (len(tri), 'triangle write in' + output)


def stupid_smoooth(indices, sp=2):
    after_smooth = []
    tmp = 0.
    for i in range(sp):
        tmp += indices[i][1]

    for i in range(sp + 1):
        tmp += indices[i + sp][1]
        after_smooth.append((indices[i][0], round(tmp / (i + 1 + sp))))

    for i in range(sp + 1, len(indices) - sp):
        tmp -= indices[i - sp - 1][1]
        tmp += indices[i + sp][1]
        after_smooth.append((indices[i][0], round(tmp / (2 * sp + 1))))

    for i in range(len(indices) - sp, len(indices)):
        tmp -= indices[i - sp - 1][1]
        after_smooth.append(
            (indices[i][0], round(tmp / (len(indices) - i + sp))))

    return after_smooth

# if __name__ == '__main__':
#     img = cv2.imread('./box/0_L.png')
#     print (img.shape)
#     cv2.imshow("cropped", pre_cut(img))
#     cv2.waitKey(0)
