#!/usr/bin/env python3
import struct
import sys

# PCL NOTE: http://docs.pointclouds.org/1.7.0/structpcl_1_1_point_x_y_z_r_g_b.html
# uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
# uint8_t r = (rgb >> 16) & 0x0000ff;
# uint8_t g = (rgb >> 8)  & 0x0000ff;
# uint8_t b = (rgb)       & 0x0000ff;


def read_pcd(file_name):
    assert file_name[-4:] == '.pcd', '%s is not a pcd file?' % file_name
    pc = []
    with open(file_name, 'r') as pcd:
        for _ in range(11):
            pcd.readline()  # read the header
        for line in pcd:
            point = [float(j) for j in line.rstrip().split(' ')]
            rgb = int(point.pop())
            b, g, r = rgb & 0x0000ff, (rgb >> 8) & 0x0000ff, (rgb >> 16) & 0x0000ff
            point += [r, g, b]
            pc.append(point)
    return pc


def write_pcd(points, file_name="model.pcd"):
    '''
      write a pointclooud as .pcd file, in binary mode or ascii mode. (compact or readable)
      points should look like this:
                            [
                              p1[x-coordinate, y-coord, z-coord, r, g, b],
                              p2[x-coordinate, y-coord, z-coord, r, g, b],
                              p3[x-coordinate, y-coord, z-coord, r, g, b],
                                ...
                            ]
                            (coordinate and color data for each point, in a list of points for a pointclooud)
    use StringsIO for output
    '''
    if type(file_name) == str:
        f = open(file_name, 'w')
    else:
        f = file_name

    # write Header
    print('# .PCD v.7 - Point Cloud Data file format', file=f)
    print('VERSION .7', file=f)
    print('FIELDS x y z rgb', file=f)
    print('SIZE 4 4 4 4', file=f)  # temp
    print('TYPE F F F F', file=f)
    print('COUNT 1 1 1 1', file=f)
    print('WIDTH %d' % len(points), file=f)
    print('HEIGHT 1', file=f)
    print('VIEWPOINT 0 0 0 10 0 0 0', file=f)
    print('POINTS %d' % len(points), file=f)
    print('DATA ascii', file=f)
    for i in points:
        print('%f %f %f %f' % (i[0], i[1], i[2], (int(i[3]) << 16) | (int(i[4]) << 8) | int(i[5])), file=f)

    print('write', len(points), 'points into ' + file_name, file=sys.stderr)


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
    use bytesIO for binary output
    use StringsIO for ascii output

    '''
    if type(output) == str:
        if mode == 'binary':
            outstl = open(output, 'wb')
        elif mode == 'ascii':
            outstl = open(output, 'w')

    else:
        outstl = output

    if mode == 'binary':
        Header = b'FLUX 3d printer: flux3dp.com, 2015'

        for i in range(80):
            if i < len(Header):
                outstl.write(struct.pack('c', Header[i:i + 1]))
            else:
                outstl.write(struct.pack("c", b' '))

        outstl.write(struct.pack("I", len(tri)))
        for i in tri:
            outstl.write(struct.pack("fff", 0, 0, 0))
            for j in i:
                for k in j:
                    outstl.write(struct.pack("f", k))
            for j in range(2):
                outstl.write(struct.pack("?", False))

    elif mode == 'ascii':
        print('solid ascii', file=outstl)
        for i in tri:
            # tmp = normal(i)
            # print(' facet normal ',tmp[0],tmp[1],tmp[2], file=outstl)
            print(' facet normal 0 0 0', file=outstl)

            print('  outer loop', file=outstl)
            for j in i:
                print('   vertex', j[0], j[1], j[2], file=outstl)
            print('  endloop', file=outstl)
            print(' endfacet', file=outstl)
        print('endsolid', file=outstl)

    else:
        print('mode error, mode could only be \'binary\' or \'ascii\'', file=sys.err)
        return
