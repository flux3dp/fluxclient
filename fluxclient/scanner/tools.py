#!/usr/bin/env python3
import math


def write_pcd(points, file_name="model.pcd"):
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
            # print i
            try:
                print('%f %f %f %f' % (i[0], i[1], i[2], i[3] | (i[4] << 8) | (i[5] << 16)), file=f)
            except:
                pass
                # print i, points.index(i)

    print('write', len(points), 'points into ' + file_name)


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

    print(len(tri), 'triangle write in' + output)