#!/usr/bin/env python3
import sys
import argparse
import logging
from re import findall
from time import time
import datetime
import numpy as np

from fluxclient.scanner.pc_process import PcProcess
from fluxclient.scanner.tools import read_pcd, write_stl, write_pcd
from fluxclient.scanner.pc_process import PcProcess


logger = logging.Logger(__name__)
logger.setLevel(level=logging.DEBUG)


def show_pc(name, pc_in):
    return '{} size:{}'.format(name, len(pc_in[0]))


def cross(p0, p1, p2):
    return ((p1[0] - p0[0]) * (p2[1] - p0[1])) - ((p1[1] - p0[1]) * (p2[0] - p0[0]))


def main(in_file, out_file, command=''):
    _PcProcess = PcProcess()
    tmp = out_file.rfind('.')
    prefix, suffix = out_file[:tmp], out_file[tmp + 1:]
    print('out_file', prefix, suffix)

    _PcProcess.clouds['in'] = _PcProcess.to_cpp([read_pcd(in_file), []])
    print(show_pc('pc_in', _PcProcess.clouds['in']))
    # for i in findall('[A-Z][+-]?[0-9]+[.]?[0-9]*', command):
    for i in findall('[A-Z][a-z]?[+-]?[0-9]*[.]?[0-9]*', command):
        timestamp = datetime.datetime.fromtimestamp(time()).strftime('%H:%M:%S')
        if i.startswith('N'):
            _PcProcess.delete_noise('in', 'in', float(i[1:]))
            print(timestamp, show_pc('after noise del', _PcProcess.clouds['in']))
        elif i.startswith('P'):
            print(timestamp, 'converting to mesh....', end='')
            _PcProcess.to_mesh('in')
            suffix = 'stl'
            print('done')
        elif i.startswith('A'):
            if i[1] == 'f':
                import bisect
                import copy
                from math import sqrt, asin, pi, radians, cos, sin

                floor = -10
                steps = 400
                print('adding floor')
                a = []
                # print(len(_PcProcess.clouds['in'][0]))
                _PcProcess.cut('in', 'in', 'z', True, floor)
                print(len(_PcProcess.clouds['in'][0]))
                # for z in range(len(_PcProcess.clouds['in'][0]) - 1):
                for z in range(len(_PcProcess.clouds['in'][0])):
                    # if _PcProcess.clouds['in'][0][z][2] < _PcProcess.clouds['in'][0][z + 1][2]:  # detect for tail
                    a.append(_PcProcess.clouds['in'][0][z])

                # get near floor and a ring point set
                d = [[p, asin(p[1] / sqrt(p[0] ** 2 + p[1] ** 2)) if p[0] > 0 else pi - asin(p[1] / sqrt(p[0] ** 2 + p[1] ** 2))] for p in a]  # add theta data
                d = sorted(d, key=lambda x: abs(x[0][2] - floor))

                rec = [float('-inf'), float('inf')]
                interval = 2 * pi / 400 * 0.9
                after = []
                for p in d:
                    tmp_index = bisect.bisect(rec, p[1])
                    if p[1] - rec[tmp_index - 1] > interval and rec[tmp_index] - p[1] > interval:
                        rec.insert(tmp_index, p[1])
                        after.append(p)
                    if len(rec) > 400 + 2:
                        break

                # print('rec l:', len(rec))
                # for p in after:
                #     p[0][3] = 255
                #     _PcProcess.clouds['in'][0].push_backPoint(*p[0])
                # continue

                # compute for center (no need?)
                after = sorted(after, key=lambda x: x[1])
                after = [p[0] for p in after]
                # c = [0.0 for _ in range(6)]
                # for p in after:
                #     for _ in range(6):
                #         c[_] += p[_]
                # for _ in range(6):
                #     c[_] /= len(after)

                plane = copy.deepcopy(after)
                for p in plane:
                    pass
                    # print(p[2])
                    # p[2] = floor

                # for p in plane:
                #     p[3] = 255
                #     _PcProcess.clouds['in'][0].push_backPoint(*p)
                # continue

                index = list(range(len(plane)))
                index = sorted(index, key=lambda x: [after[x][0], after[x][1]])
                output = []
                for j in index:  # upper
                    while len(output) >= 2 and cross(after[output[-2]], after[output[-1]], after[j]) <= 0:
                        output.pop()
                    output.append(j)

                t = len(output) + 1
                for j in index[-2::-1]:  # lower
                    while len(output) > t and cross(after[output[-2]], after[output[-1]], after[j]) <= 0:
                        output.pop()
                    output.append(j)
                output.pop()

                boarder = [after[x] for x in sorted(output)]
                # for p in plane:
                #     p[3] = 255
                #     _PcProcess.clouds['in'][0].push_backPoint(*p)
                # continue
                # plane = after

                from scipy.interpolate import Rbf
                tmp = []
                grid_leaf = 100
                X = np.linspace(min(p[0] for p in plane), max(p[0] for p in plane), grid_leaf)
                Y = np.linspace(min(p[1] for p in plane), max(p[1] for p in plane), grid_leaf)
                XI, YI = np.meshgrid(X, Y)

                x = [p[0] for p in plane]
                y = [p[1] for p in plane]
                z = [p[2] for p in plane]
                print(len(z))

                rbf = Rbf(x, y, z, function='linear')
                ZI = rbf(XI, YI)
                print(ZI.dtype)
                for xx in range(grid_leaf):
                    for yy in range(grid_leaf):
                        # print([XI[xx][yy], YI[xx][yy], ZI[xx][yy], 255, 0, 0])
                        p = [XI[xx][yy], YI[xx][yy], ZI[xx][yy], 255, 0, 0]
                        flag = True
                        for b in range(len(boarder)):
                            if (cross(boarder[b], boarder[(b + 1) % len(boarder)], p)) < 0:
                                flag = False
                                break
                        if flag:
                            tmp.append(p)
                            # print(p[2])

                # for d in range(360):
                #     for r in range(100):
                #         p = [r * cos(radians(d)), r * sin(radians(d)), floor, 0, 0, 0]
                #         flag = True
                #         for b in range(len(plane)):
                #             if (cross(plane[b], plane[(b + 1) % len(plane)], p)) < 0:
                #                 flag = False
                #                 break
                #         if flag:
                #             tmp.append(p)

                plane += tmp
                # _PcProcess.add_floor('in', 'in')
                for p in plane:
                    p[3] = 255
                    _PcProcess.clouds['in'][0].push_backPoint(*p)

                # _PcProcess.clouds['in'] = _PcProcess.to_cpp([plane, []])

                # _PcProcess.clouds['in'][0].to_mesh('GPT')
                # suffix = 'stl'

            elif i[1] == 'c':
                print('adding ceiling')
            print('done')
        elif i.startswith('E'):
            print(timestamp, 'export file {}.{}'.format(prefix, suffix))
            tmp = _PcProcess.export('in', suffix)
            with open(prefix + '.' + suffix, 'wb') as f:
                f.write(tmp)

parser = argparse.ArgumentParser(description='An experiment tool for scanning improve', formatter_class=argparse.RawTextHelpFormatter)
parser.add_argument('-i', '--input', help='input filename', required=True)
parser.add_argument('-o', '--output', help='output filename', default='output.pcd')
parser.add_argument('-c', '--command', default='', help='ex: N0.3PE\n'
                                                        'N[stddev]: noise del with dev [stddev]\n'
                                                        'P: Possion Meshing\n'
                                                        'E: export file\n'
                    )
args = parser.parse_args()

if args.output is None:
    args.output = args.input[:-4] + "_out" + ".pcd"
if __name__ == '__main__':
    main(args.input, args.output, args.command)
