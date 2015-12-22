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
            print('adding at {}...'.format(float(i[2:])))
            _PcProcess.closure('in', 'in', float(i[2:]), i[1] == 'f')
            print('done')
        elif i.startswith('E'):
            print(timestamp, 'export file {}.{}'.format(prefix, suffix))
            tmp = _PcProcess.export('in', suffix)
            with open(prefix + '.' + suffix, 'wb') as f:
                f.write(tmp)
        elif i.startswith('C'):
            print('Clustering')
            pc = _PcProcess.clouds['in']
            thres = float(i[1:])
            output = pc[0].Euclidean_Cluster(thres)
            output = sorted(output, key=lambda x: len(x))
            print('finish with {} cluster'.format(len(output)))
            # Euclidean_Cluster
            import random
            r = lambda: random.randint(0, 255)

            tmp_pc = _PcProcess.to_cpp([[], []])
            for j in output[-1:]:
                c = [r(), r(), r()]
                for i in j:
                    p = _PcProcess.clouds['in'][0][i]
                    # tmp_pc[0].push_backPoint(p[0], p[1], p[2], *c)
                    tmp_pc[0].push_backPoint(*p)
            _PcProcess.clouds['in'] = tmp_pc


parser = argparse.ArgumentParser(description='An experiment tool for scanning improve', formatter_class=argparse.RawTextHelpFormatter)
parser.add_argument('-i', '--input', help='input filename', required=True)
parser.add_argument('-o', '--output', help='output filename', default='output.pcd')
parser.add_argument('-c', '--command', default='', help='ex: N0.3PE\n'
                                                        'N[stddev]: noise del with dev [stddev]\n'
                                                        'P: Possion Meshing\n'
                                                        'A[fc][value]: add [floor] or [ceiling] at z = [value]'
                                                        'E: export file\n'
                    )
args = parser.parse_args()

if args.output is None:
    args.output = args.input[:-4] + "_out" + ".pcd"
if __name__ == '__main__':
    main(args.input, args.output, args.command)
