#!/usr/bin/env python3
import sys
import argparse
import logging
from re import findall
from time import time
import datetime

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
    for i in findall('[A-Z][+-]?[0-9]*[.]?[0-9]*', command):
        timestamp = datetime.datetime.fromtimestamp(time()).strftime('%H:%M:%S')
        if i.startswith('N'):
            _PcProcess.delete_noise('in', 'in', float(i[1:]))
            print(timestamp, show_pc('after noise del', _PcProcess.clouds['in']))
        elif i.startswith('P'):
            print(timestamp, 'converting to mesh....', end='')
            _PcProcess.to_mesh('in')
            suffix = 'stl'
            print('done')
        elif i.startswith('E'):
            print(timestamp, 'export time file {}.{}'.format(prefix, suffix))
            tmp = _PcProcess.export('in', suffix)
            with open(prefix + '.' + suffix, 'wb') as f:
                f.write(tmp)

parser = argparse.ArgumentParser(description='An experiment tool for scanning improve', formatter_class=argparse.RawTextHelpFormatter)
parser.add_argument('-i', '--input', help='input filename', required=True)
parser.add_argument('-o', '--output', help='output filename', default='output.pcd')
parser.add_argument('-c', '--command', help='ex: N0.3PE\n'
                                            'N[stddev]: noise del with dev [stddev]\n'
                                            'P: Possion Meshing\n'
                                            'E: export file\n'
                    )
args = parser.parse_args()

if args.output is None:
    args.output = args.input[:-4] + "_out" + ".pcd"
if __name__ == '__main__':
    main(args.input, args.output, args.command)
