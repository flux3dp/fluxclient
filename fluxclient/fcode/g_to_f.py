import logging
import tempfile
import struct
import sys
from zlib import crc32
from math import sqrt
import time
from getpass import getuser

from fluxclient.fcode.fcode_base import FcodeBase
from fluxclient.hw_profile import HW_PROFILE


logger = logging.getLogger("g_to_f")


class GcodeToFcode(FcodeBase):
    """transform from gcode to fcode
    fcode format: https://github.com/flux3dp/fluxmonitor/wiki/Flux-Device-Control-Describe-File-V1

    this should done several thing:
      transform gcode into fcode
      analyze metadata
      check boundary problem (?
    """
    def __init__(self, version=1):
        super(GcodeToFcode, self).__init__()

        self.tool = 0  # set by T command
        self.absolute = True  # whether using absolute position
        self.unit = 1  # how many mm is one unit in gcode, might be mm or inch(2.54)

        self.crc = 0  # computing crc32

        self.current_speed = 1  # current speed (set by F), mm/minute
        self.image = None  # png image, should be a bytes obj
        self.current_pos = [None, None, None, None, None, None]  # X, Y, Z, E1, E2, E3 -> recording the position of each axis
        self.time_need = 0.  # recording time the printing process need, in sec
        self.filament = [0., 0., 0.]  # recording the filament needed, in mm
        self.md = {'HEAD_TYPE': 'extruder'}  # basic metadata, use extruder as

        self.record_path = True
        self.record_z = 0.0
        self.path = [[[0.0, 0.0, 0.0, 3]]]  # recording the path extruder go through

    def header(self):
        """
        simple header for fcode version 1
        """
        return b'FC' + b'x0001' + b'\n'

    def write_metadata(self, stream):
        """
        deal with meta data(a dict, and a png image)
        """
        md_join = '\x00'.join([i + '=' + self.md[i] for i in self.md]).encode()

        stream.write(struct.pack('<I', len(md_join)))
        stream.write(md_join)
        stream.write(struct.pack('<I', crc32(md_join)))

        if self.image is None:
            stream.write(struct.pack('<I', 0))
        else:
            stream.write(struct.pack('<I', len(self.image)))
            stream.write(self.image)

    def XYZEF(self, input_list):
        """
        parse data into a list: [F, X, Y, Z, E1, E2, E3]
        element will be None if not setted
        and form the proper command
        """
        command = 0
        number = [None for _ in range(7)]
        for i in input_list[1:]:
            if i.startswith('F'):
                command |= (1 << 6)
                number[0] = float(i[1:])
            elif i.startswith('X'):
                command |= (1 << 5)
                number[1] = float(i[1:]) * self.unit
            elif i.startswith('Y'):
                command |= (1 << 4)
                number[2] = float(i[1:]) * self.unit
            elif i.startswith('Z'):
                command |= (1 << 3)
                number[3] = float(i[1:]) * self.unit
            elif i.startswith('E'):
                command |= (1 << (2 - self.tool))
                number[4 + self.tool] = float(i[1:]) * self.unit
            else:
                print(i)
        return command, number

    def analyze_metadata(self, input_list, comment):
        """
        input_list: [F, X, Y, Z, E1, E2, E3]
        compute filament need for each extruder
        compute time needed
        """
        if input_list[0] is not None:
            self.current_speed = input_list[0]

        extrudeflag = False
        for i in range(4, 7):  # extruder
            if input_list[i] is not None:
                extrudeflag = True
                if self.absolute:
                    self.filament[i - 4] += input_list[i] - self.current_pos[i - 1]
                    self.current_pos[i - 1] = input_list[i]
                else:
                    self.filament[i - 4] += input_list[i]
                    self.current_pos[i - 1] += input_list[i]

        tmp_path = 0.
        moveflag = False  # record if position change in this command
        for i in range(1, 4):  # position
            if input_list[i] is not None:
                moveflag = True
                if self.absolute:
                    tmp_path += (input_list[i] - self.current_pos[i - 1]) ** 2
                    self.current_pos[i - 1] = input_list[i]
                else:
                    tmp_path += (input_list[i] ** 2)
                    self.current_pos[i - 1] += input_list[i]
        tmp_path = sqrt(tmp_path)
        self.time_need += tmp_path / self.current_speed * 60  # from minute to sec
        # fill in self.path
        if self.record_path:
            self.process_path(comment, moveflag, extrudeflag)

    def writer(self, buf, stream):
        """
        write data into stream
        update length and crc
        """
        self.script_length += len(buf)
        stream.write(buf)
        self.crc = crc32(buf, self.crc)

    def process(self, input_stream, output_stream):
        try:
            # fcode = tempfile.NamedTemporaryFile(suffix='.fcode', delete=False)
            output_stream.write(self.header())

            packer = lambda x: struct.pack('<B', x)  # easy alias for struct.pack('<B', x)
            packer_f = lambda x: struct.pack('<f', x)  # easy alias for struct.pack('<f', x)

            output_stream.write(struct.pack('<I', 0))  # script length
            self.script_length = 0

            for line in input_stream:
                if ';' in line:
                    line, comment = line.split(';', 1)
                else:
                    comment = ''
                line = line.rstrip().split()

                if line:
                    if line[0] == 'G28':  # home
                        self.writer(packer(1), output_stream)
                        for tmp in range(3):
                            self.current_pos[tmp] = HW_PROFILE['model-1']['height']

                    elif line[0] == 'G90':  # set to absolute
                        self.writer(packer(2), output_stream)
                        self.absolute = True
                    elif line[0] == 'G91':  # set to relative
                        self.absolute = False
                        self.writer(packer(3), output_stream)

                    elif line[0] == 'M82':  # set extruder to absolute
                        self.extrude_absolute = True
                    elif line[0] == 'M83':  # set extruder to relative
                        self.extrude_absolute = False

                    elif line[0] == 'G92':  # set position
                        command = 64
                        sub_command, data = self.XYZEF(line)

                        tmp = lambda x: x is None
                        if all(tmp(i) for i in data):
                            sub_command = 0
                            for i in range(1, 7):
                                sub_command |= (1 << (6 - i))
                                data[i] = 0.0
                        else:  # A G92 without coordinates will reset all axes to zero.
                            pass

                        command |= sub_command
                        self.writer(packer(command), output_stream)
                        for i in range(len(data)):
                            if data[i] is not None:
                                self.writer(packer_f(data[i]), output_stream)
                                self.current_pos[i - 1] = data[i]
                    elif line[0] == 'G4':  # dwell
                        self.writer(packer(4), output_stream)
                        self.writer(packer_f(float(line[1].lstrip('P'))), output_stream)

                    elif line[0] == 'M104' or line[0] == 'M109':  # set extruder temperature
                        command = 16
                        if line[0] == 'M109':
                            command |= (1 << 3)
                        for i in line:
                            if i.startswith('S'):
                                temp = float(i.lstrip('S'))
                            elif i.startswith('T'):
                                self.tool = int(i.lstrip('T'))
                                if self.tool > 7:
                                    raise ValueError('too many extruder! %d' % self.tool)
                        command |= self.tool
                        self.writer(packer(command), output_stream)
                        self.writer(packer_f(temp), output_stream)

                    elif line[0] == 'G20' or line[0] == 'G21':  # set for inch or mm
                        if line == 'G20':  # inch
                            self.unit = 25.4
                        elif line == 'G21':  # mm
                            self.unit = 1

                    elif line[0] == 'G0' or line[0] == 'G1':  # move
                        command = 128
                        subcommand, data = self.XYZEF(line)
                        self.analyze_metadata(data, comment)

                        command |= subcommand
                        self.writer(packer(command), output_stream)
                        for i in data:
                            if i is not None:
                                self.writer(packer_f(i), output_stream)

                    elif line[0] == 'T0' or line[0] == 'T1':  # change tool
                        if line[0] == 'T0':
                            self.tool = 0
                        if line[0] == 'T1':
                            self.tool = 1

                    elif line[0] == 'M107' or line[0] == 'M106':  # fan control
                        command = 48
                        command |= 1  # TODO: change this part, consder fan control protocol
                        self.writer(packer(command), output_stream)
                        if line[0] == 'M107':
                            self.writer(packer_f(0.0), output_stream)
                        elif line[0] == 'M106':
                            self.writer(packer_f(float(line[1].lstrip('S')) / 255.), output_stream)

                    elif line[0] in ['M84', 'M140']:  # loosen the motor
                        pass  # should only appear when printing done, not define in fcode yet
                    else:
                        if line[0] in ['G4', 'M400'] or line[0].startswith('X2O'):
                            pass
                        else:
                            print('Undefine gcode', line, file=sys.stderr)
                        # raise ValueError('Undefine gcode', line)
            output_stream.write(struct.pack('<I', self.crc))
            output_stream.seek(len(self.header()), 0)
            output_stream.write(struct.pack('<I', self.script_length))
            output_stream.seek(0, 2)  # go back to file end

            # warning: fileformat didn't consider multi-extruder, use first extruder instead
            self.md['FILAMENT_USED'] = ','.join(map(str, self.filament))
            self.md['TIME_COST'] = str(self.time_need)
            self.md['CREATED_AT'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.localtime(time.time()))
            self.md['AUTHOR'] = getuser()  # TODO: use fluxstudio user name?
            self.write_metadata(output_stream)
        except Exception as e:
            print('FcodeError:')
            raise e

if __name__ == '__main__':
    m_GcodeToFcode = GcodeToFcode()
    with open(sys.argv[1], 'r') as input_stream:
        with open(sys.argv[2], 'wb') as output_stream:
            m_GcodeToFcode.process(input_stream, output_stream)
    if sys.argv[3]:
        with open(sys.argv[3], 'w') as f:
            if m_GcodeToFcode.path is None:
                print('', file=f)
            else:
                result = []
                for layer in m_GcodeToFcode.path:
                    tmp = []
                    for p in layer:
                        tmp.append('{"t":%d, "p":[%.5f, %.5f, %.5f]}' % (p[3], p[0], p[1], p[2]))
                    result.append('[' + ','.join(tmp) + ']')
                print('[' + ','.join(result) + ']', file=f)
