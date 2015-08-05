import logging
import tempfile
import struct
import sys
from zlib import crc32
from math import sqrt

from fluxclient.fcode.fcode_base import FcodeBase
from fluxclient.hw_profile import HW_PROFILE


logger = logging.getLogger("g_to_f")


class GcodeToFcode(FcodeBase):
    """transform from gcode to fcode
    fcode format: https://github.com/flux3dp/fluxmonitor/wiki/Flux-Device-Control-Describe-File-V1

    this should done several thing:
      check boundary problem
      get metadata
      long path will split into many different command in order to support emergency stop

    """
    def __init__(self, model='model-1'):
        super(GcodeToFcode, self).__init__()

        if model not in HW_PROFILE:
            logger.info("Undefine model:%d , using 'model-1'instead" % (model))
            model = "model-1"

        self.r = HW_PROFILE[model]["radius"]

        self.tool = 0  # set by T command
        self.absolute = True
        self.unit = 1

        self.crc = 0

        self.current_speed = 1
        self.image = None
        self.current_pos = [None, None, None, None]  # X, Y, Z, E
        self.time_need = 0.
        self.filament = 0.
        self.md = {'HEAD_TYPE': 'extruder'}
        # a = self.r[1:1 + 2]
        # x = x * 2 + 1

    def header(self):
        """
        simple header for fcode version 1
        """
        return b'FC' + b'x0001' + b'\n'

    def metadata(self, stream):
        """
        deal with meta data
        """
        md_join = '\n'.join([i + '=' + self.md[i] for i in self.md]).encode()
        print (md_join)

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
        parse data into a list: [F, X, Y, Z, E]
        element will be None if not setted
        and form the proper command
        """
        command = 0
        number = [None for _ in range(5)]
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
                number[4] = float(i[1:]) * self.unit
            else:
                print(i)
        return command, number

    def analyze_metadata(self, input_list):
        """
        input_list: [F, X, Y, Z, E]
        """
        self.current_pos[0]
        if input_list[0] is not None:
            self.current_speed = input_list[0]

        if input_list[4] is not None:  # extruder
            if self.absolute:
                self.filament += input_list[4] - self.current_pos[3]
                self.current_pos[3] = input_list[4]
            else:
                self.filament += input_list[4]
                self.current_pos[3] += input_list[4]

        tmp_path = 0.
        for i in range(1, 4):  # position
            if input_list[i] is not None:
                if self.absolute:
                    tmp_path += (input_list[i] - self.current_pos[i - 1]) ** 2
                    self.current_pos[i - 1] = input_list[i]
                else:
                    tmp_path += (input_list[i] ** 2)
                    self.current_pos[i - 1] += input_list[i]
        tmp_path = sqrt(tmp_path)
        self.time_need += tmp_path / self.current_speed  # TODO: figure out how time is compute

    def writer(self, buf, stream):
        """
        write data into stream
        update length and crc
        """
        self.script_length += len(buf)
        stream.write(buf)
        self.crc = crc32(buf, self.crc)

    def process(self, file_name):
        # fcode = tempfile.NamedTemporaryFile(suffix='.fcode', delete=False)
        fcode = open('GG.fcode', 'wb')
        fcode.write(self.header())

        packer = lambda x: struct.pack('<B', x)  # due to appear so many times, use this as a alias for 'struct.pack('<B', x)'

        fcode.write(struct.pack('<I', 0))  # script length
        self.script_length = 0

        with open(file_name, 'r') as f:
            if file_name[-6:] != '.gcode':
                raise ValueError('Unrecognized file format%s' % (file_name))
            for line in f:
                if ';' in line:
                    line = line[:line.index(';')].rstrip()

                line = line.split()

                if line:
                    if line[0] == 'G28':  # home
                        self.writer(packer(1), fcode)
                        for tmp in range(3):
                            self.current_pos[tmp] = 0

                    elif line[0] == 'G90':  # set to absolute
                        self.writer(packer(2), fcode)
                        self.absolute = True
                    elif line[0] == 'G91':  # set to relative
                        self.absolute = False
                        self.writer(packer(3), fcode)

                    elif line[0] == 'M82':  # set extruder to absolute
                        self.extrude_absolute = True
                    elif line[0] == 'M83':  # set extruder to relative
                        self.extrude_absolute = False

                    elif line[0] == 'G92':  # set position
                    # TODO:A G92 without coordinates will reset all axes to zero.
                        command = 64
                        sub_command, data = self.XYZEF(line)
                        command |= sub_command
                        self.writer(packer(command), fcode)
                        for i in range(len(data)):
                            if data[i] is not None:
                                self.writer(struct.pack('<f', i), fcode)
                                self.current_pos[i - 1] = data[i]

                    elif line[0] == 'G4':  # dwell
                        self.writer(packer(4), fcode)
                        self.writer(struct.pack('<f', float(line[1].lstrip('P'))), fcode)

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
                        self.writer(packer(command), fcode)
                        self.writer(struct.pack('<f', temp), fcode)

                    elif line[0] == 'G20' or line[0] == 'G21':  # set for inch or mm
                        if line == 'G20':  # inch
                            self.unit = 25.4
                        elif line == 'G21':  # mm
                            self.unit = 1

                    elif line[0] == 'G0' or line[0] == 'G1':  # move
                        command = 128
                        subcommand, data = self.XYZEF(line)
                        self.analyze_metadata(data)

                        command |= subcommand
                        self.writer(packer(command), fcode)
                        for i in data:
                            if i is not None:
                                self.writer(struct.pack('<f', i), fcode)

                    elif line[0] == 'T0' or line[0] == 'T1':  # change tool
                        if line[0] == 'T0':
                            self.tool = 0
                        if line[0] == 'T1':
                            self.tool = 1

                    elif line[0] == 'M107' or line[0] == 'M106':  # fan control
                        command = 48
                        command |= 1  # TODO: change this part
                        self.writer(packer(command), fcode)
                        if line[0] == 'M107':
                            self.writer(struct.pack('<f', 0.0), fcode)
                        elif line[0] == 'M106':
                            self.writer(struct.pack('<f', float(line[1].lstrip('S'))), fcode)

                    elif line[0] == 'M84':  # loosen the motor
                        pass  # should only appear when printing done
                    else:
                        print(line, file=sys.stderr)
                        raise ValueError('Undefine gcode')
        fcode.write(struct.pack('<I', self.crc))
        fcode.seek(len(self.header()), 0)
        fcode.write(struct.pack('<I', self.script_length))
        fcode.seek(0, 2)  # go back to file end

        self.md['FILAMENT_USED'] = str(self.filament)
        self.md['TIME_COST'] = str(self.time_need)
        print(self.filament, self.time_need)
        self.metadata(fcode)
if __name__ == '__main__':
    m_GcodeToFcode = GcodeToFcode()
    m_GcodeToFcode.process('tmp.gcode')
