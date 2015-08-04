import logging
import tempfile
import struct
import sys
from zlib import crc32

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

        # a = self.r[1:1 + 2]
        # x = x * 2 + 1

    def header(self):
        return b'FC' + b'x0001' + b'\n'

    def metadata(self, stream):
        md = b'HEAD_TYPE=extruder'
        stream.write(struct.pack('<I', len(md)))
        stream.write(md)
        stream.write(struct.pack('<I', crc32(md)))
        stream.write(struct.pack('<I', 0))

    def XYZEF(self, input_list):
        command = 0
        number = []
        for i in input_list[0]:
            if i.startswith('F'):
                command |= (1 << 6)
                number.append(float(i[1:]))
            elif i.startswith('X'):
                command |= (1 << 5)
                number.append(float(i[1:]))
            elif i.startswith('Y'):
                command |= (1 << 4)
                number.append(float(i[1:]))
            elif i.startswith('Z'):
                command |= (1 << 3)
                number.append(float(i[1:]))
            elif i.startswith('E'):
                command |= (1 << (2 - self.tool))
                number.append(float(i[1:]))
        return command, number

    def writer(self, buf, stream):
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
                        if line[0] == 'G28':
                            self.writer(packer(1), fcode)

                        elif line[0] == 'G90':
                            self.writer(packer(2), fcode)
                            self.absolute = True
                        elif line[0] == 'G91':
                            self.absolute = False
                            self.writer(packer(3), fcode)

                        elif line[0] == 'M82':
                            self.extrude_absolute = True
                        elif line[0] == 'M83':
                            self.extrude_absolute = False

                        elif line[0] == 'G92':
                            command = 64
                            sub_command, data = self.XYZEF(line)
                            command |= sub_command
                            self.writer(packer(command), fcode)
                            for i in data:
                                self.writer(struct.pack('<f', i), fcode)

                        elif line[0] == 'G4':
                            self.writer(packer(4), fcode)
                            self.writer(struct.pack('<f', float(line[1].lstrip('P'))), fcode)

                        elif line[0] == 'M104' or line[0] == 'M109':
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

                        elif line[0] == 'G20' or line[0] == 'G21':
                            if line == 'G20':  # inch
                                self.unit = 25.4
                            elif line == 'G21':  # mm
                                self.unit = 1

                        elif line[0] == 'G0' or line[0] == 'G1':
                            command = 128
                            subcommand, data = self.XYZEF(line)
                            command |= subcommand
                            self.writer(packer(command), fcode)
                            for i in data:
                                self.writer(struct.pack('<f', i), fcode)

                        elif line[0] == 'T0' or line[0] == 'T1':
                            if line[0] == 'T0':
                                self.tool = 0
                            if line[0] == 'T1':
                                self.tool = 1

                        elif line[0] == 'M107' or line[0] == 'M106':
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
                            pass
                            # print(line)
        fcode.write(struct.pack('<I', self.crc))
        fcode.seek(len(self.header()), 0)
        print(self.script_length)
        fcode.write(struct.pack('<I', self.script_length))
        fcode.seek(0, 2)  # go back to file end
        # self.metadata(fcode)

if __name__ == '__main__':
    m_GcodeToFcode = GcodeToFcode()
    m_GcodeToFcode.process('tmp.gcode')
