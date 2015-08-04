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

    def process(self, file_name):
        # fcode = tempfile.NamedTemporaryFile(suffix='.fcode', delete=False)
        fcode = open('GG.fcode', 'wb')
        fcode.write(self.header())

        packer = lambda x: struct.pack('<B', x)  # due to appear so many times, use this as a alias for 'struct.pack('<B', x)'

        fcode.write(struct.pack('<I', 0))  # script length
        script_length = 0

        with open(file_name, 'r') as f:
            if file_name[-6:] != '.gcode':
                raise ValueError('Unrecognized file format%s' % (file_name))
            for line in f:
                if ';' in line:
                    line = line[:line.index(';')].rstrip()
                    line = line.split()
                    if line:
                        if line[0] == 'G28':
                            fcode.write(packer(1))
                            script_length += 1
                        elif line[0] == 'G90':
                            fcode.write(packer(2))
                            self.absolute = True
                            script_length += 1
                        elif line[0] == 'G91':
                            fcode.write(packer(3))
                            self.absolute = False
                            script_length += 1
                        elif line[0] == 'M82':
                            self.extrude_absolute = True
                        elif line[0] == 'M83':
                            self.extrude_absolute = False

                        elif line[0] == 'G92':
                            command = 64
                            sub_command, data = self.XYZEF(line)
                            command |= sub_command
                            fcode.write(packer(command))
                            for i in data:
                                fcode.write(struct.pack('<f', i))
                            script_length += 1 + (4 * len(data))

                        elif line[0] == 'G4':
                            fcode.write(packer(4))
                            fcode.write(struct.pack('<f', float(line[1].lstrip('P'))))
                            script_length += 1 + 4

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
                            fcode.write(packer(command))
                            fcode.write(struct.pack('<f', temp))
                            script_length += 1 + 4

                        elif line[0] == 'G20' or line[0] == 'G21':
                            if line == 'G20':  # inch
                                self.unit = 25.4
                            elif line == 'G21':  # mm
                                self.unit = 1
                        elif line[0] == 'G0' or line[0] == 'G1':
                            command = 128
                            subcommand, data = self.XYZEF(line)
                            command |= subcommand
                            fcode.write(packer(command))
                            for i in data:
                                fcode.write(struct.pack('<f', i))
                            script_length += 1 + (4 * len(data))

                        elif line[0] == 'T0' or line[0] == 'T1':
                            if line[0] == 'T0':
                                self.tool = 0
                            if line[0] == 'T1':
                                self.tool = 1
                        elif line[0] == 'M107' or line[0] == 'M106':
                            command = 48
                            command |= 1  # TODO: change this part
                            fcode.write(packer(command))
                            if line[0] == 'M107':
                                fcode.write(struct.pack('<f', 0.0))
                            elif line[0] == 'M106':
                                fcode.write(struct.pack('<f', float(line[1].lstrip('S'))))
                            script_length += 1 + 4

                        elif line[0] == 'M84':  # loosen the motor
                            pass  # should only appear when printing done
                        else:
                            pass
                            # print(line)
        fcode.seek(len(self.header()), 0)
        fcode.write(struct.pack('<I', script_length))
        fcode.seek(0, 2)  # go back to file end
        self.metadata(fcode)

if __name__ == '__main__':
    m_GcodeToFcode = GcodeToFcode()
    m_GcodeToFcode.process('tmp.gcode')
