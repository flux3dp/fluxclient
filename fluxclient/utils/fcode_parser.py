
from zipfile import crc32
import struct
import sys

FILE_BROKEN = "FILE_BROKEN"
uint_unpacker = lambda x: struct.Struct("<I").unpack(x)[0]  # 4 bytes uint
uchar_unpacker = lambda x: struct.Struct("<B").unpack(x)[0]  # 1 byte uchar, use for command
float_unpacker = lambda x: struct.Struct("<f").unpack(x)[0]  # 4 bytes float


def XYZE(command):
    '[F, X, Y, Z, E1, E2, E3]'
    tmp = [None for _ in range(7)]
    c = 0
    while c < 7:
        if command & (1 << c):
            tmp[-c - 1] = True
        c += 1
    return tmp


class FcodeParser(object):
    """
    https://github.com/flux3dp/fluxmonitor/wiki/Flux-Device-Control-Describe-File-V1
    """
    def __init__(self):
        super(FcodeParser, self).__init__()
        self.data = None
        self.current_T = 0

    def upload_content(self, buf):
        if type(buf) == bytes:
            tmp_data = self.data
            self.data = buf
            if self.full_check():
                return True
            else:
                self.data = tmp_data
                return False
        elif type(buf) == str:
            with open(sys.argv[1], 'rb') as f:
                return self.upload_content(f.read())

    def full_check(self):
        try:
            assert self.data[:8] == b"FCx0001\n"
            self.script_size = uint_unpacker(self.data[8:12])
            assert crc32(self.data[12:12 + self.script_size]) == uint_unpacker(self.data[12 + self.script_size:16 + self.script_size])
            index = 16 + self.script_size
            self.meta_size = uint_unpacker(self.data[index:index + 4])
            index += 4
            assert crc32(self.data[index:index + self.meta_size]) == uint_unpacker(self.data[index + self.meta_size:index + self.meta_size + 4])
            index = index + self.meta_size + 4
            self.image_size = uint_unpacker(self.data[index:index + 4])
            index += 4
            return True
        except AssertionError as e:
            raise RuntimeError(FILE_BROKEN, e.args[0] if e.args else "#")

    def get_img(self):
        if self.data:
            return self.data[28 + self.script_size + self.meta_size:28 + self.script_size + self.meta_size + self.image_size]
        else:
            return None

    def get_meta(self):
        if self.data:

            meta_buf = self.data[20 + self.script_size:20 + self.script_size + self.meta_size]
            metadata = {}
            outstring = []
            for item in meta_buf.split(b"\x00"):
                sitem = item.split(b"=", 1)
                if len(sitem) == 2:
                    metadata[sitem[0].decode()] = sitem[1].decode()
                    outstring.append('"%s": "%s"' % (sitem[0].decode(), sitem[1].decode()))
            return '{' + ', '.join(outstring) + '}'
        else:
            return None

    def get_path(self):  # TODO
        pass

    def f_to_g(self, outstream):
        index = 12
        self.data[12:12 + self.script_size]
        while index < 12 + self.script_size:
            command = uchar_unpacker(self.data[index:index + 1])
            index += 1
            if command == 1:
                outstream.write('G28\n')
            elif command == 2:
                outstream.write('G90\n')
            elif command == 3:
                outstream.write('G91\n')
            elif command == 4:
                outstream.write('G4 P{}\n'.format(float_unpacker(self.data[index:index + 4])))
                index += 4
            elif command == 5:
                print('find pause fcode', file=sys.stderr)

            elif command == 6:
                outstream.write('; raw command to mb\n')
                start = index
                while self.data[index: index + 1] != b'\n':
                    index += 1
                index += 1
                outstream.write(self.data[start:index])
                outstream.write('; raw command to mb end.\n')

            elif command == 7:
                outstream.write('; raw command to print head\n')
                start = index
                while self.data[index] != '\n':
                    index += 1
                index += 1
                outstream.write(self.data[start:index])
                outstream.write('; raw command to print head end.\n')

            elif command >= 16 and command <= 31:  # temperature
                if command & 8:
                    c = 'M109'
                else:
                    c = 'M104'
                temp = float_unpacker(self.data[index:index + 4])
                if temp == float('-inf'):
                    temp = 0
                outstream.write('{} T{} S{}\n'.format(c, command & 7, temp))  # not T but P????
                index += 4
            elif command >= 32 and command <= 39:  # laser
                strength = float_unpacker(self.data[index:index + 4])
                strength = round(strength * 255)
                outstream.write('X2O{} T{}\n'.format(strength, command & 7))
                index += 4
            elif command >= 48 and command <= 63:  # fan speed
                speed = float_unpacker(self.data[index:index + 4])
                speed = round(speed * 255)
                if speed == 0:
                    c = 'M107'
                else:
                    c = 'M106'
                outstream.write('{} S{} T{}\n'.format(c, speed, command & 15))
                index += 4
            elif command >= 64 and command <= 127:  # set position
                tmp = XYZE(command)
                s = ['G92 ']
                for i, c in zip(tmp[:3], ['X', 'Y', 'Z']):
                    if i:
                        value = float_unpacker(self.data[index:index + 4])
                        index += 4
                        s.append('{}{} '.format(c, value))
                s.append('\n')
                outstream.write(''.join(s))

                for i in range(4, 6):
                    if tmp[i]:
                        if self.current_T != i - 4:
                            outstream.write('T{}\n'.format(i - 4))
                            self.current_T = i - 4
                        value = float_unpacker(self.data[index:index + 4])
                        index += 4
                        outstream.write('G92 E{}\n'.format(value))

            elif command >= 128 and command <= 255:  # moving
                tmp = XYZE(command)
                s = ['G1 ']
                for i, c in zip(tmp, ['F', 'X', 'Y', 'Z', 'E', 'E', 'E']):
                    if i:
                        value = float_unpacker(self.data[index:index + 4])
                        index += 4
                        if c == 'E':
                            s.append('{}{:.5f} '.format(c, value))
                        else:
                            s.append('{}{:.3f} '.format(c, value))
                s.append('\n')
                outstream.write(''.join(s))
            else:
                print('something wrong')


if __name__ == '__main__':
    m_FcodeParser = FcodeParser()
    with open(sys.argv[1], 'rb') as f, open('tmp.gcode', 'w') as f2:
        m_FcodeParser.upload_content(f.read())
        m_FcodeParser.f_to_g(f2)
        print(m_FcodeParser.get_meta())
