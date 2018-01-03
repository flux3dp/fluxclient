# !/usr/bin/env python3

from zipfile import crc32
import struct
import sys
from threading import Thread

from fluxclient.fcode.fcode_base import FcodeBase
from fluxclient.hw_profile import HW_PROFILE
import logging

logger = logging.getLogger("F_TO_G")

FILE_BROKEN = "FILE_BROKEN"
FCODE_FAIL = "FCODE_FAIL"
uint_unpacker = lambda x: struct.Struct("<I").unpack(x)[0]  # 4 bytes uint
uchar_unpacker = lambda x: struct.Struct("<B").unpack(x)[0]  # 1 byte uchar, use for command
float_unpacker = lambda x: struct.Struct("<f").unpack(x)[0]  # 4 bytes float


def num_to_XYZE(command):
    """
    convert a number to information about each axis
    command[input]: a encoded number
    return things look like this [True, True, True, None, None, None, True]

    in [F, X, Y, Z, E1, E2, E3] order

    """
    tmp = [None for _ in range(7)]
    c = 0
    while c < 7:
        if command & (1 << c):
            tmp[-c - 1] = True
        c += 1
    return tmp


class FcodeToGcode(FcodeBase):
    def __init__(self, buf=''):
        super(FcodeToGcode, self).__init__()
        self.data = None
        self.metadata = None

        self.current_T = 0
        self.extrudeflag = False
        self.laserflag = False

        if buf:
            self.upload_content(buf)

    def upload_content(self, buf, model='model-1'):
        """
        upload fcode content in this object,
        buf[in]: could be string indicating the path to .fcode or bytes
        bool [return]: retrun bool showing whether it's valid fcode
        """
        if type(buf) == bytes:
            tmp_data = self.data
            self.data = buf
            if self.full_check():
                md = self.get_metadata()
                if model == 'beambox':
                    if float(md.get('MAX_X', 0)) > HW_PROFILE[model]['width']:
                        return 'out_of_bound'
                    elif float(md.get('MAX_Y', 0)) > HW_PROFILE[model]['length']:
                        return 'out_of_bound'
                    elif float(md.get('MAX_Z', 0)) > HW_PROFILE[model]['height'] or float(md.get('MAX_Z', 0)) < 0:
                        return 'out_of_bound'
                    else:
                        return 'ok'
                else:
                    if float(md.get('MAX_X', 0)) > HW_PROFILE[model]['radius']:
                        return 'out_of_bound'
                    elif float(md.get('MAX_Y', 0)) > HW_PROFILE[model]['radius']:
                        return 'out_of_bound'
                    elif float(md.get('MAX_R', 0)) > HW_PROFILE[model]['radius']:
                        return 'out_of_bound'
                    elif float(md.get('MAX_Z', 0)) > HW_PROFILE[model]['height'] or float(md.get('MAX_Z', 0)) < 0:
                        return 'out_of_bound'
                    else:
                        return 'ok'
            else:
                self.data = tmp_data
                return 'broken'
        elif type(buf) == str:
            with open(sys.argv[1], 'rb') as f:
                return self.upload_content(f.read())

    def full_check(self):
        """
        fully check the fcode file, including all the checksum
        raise error if any error occur
        """
        try:
            logger.info("Start Checking CRC %s" % str(self.data[:8]));
            assert self.data[:8] == b"FCx0001\n"
            logger.info("Passed Header Check");
            self.script_size = uint_unpacker(self.data[8:12])
            logger.info("Script size = " + str(self.script_size))
            assert crc32(self.data[12:12 + self.script_size]) == uint_unpacker(self.data[12 + self.script_size:16 + self.script_size])
            logger.info("Passed CRC32 -1 ");
            index = 16 + self.script_size
            self.meta_size = uint_unpacker(self.data[index:index + 4])
            index += 4
            assert crc32(self.data[index:index + self.meta_size]) == uint_unpacker(self.data[index + self.meta_size:index + self.meta_size + 4])
            logger.info("Passed CRC32 -2 ");
            index = index + self.meta_size + 4
            self.image_size = uint_unpacker(self.data[index:index + 4])
            index += 4
            logger.info("Passed full check");
            return True
        except AssertionError as e:
            logger.info(str(e));
            return False

    def get_img(self):
        """
        get the .png preview image(should be 640 * 640) in fcode, in bytes
        """
        if self.data:
            return self.data[28 + self.script_size + self.meta_size:28 + self.script_size + self.meta_size + self.image_size]
        else:
            return None

    def change_img(self, buf):
        """
        change the preview image in fcode file
        """
        if self.data:
            self.data = b''.join([self.data[:-(self.image_size + 4)], struct.pack('<I', len(buf)), buf])
            self.image_size = len(buf)
            return True
        else:
            return False

    def writeStr(self, o, s):
        o.write(s);

    def get_metadata(self):
        """
        get the metadata
        dict [return]: metadata, a dict object like this {"AUTHOR": "Yen", "HEAD_TYPE": "EXTRUDER"}
        """
        if self.data:
            if self.metadata is None:
                meta_buf = self.data[20 + self.script_size:20 + self.script_size + self.meta_size]
                metadata = {}
                for item in meta_buf.split(b"\x00"):
                    itme = item.split(b"=", 1)
                    if len(itme) == 2:
                        metadata[itme[0].decode()] = itme[1].decode()
                self.metadata = metadata
        return self.metadata

    def f_to_g(self, outstream, include_meta=False):
        self.path_js = None
        if include_meta:
            meta = self.get_metadata()
            for key, value in meta.items():
                outstream.write(bytes(";%s=%s\n" % (key, value), "UTF-8"))
            outstream.write(bytes("\n", "UTF-8"))

        index = 12
        while index < 12 + self.script_size:
            command = uchar_unpacker(self.data[index:index + 1])
            index += 1
            if command == 1:
                self.writeStr(outstream, 'G28\n')
            elif command == 2:
                self.writeStr(outstream, 'G90\n')
            elif command == 3:
                self.writeStr(outstream, 'G91\n')
            elif command == 4:
                self.writeStr(outstream, 'G4 P{}\n'.format(float_unpacker(self.data[index:index + 4])))
                index += 4
            elif command == 5:
                pass

            elif command == 6:
                self.writeStr(outstream, '; raw command to mb\n')
                start = index
                while self.data[index: index + 1] != b'\n':
                    index += 1
                index += 1
                self.writeStr(outstream, elf.data[start:index])
                self.writeStr(outstream, '; raw command to mb end.\n')

            elif command == 7:
                self.writeStr(outstream, '; raw command to print head\n')
                start = index
                while self.data[index] != '\n':
                    index += 1
                index += 1
                self.writeStr(outstream, elf.data[start:index])
                self.writeStr(outstream, '; raw command to print head end.\n')

            elif command >= 16 and command <= 31:  # temperature
                if command & 8:
                    c = 'M109'
                else:
                    c = 'M104'
                temp = float_unpacker(self.data[index:index + 4])
                if temp == float('-inf'):
                    temp = 0
                self.writeStr(outstream, '{} T{} S{}\n'.format(c, command & 7, temp))  # not T but P????
                index += 4
            elif command >= 32 and command <= 39:  # laser
                strength = float_unpacker(self.data[index:index + 4])
                strength = round(strength * 255)
                if strength > 0:
                    self.laserflag = True
                else:
                    self.laserflag = False
                self.writeStr(outstream, 'X2O{} T{}\n'.format(strength, command & 7))
                index += 4
            elif command >= 48 and command <= 63:  # fan speed
                speed = float_unpacker(self.data[index:index + 4])
                speed = round(speed * 255)
                if speed == 0:
                    c = 'M107'
                else:
                    c = 'M106'
                self.writeStr(outstream, '{} S{} T{}\n'.format(c, speed, command & 15))
                index += 4
            elif command >= 64 and command <= 127:  # set position
                tmp = num_to_XYZE(command)
                s = ['G92 ']
                for i, c in zip(tmp[:3], ['X', 'Y', 'Z']):
                    if i:
                        value = float_unpacker(self.data[index:index + 4])
                        index += 4
                        s.append('{}{} '.format(c, value))
                s.append('\n')
                self.writeStr(outstream, ''.join(s))

                for i in range(4, 6):
                    if tmp[i]:
                        if self.current_T != i - 4:
                            self.writeStr(outstream, 'T{}\n'.format(i - 4))
                            self.current_T = i - 4
                        value = float_unpacker(self.data[index:index + 4])
                        index += 4
                        self.writeStr(outstream, 'G92 E{}\n'.format(value))

            elif command >= 128 and command <= 255:  # moving
                line_segment = num_to_XYZE(command)
                s = ['G1 ']
                for f, c, i in zip(line_segment, ['F', 'X', 'Y', 'Z', 'E', 'E', 'E'], range(7)):
                    if f is True:
                        value = float_unpacker(self.data[index:index + 4])
                        index += 4
                        if c == 'E':
                            s.append('{}{:.5f} '.format(c, value))
                        else:
                            s.append('{}{:.3f} '.format(c, value))
                        if i >= 1:
                            self.current_pos[i - 1] = value
                s.append('\n')
                self.writeStr(outstream, ''.join(s))

                if any(i is not None for i in line_segment[4:6]):
                    self.extrudeflag = True
                else:
                    self.extrudeflag = False
                move_flag = any(i is not None for i in line_segment[1:4])
                self.process_path('', move_flag, self.laserflag or self.extrudeflag)
            else:
                raise RuntimeError("FCODE_FAIL unknown command "+ str(command) + " @"+str(index))
        self.T = Thread(target=self.sub_convert_path)
        self.T.start()
