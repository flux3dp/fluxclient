
from zipfile import crc32
import struct
import sys

FILE_BROKEN = "FILE_BROKEN"
UINT_UNPACKER = lambda x: struct.Struct("<I").unpack(x)[0]


class FcodeParser(object):
    """docstring for FcodeParser"""
    def __init__(self):
        super(FcodeParser, self).__init__()
        self.data = None

    def upload_content(self, buf):
        if type(buf) == bytes:  # TODO:what about use file name
            tmp_data = self.data
            self.data = buf
            if self.full_check():
                return True
            else:
                self.data = tmp_data
                return False

    def full_check(self):
        try:
            assert self.data[:8] == b"FCx0001\n"
            self.script_size = UINT_UNPACKER(self.data[8:12])
            assert crc32(self.data[12:12 + self.script_size]) == UINT_UNPACKER(self.data[12 + self.script_size:16 + self.script_size])
            index = 16 + self.script_size
            self.meta_size = UINT_UNPACKER(self.data[index:index + 4])
            index += 4
            assert crc32(self.data[index:index + self.meta_size]) == UINT_UNPACKER(self.data[index + self.meta_size:index + self.meta_size + 4])
            index = index + self.meta_size + 4
            self.image_size = UINT_UNPACKER(self.data[index:index + 4])
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

    def get_path():  # TODO
        pass

if __name__ == '__main__':
    m_FcodeParser = FcodeParser()
    with open(sys.argv[1], 'rb') as f:
        m_FcodeParser.upload_content(f.read())
        print(m_FcodeParser.get_meta())

        # m_FcodeParser.full_check()
