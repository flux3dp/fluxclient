
from zipfile import crc32
import struct


def to_uint8(buf):
    return struct.unpack("<B", buf)[0]


def to_uint32(buf):
    return struct.unpack("<I", buf)[0]


def to_float(buf):
    return struct.unpack("<f", buf)[0]


class _ReadHelper(object):
    def __init__(self, stream):
        self.s = stream
        self.crc32 = 0
        self.length = 0

    def read(self, size):
        buf = self.s.read(size)
        self.length += size
        self.crc32 = crc32(buf, self.crc32)
        return buf

    def uint8(self):
        try:
            return to_uint8(self.read(1))
        except struct.error:
            raise ValueError("Contents terminated unexpectedly")

    def uint32(self):
        try:
            return to_uint32(self.read(4))
        except struct.error:
            raise ValueError("Contents terminated unexpectedly")

    def float(self):
        try:
            return to_float(self.read(4))
        except struct.error:
            raise ValueError("Contents terminated unexpectedly")

    def data_length(self):
        try:
            return to_uint32(self.s.read(4))
        except struct.error:
            raise ValueError("Contents terminated unexpectedly")

    def check_crc32(self):
        except_crc32 = self.data_length()
        if except_crc32 != self.crc32:
            raise ValueError("CRC32 not match (except: %i, real: %i)" % (
                             except_crc32, self.crc32))
        self.crc32 = 0
        self.length = 0

    def get_previews(self):
        l = self.data_length()
        while l:
            yield self.s.read(l)
            l = self.data_length()


class FCodeParser(object):
    @classmethod
    def from_file(cls, filename, toolpath_processor):
        with open(filename, "rb") as f:
            return cls.from_stream(f, toolpath_processor)

    @classmethod
    def from_stream(cls, stream, toolpath_processor):
        tp = toolpath_processor
        magic_number = stream.read(8)

        if magic_number != b"FCx0001\n":
            raise ValueError("Bad file header")

        reader = _ReadHelper(stream)
        script_length = reader.data_length()

        while reader.length < script_length:
            cmd = reader.uint8()

            if cmd & 128:
                f = x = y = z = e0 = e1 = e2 = float("nan")
                if cmd & 64:
                    f = reader.float()
                if cmd & 32:
                    x = reader.float()
                if cmd & 16:
                    y = reader.float()
                if cmd & 8:
                    z = reader.float()
                if cmd & 4:
                    e0 = reader.float()
                if cmd & 2:
                    e1 = reader.float()
                if cmd & 1:
                    e2 = reader.float()
                tp.moveto(feedrate=f, x=x, y=y, z=z, e0=e0, e1=e1, e2=e2)
            elif cmd & 64:
                # No use, just ignore
                flag = 32
                while flag:
                    if cmd & flag:
                        reader.float()
                    flag >>= 1
            elif (cmd & 48) == 48:
                tp.set_toolhead_fan_speed(reader.float())
            elif cmd & 32:
                tp.set_toolhead_pwm(reader.float())
            elif cmd & 16:
                block = True if cmd & 8 else False
                tp.set_toolhead_heater_temperature(reader.float(), block)
            elif cmd == 6:
                tp.pause(True)
            elif cmd == 5:
                tp.pause(False)
            elif cmd & 4:
                tp.sleep(reader.float() / 1000.0)
            else:
                tp.on_error(True, "Can not handle command id: %i" % cmd)

        if reader.length != script_length:
            raise ValueError("Script body size error (%s, %s)" % (reader.length, script_length))

        reader.check_crc32()

        metadata_length = reader.data_length()
        metadata_buf = reader.read(metadata_length)
        reader.check_crc32()
        previews = tuple(reader.get_previews())

        keyvalues = map(lambda kv: kv.split("=", 1),
                        metadata_buf.decode("utf8").split("\x00"))
        metadata = {}
        for k, v in map(lambda i: i if len(i) == 2 else (i[0], None),
                        keyvalues):
            if k:
                metadata[k] = v
                tp.append_comment("%s=%s" % (k, v))

        return metadata, previews
