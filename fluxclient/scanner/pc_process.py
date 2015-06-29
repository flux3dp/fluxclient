#!/usr/bin/env python3
import struct
from operator import ge, le


import fluxclient.scanner.scan_settings as scan_settings
try:
    import fluxclient.scanner._scanner as _scanner
except:
    pass


class pc_process():
    """process point cloud"""
    def __init__(self):
        self.clouds = {}  # clouds that hold all the point cloud data, key:name, value:point cloud

    def upload(self, name, buffer_pc_L, buffer_pc_R, L_len, R_len):
        self.clouds[name] = (self.unpack_data(buffer_pc_L), self.unpack_data(buffer_pc_R))
        print('upload %s,L: %d R: %d' % (name, len(self.clouds[name][1]), len(self.clouds[name][1])))
        print('all:' + " ".join(self.clouds.keys()))
        # upload [name] [point count L] [point count R]

    def unpack_data(self, buffer_data):
        """
            unpack buffer data into [[x, y, z, r, g, b]]
        """
        assert len(buffer_data) % 24 == 0, "wrong buffer size %d (can't devide by 24)" % (len(buffer_data) % 24)
        pc = []
        for p in range(int(len(buffer_data) / 24)):
            tmp_point = list(struct.unpack('<ffffff', buffer_data[p * 24:p * 24 + 24]))
            tmp_point[3] = round(tmp_point[3] * 255)
            tmp_point[4] = round(tmp_point[4] * 255)
            tmp_point[5] = round(tmp_point[5] * 255)
            pc.append(tmp_point)
        return pc

    def base(self, name):
        self.current_name = name

    def cut(self, name_in, name_out, mode, direction, value):
        """
            manually cut the point cloud
            mode = 'x', 'y', 'z' ,'r'
            direction = True(>=), False(<=)
            [FUTURE WORK] transplant to cpp in future to spped up
        """
        pc = self.clouds[name_in]
        cropped_pc = []

        if direction:  # ge = >=, le = <=
            cmp_function = ge
        else:
            cmp_function = le

        if mode == 'r':
            for p in pc:
                if cmp_function(p[0] ** 2 + p[1] ** 2, thres ** 2):
                    cropped_pc.append(p)
            return

        elif mode == 'x':
            index = 0
        elif mode == 'y':
            index = 1
        elif mode == 'z':
            index = 2
        for p in pc:
            if cmp_function(p[index], thres):
                cropped_pc.append(p)

        self.clouds[name_out] = cropped_pc

    def to_cpp(pc_python):
        """
        convert python style pc into cpp style pc object
        """
        pc = _scanner.PointCloudXYZRGBObj()
        for i in pc_python:
            _scanner.push_backPoint(pc, i[0], i[1], i[2], i[3] | (i[4] << 8) | (i[5] << 16))
        return pc

    def noise_del(self, name_in, name_out, r):
        """
        delete noise base on distance of each point
        pc_source could be a string indcating the point cloud that we want
        """
        # if type(pc_source) == str:
        #     pc = _scanner.PointCloudXYZRGBObj()
        #     pc.load(pc_source)
        # else:
        #     pc = pc_source

        pc = self.clouds[name_in]
        pc = self.to_cpp(pc)
        pc.SOR(50, 0.3)
        self.clouds[name_out] = pc
        return 0

    def to_mesh(self):
        pass

    def dump(self, name):
        pc_both = self.clouds[name]
        buffer_data = []

        for pc in pc_both:
            for p in pc:
                buffer_data.append(struct.pack('<ffffff', p[0], p[1], p[2], p[3] / 255., p[4] / 255., p[5] / 255.))
        buffer_data = b''.join(buffer_data)
        assert (len(pc_both[0]) + len(pc_both[1])) * 24 == len(buffer_data), "dumping error!"
        return len(pc_both[0]), len(pc_both[1]), buffer_data
