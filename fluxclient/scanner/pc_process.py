#!/usr/bin/env python3
import struct
from operator import ge, le
import logging
import io
import sys

import fluxclient.scanner.scan_settings as scan_settings
from fluxclient.scanner.tools import write_stl, write_pcd

try:
    import fluxclient.scanner._scanner as _scanner
except:
    pass

logger = logging.getLogger(__name__)


class PcProcess():
    """process point cloud"""
    def __init__(self):
        self.clouds = {}  # clouds that hold all the point cloud data, key:name, value:point cloud
        self.meshs = {}

    def upload(self, name, buffer_pc_L, buffer_pc_R, L_len, R_len):
        self.clouds[name] = (self.unpack_data(buffer_pc_L), self.unpack_data(buffer_pc_R))
        logger.debug('upload %s,L: %d R: %d' % (name, len(self.clouds[name][0]), len(self.clouds[name][1])))
        logger.debug('all:' + " ".join(self.clouds.keys()))
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

    # def base(self, name):
    #     self.current_name = name

    def cut(self, name_in, name_out, mode, direction, value):
        """
            manually cut the point cloud
            mode = 'x', 'y', 'z' ,'r'
            direction = True(>=), False(<=)

            TODO: transplant to cpp in future to spped up
        """
        logger.debug('cut name_in[%s] name_out[%s] mode[%s] direction[%s] value[%.4f]' % (name_in, name_out, mode, direction, value))

        pc_both = self.clouds[name_in]
        pc_both_o = []
        for pc in pc_both:
            cropped_pc = []
            if direction:  # ge = >=, le = <=
                cmp_function = ge
            else:
                cmp_function = le

            if mode == 'r':
                for p in pc:
                    if cmp_function(p[0] ** 2 + p[1] ** 2, value ** 2):
                        cropped_pc.append(p)
                pc_both_o.append(cropped_pc)
                continue

            elif mode == 'x':
                index = 0
            elif mode == 'y':
                index = 1
            elif mode == 'z':
                index = 2
            else:
                raise ValueError('Undefine cutting mode: %s ' % mode)
            for p in pc:
                if cmp_function(p[index], value):
                    cropped_pc.append(p)
            pc_both_o.append(cropped_pc)
        self.clouds[name_out] = pc_both_o

    def to_cpp(self, pc_both):
        """
        convert python style pc into cpp style pc object
        """
        tmp = []
        for pc_python in pc_both:
            pc = _scanner.PointCloudXYZRGBObj()
            for i in pc_python:
                pc.push_backPoint(i[0], i[1], i[2], (i[3] << 16) | (i[4] << 8) | i[5])
                # pc.push_backPoint(i[0], i[1], i[2], (255 << 16))
            tmp.append(pc)
        logger.debug('to_cpp done')
        return tmp

    def delete_noise(self, name_in, name_out, stddev):
        """
        delete noise base on distance of each point
        pc_source could be a string indcating the point cloud that we want

        """
        logger.debug('delete_noise [%s] [%s] [%.4f]' % (name_in, name_out, stddev))
        pc_both = self.clouds[name_in]

        if type(pc_both[0]) == list:
            pc_both = self.to_cpp(self.clouds[name_in])
        else:
            logger.debug('%s' % str(type(pc_both[0])))
            pc_both = [i.clone() for i in pc_both]
            logger.debug('%s' % str(type(pc_both[0])))
        for pc in pc_both:
            logger.debug('start with %d point' % pc.get_w())
            pc.SOR(50, stddev)  # TODO: put magic number away
            logger.debug('finished with %d point' % pc.get_w())

        self.clouds[name_out] = pc_both

        return 0

    def to_mesh(self, name_in):
        pc_both = self.clouds[name_in]
        if type(pc_both[0]) == list:
            pc_both = self.to_cpp(self.clouds[name_in])
        else:
            pc_both = [i.clone() for i in pc_both]

            pc = pc_both[0].clone()
            pc.add(pc_both[1])

        for pc in pc_both[1:]:
            pc.ne()
            # pc.ne_viewpoint()  # should use this one in the future
            pc_new = pc.to_mesh()

            self.clouds['wth'] = pc_new
            a = pc.STL_to_Faces()
            print(len(a))
            m_mesh = mesh('wth', a, self.clouds)
        buf = io.StringIO()
        write_stl(m_mesh, buf, 'ascii')
        return buf.getvalue().encode()

    def dump(self, name):
        """
        dump the indicated(name) cloud
        """
        logger.debug('dumping' + name)

        pc_both = self.clouds[name]
        # print(len(pc_both), len(self.clouds[name][0]), len(self.clouds[name][1]))
        buffer_data = []

        if type(pc_both[0]) == list:
            for pc in pc_both:
                for p in pc:
                    buffer_data.append(struct.pack('<ffffff', p[0], p[1], p[2], p[3] / 255., p[4] / 255., p[5] / 255.))
            buffer_data = b''.join(buffer_data)
            assert (len(pc_both[0]) + len(pc_both[1])) * 24 == len(buffer_data), "dumping error!"
            return len(pc_both[0]), len(pc_both[1]), buffer_data

        else:
            pc_size = []
            for pc in pc_both:
                pc_size.append(pc.get_w())
                for p_i in range(pc_size[-1]):
                    p = pc.get_item(p_i)
                    buffer_data.append(struct.pack('<ffffff', p[0], p[1], p[2], p[3] / 255., p[4] / 255., p[5] / 255.))
                    # buffer_data.append(struct.pack('<ffffff', p[0], p[1], p[2], 0 / 255., 0 / 255., 0 / 255.))
            buffer_data = b''.join(buffer_data)
            return pc_size[0], pc_size[1], buffer_data

    def export(self, name, file_format):
        if file_format == 'pcd':
            pc_both = self.clouds[name]
            if type(pc_both[0]) == list:
                pc_add = pc_both[0] + pc_both[0]
            else:
                pc_add = []
                pc_size = []
                for pc in pc_both:
                    pc_size.append(pc.get_w())
                    for p_i in range(pc_size[-1]):
                        pc_add.append(pc_both.get_item(p_i))
            tmp = io.StringIO()
            write_pcd(pc_add, tmp)
            return tmp.getvalue()

        elif file_format == 'stl':
            return self.to_mesh(name)

    def merge(self, name_base, name_2, x, y, z, rx, ry, rz, name_out):
        self.clouds[name_out] = self.clouds[name_2]
        # not done yet
        return True

    def auto_merge(self, name_base, name_2, name_out):
        for name in [name_base, name_2]:
            if type(self.clouds[name][0]) == list:
                self.clouds[name] = self.to_cpp(self.clouds[name])
        pc_both = []
        for i in range(1, 2):  # fake code
        # TODO: should ignore avoid zero input point
            reg = _scanner.RegCloud(self.clouds[name_base][i], self.clouds[name_2][i])
            result, pc = reg.SCP()
            pc_both.append(pc)
        self.clouds[name_out] = pc_both
        return True


class mesh(object):
    """mesh"""
    def __init__(self, cloud_name, index_list, clouds):
        super(mesh, self).__init__()
        self.cloud_name = cloud_name
        self.faces = index_list
        self.clouds = clouds
        self.cur = 0

    def __len__(self):
        return len(self.faces)

    def __iter__(self):
        return self

    def __next__(self):
        if self.cur < len(self.faces):
            face = [self.clouds[self.cloud_name].get_item(index) for index in self.faces[self.cur]]
            self.cur += 1
            return face
        else:
            raise StopIteration


class PcProcessNoPCL(PcProcess):
    """docstring for PcProcessNoPCL"""
    def __init__(self):
        super(PcProcessNoPCL, self).__init__()

    def delete_noise(self, name_in, name_out, stddev):
        """
        delete noise base on distance of each point
        pc_source could be a string indcating the point cloud that we want

        """
        logger.debug('delete_noise [%s] [%s] [%.4f]' % (name_in, name_out, stddev))
        pc_both = self.clouds[name_in]
        pc_both_o = []
        for pc in pc_both:
            pc_o = []
            for p in pc:
                # pc_o.append([p[0], p[1], p[2], len(pc_both), 0, 0])
                pc_o.append([p[0], p[1], p[2], 255 - 255 * len(pc_both_o), 0, 0])
            pc_both_o.append(pc_o)

        pc_both_o.reverse()
        self.clouds[name_out] = pc_both_o
        return 0

    def merge(self, name_base, name_2, x, y, z, rx, ry, rz, name_out):
        self.clouds[name_out] = self.clouds[name_base]  # TODO: clone
        return True

    def export(self, name, file_format):
        return b"FLUX 3d printer: flux3dp.com, 2015                                              \x0c\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00"
