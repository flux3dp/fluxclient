#!/usr/bin/env python3
import struct
from operator import ge, le
import logging
import io
import sys

from . import scan_settings
from .tools import write_stl, write_pcd


from . import _scanner


logger = logging.getLogger(__name__)


class PcProcess():
    """process point cloud"""
    def __init__(self):
        self.clouds = {}  # clouds that hold all the point cloud data, key:name, value:point cloud
        self.meshs = {}

    def upload(self, name, buffer_pc_L, buffer_pc_R, L_len, R_len):
        """
        # upload [name] [point count L] [point count R]
        """
        self.clouds[name] = self.to_cpp((self.unpack_data(buffer_pc_L), self.unpack_data(buffer_pc_R)))
        logger.debug('upload %s, L: %d R: %d' % (name, len(self.clouds[name][0]), len(self.clouds[name][1])))
        logger.debug('all:' + " ".join(self.clouds.keys()))

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

    def to_cpp(self, pc_both):
        """
        convert python style pc into cpp style pc object
        """
        c_both = []
        for pc_python in pc_both:
            pc = _scanner.PointCloudXYZRGBObj()
            for i in pc_python:
                pc.push_backPoint(*i)  # pc.push_backPoint(i[0], i[1], i[2], i[3], i[4], i[5])
            # ne(): normal estimate
            # ne_viewpoint() :normal estimate considering view point
            # ref: http://pointclouds.org/documentation/tutorials/normal_estimation.php
            # pc.ne_viewpoint()
            c_both.append(pc)
        logger.debug('to_cpp done')
        return c_both

    def cut(self, name_in, name_out, mode, direction, value):
        """
            manually cut the point cloud
            mode = 'x', 'y', 'z' ,'r'
            direction = True(>=), False(<=)
        """
        logger.debug('cut name_in[%s] name_out[%s] mode[%s] direction[%s] value[%.4f]' % (name_in, name_out, mode, direction, value))
        pc_both = self.clouds[name_in]

        self.clouds[name_out] = []
        for pc in pc_both:
            self.clouds[name_out].append(pc.cut('xyzr'.index(mode), 1 if direction else 0, value))

    def delete_noise(self, name_in, name_out, stddev):
        """
        delete noise base on distance of each point
        pc_source could be a string indcating the point cloud that we want

        """
        logger.debug('delete_noise [%s] [%s] [%.4f]' % (name_in, name_out, stddev))
        pc_both = [i.clone() for i in self.clouds[name_in]]

        for pc in pc_both:
            logger.debug('start with %d point' % len(pc))
            pc.SOR(scan_settings.SOR_neighbors, stddev)
            logger.debug('finished with %d point' % len(pc))

        self.clouds[name_out] = pc_both

    def to_mesh(self, name_in):
        logger.debug('to_mesh name:%s' % name_in)
        pc_both = self.clouds[name_in]

        # WARNING: merge L and R here!
        # pc = pc_both[0].clone()
        pc = pc_both[0].add(pc_both[1])
        # pc.ne_viewpoint()
        pc.ne()
        pc.to_mesh()  # compute mesh
        return pc

    def dump(self, name):
        """
        dump the indicated(name) cloud
        """
        logger.debug('dumping ' + name)

        pc_both = self.clouds[name]
        buffer_data = []

        pc_size = []
        for pc in pc_both:
            l = len(pc)
            pc_size.append(l)
            for p_i in range(l):  # ?????
                p = pc.get_item(p_i)
                buffer_data.append(struct.pack('<ffffff', p[0], p[1], p[2], p[3] / 255., p[4] / 255., p[5] / 255.))
                # buffer_data.append(struct.pack('<ffffff', p[0], p[1], p[2], 0 / 255., 0 / 255., 0 / 255.))
        buffer_data = b''.join(buffer_data)
        return pc_size[0], pc_size[1], buffer_data

    def export(self, name, file_format, mode='binary'):
        if file_format == 'pcd':
            pc_both = self.clouds[name]
            # WARNING: merge L and R here!
            pc_add = []
            pc_size = []
            for pc in pc_both:
                pc_size.append(len(pc))
                for p_i in range(pc_size[-1]):
                    pc_add.append(pc.get_item(p_i))
            tmp = io.StringIO()
            write_pcd(pc_add, tmp)
            return tmp.getvalue().encode()

        elif file_format == 'stl':
            pc_mesh = self.to_mesh(name)

            if mode == 'ascii':
                buf = io.StringIO()
                write_stl(pc_mesh.STL_to_List(), buf, mode)
                return strbuf.getvalue().encode()

            elif mode == 'binary':
                buf = io.BytesIO()
                write_stl(pc_mesh.STL_to_List(), './output.stl', mode)

                write_stl(pc_mesh.STL_to_List(), buf, mode)
                return buf.getvalue()

    def apply_transform(self, name_in, x, y, z, rx, ry, rz, name_out):
        both_pc = []
        for pc in self.clouds[name_in]:
            pc_new = pc.clone()
            pc_new.apply_transform(x, y, z, rx, ry, rz)
            both_pc.append(pc_new)

        self.clouds[name_out] = both_pc

    def merge(self, name_base, name_2, name_out):
        logger.debug('merge %s, %s as %s' % (name_base, name_2, name_out))
        both_pc = []
        for i in range(2):
            pc = self.clouds[name_2][i].clone()
            pc_new = self.clouds[name_base][i].add(pc)
            both_pc.append(pc_new)

        self.clouds[name_out] = both_pc

    # below not reviewed yet
    def auto_merge(self, name_base, name_2, name_out):
        logger.debug('automerge %s, %s' % (name_base, name_2))
        pc_both = []
        for i in range(2):
            if len(self.clouds[name_base][i]) == 0 or len(self.clouds[name_2][i]) == 0:
                # if either pointcloud with zero point, return name_2 without transform
                pc_both.append(self.clouds[name_2][i].clone())
                continue
            reg = _scanner.RegCloud(self.clouds[name_base][i], self.clouds[name_2][i])
            result, pc = reg.SCP()
            # TODO:result?
            pc_both.append(pc)

        self.clouds[name_out] = pc_both
        return True


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

    def merge(self, name_base, name_2, name_out):
        self.clouds[name_out] = self.clouds[name_base]
        return True

    def export(self, name, file_format):
        return b"FLUX 3d printer: flux3dp.com, 2015                                              \x0c\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00 A\x00\x00\x00\x00\x00\x00 A\x00\x00"
