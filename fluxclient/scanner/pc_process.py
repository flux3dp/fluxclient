#!/usr/bin/env python3
import struct
from operator import ge, le
import logging
from io import BytesIO, StringIO
from bisect import bisect
import copy
from os import environ
from math import sqrt, asin, pi, radians, cos, sin, isnan
import uuid
import threading
import sys

from scipy.interpolate import Rbf
import numpy as np

from fluxclient.scanner.tools import write_stl, write_pcd, write_asc, read_pcd, cross
from fluxclient.scanner import _scanner


logger = logging.getLogger(__name__)


class PcProcess():
    """process point cloud"""
    def __init__(self, scan_settings):
        self.clouds = {}  # clouds that hold all the point cloud data, key:name, value:point cloud
        self.meshs = {}
        self.settings = scan_settings

        self.export_data = {}
        self.lock = threading.Lock()

    def upload(self, name, buffer_pc_L, buffer_pc_R, L_len, R_len):
        """
        # upload [name] [point count L] [point count R]
        """
        self.clouds[name] = self.to_cpp((self.unpack_data(buffer_pc_L), self.unpack_data(buffer_pc_R)))
        logger.debug('upload %s, L: %d R: %d' % (name, len(self.clouds[name][0]), len(self.clouds[name][1])))
        logger.debug('all:' + " ".join(self.clouds.keys()))

    def import_file(self, name, buf, filetype):
        """
        import file from file not from machine
        [in] name: name of the point cloud
        [in] buf: file content
        [in] filetype: filetype, now only support pcd, should support ply in the future
        """
        if filetype == 'pcd':
            try:
                tmp = read_pcd(buf)
                tmp = self.to_cpp((tmp, []))
                self.clouds[name] = tmp
                return True, ''
            except:
                return False, "Import fail, file broken?"
        else:
            logger.warning("can't parse {} file".format(filetype))
            raise NotImplementedError

    def unpack_data(self, buffer_data):
        """
        unpack buffer data into [[x, y, z, r, g, b]]
        [in] buffer_data:
        """
        assert len(buffer_data) % 24 == 0, "wrong buffer size %d (can't devide by 24)" % (len(buffer_data) % 24)
        pc = []
        for p in range(int(len(buffer_data) / 24)):
            tmp_point = list(struct.unpack('<ffffff', buffer_data[p * 24:p * 24 + 24]))
            tmp_point[3] = round(0 if isnan(tmp_point[3] * 255) else tmp_point[5] * 255)
            tmp_point[4] = round(0 if isnan(tmp_point[4] * 255) else tmp_point[5] * 255)
            tmp_point[5] = round(0 if isnan(tmp_point[5] * 255) else tmp_point[5] * 255)
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

        tmp = []
        for pc in pc_both:
            tmp.append(pc.cut('xyzr'.index(mode), 1 if direction else 0, value))
        self.clouds[name_out] = tmp

    def delete_noise(self, name_in, name_out, stddev):
        """
        delete noise base on distance of each point
        pc_source could be a string indcating the point cloud that we want
        """
        logger.debug('delete_noise [%s] [%s] [%.4f]' % (name_in, name_out, stddev))
        pc = [i.clone() for i in self.clouds[name_in]]
        pc0_size = len(pc[0])

        pc = pc[0].add(pc[1])

        if len(pc) == 0:
            logger.debug('empty pc')
            pc_both = [pc, _scanner.PointCloudXYZRGBObj()]
            self.clouds[name_out] = pc_both
        else:
            logger.debug('start with %d point' % len(pc))
            pc.SOR(int(self.settings.NoiseNeighbors), stddev)
            logger.debug('finished with %d point' % len(pc))
            pc_both = [pc, _scanner.PointCloudXYZRGBObj()]
            self.clouds[name_out] = pc_both
            self.cluster(name_out, name_out, self.settings.SegmentationDistance)
            self.closure(name_out, name_out, self.settings.CloseTop, False, 10)
            self.closure(name_out, name_out, self.settings.CloseBottom, True, 10)

    def cluster(self, name_in, name_out, thres=2):
        """
        make a cluster from the input cloud, can help deleting the noise by removing cluster that are too small
        """
        pc = self.clouds[name_in]
        logger.debug('cluster {} points'.format(len(pc[0]) + len(pc[1])))
        pc0_size = len(pc[0])
        output = (pc[0].add(pc[1])).Euclidean_Cluster(thres)
        output = sorted(output, key=lambda x: len(x))

        tmp_pc = self.to_cpp([[], []])
        for j in output[-1:]:
            for i in j:
                if i < pc0_size:
                    p = pc[0][i]
                else:
                    p = pc[1][i - pc0_size]
                tmp_pc[0].push_backPoint(*p)
        logger.debug('finish with {} cluster, {} points in biggest one'.format(len(output), len(output[-1])))
        self.clouds[name_out] = tmp_pc

    def to_mesh(self, name_in):
        """
        convert point cloud to mesh
        """
        logger.debug('to_mesh name:%s' % name_in)

        pc_both = self.clouds[name_in]
        # WARNING: merge L and R here!
        # pc = pc_both[0].clone()
        pc = pc_both[0].add(pc_both[1])
        pc.ne_viewpoint(self.settings.NeighborhoodDistance)
        # pc.ne()
        pc.to_mesh([5.5], method='POS')  # compute mesh
        return pc

    def dump(self, name):
        """
        dump the indicated(name) cloud, dumping
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
        """
        export as a file
        [in] name: the name of desired pc
        [in] file_format: output as .pcd, .ply, .stl file
        [in] mode: if using stl mode, you can specified ascii or binary stl file
        """
        if file_format == 'pcd':
            pc_both = self.clouds[name]
            # WARNING: merge L and R here!
            pc_add = []
            pc_size = []
            for pc in pc_both:
                pc_size.append(len(pc))
                for p_i in range(pc_size[-1]):
                    pc_add.append(pc.get_item(p_i))
            tmp = StringIO()
            write_pcd(pc_add, tmp)
            return tmp.getvalue().encode()

        elif file_format == 'asc':
            pc_both = self.clouds[name]
            # WARNING: merge L and R here!
            pc_add = []
            pc_size = []
            for pc in pc_both:
                pc_size.append(len(pc))
                for p_i in range(pc_size[-1]):
                    pc_add.append(pc.get_item(p_i))
            tmp = StringIO()
            write_asc(pc_add, tmp)
            return tmp.getvalue().encode()

        elif file_format == 'ply':
            pc_both = self.clouds[name]
            # WARNING: merge L and R here!
            pc_add = []
            pc_size = []
            for pc in pc_both:
                pc_size.append(len(pc))
                for p_i in range(pc_size[-1]):
                    pc_add.append(pc.get_item(p_i))
            raise NotImplementedError

        elif file_format == 'stl':
            pc_mesh = self.to_mesh(name)
            mesh_l = pc_mesh.STL_to_List()
            if mode == 'ascii':
                strbuf = StringIO()
                ##################### fake code ###########################
                if environ.get("flux_debug") == '1':
                    write_stl(mesh_l, './output.stl', mode)
                ###########################################################
                write_stl(mesh_l, strbuf, mode)
                return strbuf.getvalue().encode()

            elif mode == 'binary':
                buf = BytesIO()
                ##################### fake code ###########################
                if environ.get("flux_debug") == '1':
                    write_stl(mesh_l, './output.stl', mode)
                ###########################################################
                write_stl(mesh_l, buf, mode)
                return buf.getvalue()

    def export_threading(self, name, file_format, mode='binary'):
        """
        export as a file
        [in] name: the name of desired pc
        [in] file_format: output as .pcd, .ply, .stl file
        [in] mode: if using stl mode, you can specified ascii or binary stl file
        """
        collect_name = str(uuid.uuid4())
        T = threading.Thread(target=self.sub_export, args=(collect_name, name, file_format, mode))
        T.start()

        self.lock.acquire()
        self.export_data[collect_name] = T
        self.lock.release()
        return collect_name

    def sub_export(self, collect_name, name, file_format, mode='binary'):
        if file_format == 'pcd':
            pc_both = self.clouds[name]
            # WARNING: merge L and R here!
            pc_add = []
            pc_size = []
            for pc in pc_both:
                pc_size.append(len(pc))
                for p_i in range(pc_size[-1]):
                    pc_add.append(pc.get_item(p_i))
            tmp = StringIO()
            write_pcd(pc_add, tmp)
            ret_buf = tmp.getvalue().encode()

        elif file_format == 'ply':
            pc_both = self.clouds[name]
            # WARNING: merge L and R here!
            pc_add = []
            pc_size = []
            for pc in pc_both:
                pc_size.append(len(pc))
                for p_i in range(pc_size[-1]):
                    pc_add.append(pc.get_item(p_i))
            raise NotImplementedError

        elif file_format == 'stl':
            pc_mesh = self.to_mesh(name)
            mesh_l = pc_mesh.STL_to_List()
            if mode == 'ascii':
                strbuf = StringIO()
                ##################### fake code ###########################
                if environ.get("flux_debug") == '1':
                    write_stl(mesh_l, './output.stl', mode)
                ###########################################################
                write_stl(mesh_l, strbuf, mode)
                ret_buf = strbuf.getvalue().encode()

            elif mode == 'binary':
                buf = BytesIO()
                ##################### fake code ###########################
                if environ.get("flux_debug") == '1':
                    write_stl(mesh_l, './output.stl', mode)
                ###########################################################
                write_stl(mesh_l, buf, mode)
                ret_buf = buf.getvalue()
        self.lock.acquire()
        self.export_data[collect_name] = ret_buf
        self.lock.release()
        return

    def export_collect(self, collect_name):

        flag = self.lock.acquire(blocking=False)
        if flag:
            if not collect_name in self.export_data:
                return 'key error'
            if isinstance(self.export_data[collect_name], bytes):
                buf = self.export_data[collect_name]
            else:
                buf = False
            self.lock.release()
            return buf
        else:
            return False

    def apply_transform(self, name_in, x, y, z, rx, ry, rz, name_out):
        """
        apply_transform to [name_in] pointcloud
        and put it into [name_out]
        note that we transforming left and right pointcloud together, and split it into 2 subset
        it's because the left and right pc might have different center position
        """
        # add L and R
        pc_both = self.clouds[name_in]
        pc = pc_both[0].add(pc_both[1])  # return a new pc

        pc.apply_transform(x, y, z, rx, ry, rz)

        # split
        new_pc = self.to_cpp([[], []])
        for i in range(len(pc_both[0])):
            new_pc[0].push_backPoint(*pc[i])
        for i in range(len(pc_both[0]), len(pc_both[0]) + len(pc_both[1])):
            new_pc[1].push_backPoint(*pc[i])

        self.clouds[name_out] = new_pc

    def merge(self, name_base, name_2, name_out):
        """
        simply add two pc together
        [in] name_base, name_2
        [out] name_out
        """
        logger.debug('merge %s, %s as %s' % (name_base, name_2, name_out))
        both_pc = []
        for i in range(2):
            pc = self.clouds[name_2][i].clone()
            pc_new = self.clouds[name_base][i].add(pc)
            both_pc.append(pc_new)

        self.clouds[name_out] = both_pc

    def subset(self, name_base, name_out, mode):
        if not mode in ['left', 'right', 'both']:
            return "mode error:{}, should be 'left', 'right' or 'both'".format(mode)
        if not name_base in self.clouds:
            return "name error: {} not upload yet"
        else:
            logger.debug('generate subset from {} into {}'.format(name_base, name_out))
            out_pc = []
            if mode == 'left' or mode == 'both':
                out_pc.append(self.clouds[name_base][0].clone())
            else:
                out_pc.append(_scanner.PointCloudXYZRGBObj())

            if mode == 'right' or mode == 'both':
                out_pc.append(self.clouds[name_base][1].clone())
            else:
                out_pc.append(_scanner.PointCloudXYZRGBObj())

            self.clouds[name_out] = out_pc
            return 'ok'

    def cone_bottom(self, name_in, name_out, z_value, thick=5):
        pass

    def closure(self, name_in, name_out, z_value, floor, thick=5):
        if floor:
            logger.debug('adding floor at {}'.format(z_value))
        else:
            logger.debug('adding ceiling at {}'.format(z_value))

        points = []
        out_pc = [i.clone() for i in self.clouds[name_in]]

        self.cut(name_out, name_out, 'z', floor, z_value)  # what the fuck
        for i in range(2):
            for p in range(len(out_pc[i])):
                points.append(out_pc[i][p])

        # get near floor and a ring point set
        points = [[p, asin(p[1] / sqrt(p[0] ** 2 + p[1] ** 2)) if p[0] > 0 else pi - asin(p[1] / sqrt(p[0] ** 2 + p[1] ** 2))] for p in points]  # add theta data
        points = sorted(points, key=lambda x: abs(x[0][2] - z_value))

        # TODO: use a better way to find ring
        rec = [float('-inf'), float('inf')]
        interval = 2 * pi / self.settings.scan_step * 0.8
        after = [points[0]]  # find out the boarder points

        for p in points[1:]:
            tmp_index = bisect(rec, p[1])  # binary search where to insert
            if p[1] - rec[tmp_index - 1] > interval and rec[tmp_index] - p[1] > interval and abs(after[0][0][2] - p[0][2]) < thick:
                rec.insert(tmp_index, p[1])
                after.append(p)
            if len(rec) > self.settings.scan_step + 2:
                break

        after = sorted(after, key=lambda x: x[1])
        after = [p[0] for p in after]

        plane = copy.deepcopy(after)
        index = sorted(range(len(plane)), key=lambda x: [after[x][0], after[x][1]])

        # find the convex hull
        # ref: http://www.csie.ntnu.edu.tw/~u91029/ConvexHull.html
        # Andrew's Monotone Chain
        output = []
        for j in index:  # upper
            while len(output) >= 2 and cross(after[output[-2]], after[output[-1]], after[j]) <= 0:
                output.pop()
            output.append(j)

        t = len(output) + 1
        for j in index[-2::-1]:  # lower
            while len(output) > t and cross(after[output[-2]], after[output[-1]], after[j]) <= 0:
                output.pop()
            output.append(j)
        output.pop()

        boarder = [after[x] for x in sorted(output)]

        # compute the plane using RBF model
        tmp = []
        grid_leaf = 100
        X = np.linspace(min(p[0] for p in plane), max(p[0] for p in plane), grid_leaf)
        Y = np.linspace(min(p[1] for p in plane), max(p[1] for p in plane), grid_leaf)
        XI, YI = np.meshgrid(X, Y)

        x = np.array([p[0] for p in plane])
        y = np.array([p[1] for p in plane])
        z = np.array([p[2] for p in plane])
        logger.debug('plane len %i', len(plane))
        try:

            rbf = Rbf(x, y, z, function='thin_plate', smooth=0)

            # for p in plane:
            #     tmp.append([p[0], p[1], p[2], 255, 0, 0])

            if floor:
                color = [255, 0, 0]
            else:
                color = [0, 255, 0]

            color = [0., 0., 0.]
            for i in after:
                for j in range(3):
                    color[j] += i[j + 3]
            color = [i / len(after) for i in color]
            # color = [255, 0, 0]

            ZI = rbf(XI, YI)
            for xx in range(grid_leaf):
                for yy in range(grid_leaf):
                    p = [XI[xx][yy], YI[xx][yy], ZI[xx][yy]] + color[:]
                    flag = True
                    for b in range(len(boarder)):  # check whether inside the boundary
                        if (cross(boarder[b], boarder[(b + 1) % len(boarder)], p)) < 0:
                            flag = False
                            break
                    if flag:
                        tmp.append(p)
            del rbf
        except Exception as e:
            logger.exception("rbf error")

        plane += tmp
        for p in plane:
            out_pc[0].push_backPoint(*p)
        self.clouds[name_out] = out_pc

    def auto_alignment(self, name_base, name_2, name_out):
        """
        """
        # what about empty point cloud?
        logger.debug('auto_alignment %s, %s' % (name_base, name_2))

        # add L and R
        pc_base = self.clouds[name_base]
        pc_base = pc_base[0].add(pc_base[1])
        pc_2 = self.clouds[name_2]
        pc_2 = pc_2[0].add(pc_2[1])

        if len(pc_base) == 0 or len(pc_2) == 0:
            # if either pointcloud with zero point, return name_2 without transform
            pc_both = pc_2
        else:
            reg = _scanner.RegCloud(pc_base, pc_2, self.settings)
            result, pc_both = reg.SCP()
            # TODO:result???

        new_pc = self.to_cpp([[], []])
        for i in range(len(self.clouds[name_2][0])):
            new_pc[0].push_backPoint(*pc_both[i])
        for i in range(len(self.clouds[name_2][0]), len(self.clouds[name_2][0]) + len(self.clouds[name_2][1])):
            new_pc[1].push_backPoint(*pc_both[i])

        self.clouds[name_out] = new_pc
        return True
