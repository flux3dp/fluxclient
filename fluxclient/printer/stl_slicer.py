# !/usr/bin/env python3

from platform import platform
from struct import unpack, Struct, pack
from io import BytesIO, StringIO
import subprocess
import tempfile
import logging
import shutil
import json
import copy
import sys
import gc
import os

from PIL import Image
import numpy as np

from fluxclient.scanner.tools import dot, normal, normalize, dotX, normalX
from fluxclient.utils._utils import GcodeToFcodeCpp, Tools
from fluxclient.fcode.g_to_f import GcodeToFcode
from fluxclient.hw_profile import HW_PROFILE
from fluxclient.printer.flux_raft import Raft
from fluxclient.printer import ini_string, ini_string_cura2, ini_constraint, ignore, ini_flux_params
from fluxclient.printer import _printer

logger = logging.getLogger(__name__)


def rreplace(s, old, new, occurrence):
    li = s.rsplit(old, occurrence)
    return new.join(li)


def read_until(f):
    """
    eat all the empty line
    """
    while True:
        l = f.readline().strip()
        if len(l) != 0:
            return l
        if f.tell() == len(f.getvalue()):
            return ''


class StlSlicer(object):
    """slicing objects"""
    def __init__(self, slic3r):
        super(StlSlicer, self).__init__()
        self.reset(slic3r)

    def reset(self, slic3r):
        self.working_p = []  # process that are slicing
        self.models = {}  # models data, store the buf(stl file)
        self.parameter = {}  # model's parameter

        # self.slic3r = '../Slic3r/slic3r.pl'  # slic3r's location
        # self.slic3r = '/Applications/Slic3r.app/Contents/MacOS/slic3r'
        self.slic3r = slic3r

        self.config = self.parse_config(ini_string.split('\n'))
        self.configCura2 = self.parse_config(ini_string_cura2.split('\n'))
        self.config['gcode_comments'] = '1'  # force open comment in gcode generated
        self.path = None
        self.output = None
        self.image = b''
        self.ext_metadata = {'CORRECTION': 'A'}
        self.path_js = None
        self.setting_slicer = "slic3r"
        self.version = 0
        self.T = None

    def from_other(self, other):
        self.working_p = other.working_p
        self.models = other.models
        self.parameter = other.parameter
        self.config = other.config
        self.path = None
        self.image = b''
        self.ext_metadata = {'CORRECTION': 'A'}
        return self

    def upload(self, name, buf, buf_type='stl'):
        """
        upload a model's data in stl as bytes data
        """
        try:
            if buf_type == 'stl':
                self.models[name] = self.read_stl(buf)
            elif buf_type == 'obj':
                self.models[name] = self.read_obj(buf)
            else:
                raise('unknown file type')
        except:
            logger.exception("oops")
            return False
        else:
            return True

    def is_aborted(self, p_index):
        return self.working_p[p_index][3]

    def duplicate(self, name_in, name_out):
        """
        name_in[in]: name for the original one
        name_out[in] name for the new one

        duplicate a model in models(but not set position yet)
        """
        logger.debug('duplicate in:{} out:{}'.format(name_in, name_out))
        if name_in in self.models:
            self.models[name_out] = [self.models[name_in][0], self.models[name_in][1]]
            return True
        else:
            return False

    def upload_image(self, buf):
        b = BytesIO()
        b.write(buf)
        img = Image.open(b)
        img = img.resize((640, 640))  # resize preview image

        b = BytesIO()
        img.save(b, 'png')
        image_bytes = b.getvalue()
        self.image = image_bytes

        uint_unpacker = lambda x: Struct("<I").unpack(x)[0]  # 4 bytes uint
        if self.output:
            script_size = uint_unpacker(self.output[8:12])
            index = 16 + script_size
            self.meta_size = uint_unpacker(self.output[index:index + 4])
            index += 4
            # assert crc32(self.output[index:index + self.meta_size]) == uint_unpacker(self.output[index + self.meta_size:index + self.meta_size + 4])
            index = index + self.meta_size + 4
            self.image_size = uint_unpacker(self.output[index:index + 4])
            index += 4
            self.output = b''.join([self.output[:-(self.image_size + 4)], pack('<I', len(self.image)), self.image])

        ######################### fake code ###################################
        if os.environ.get("flux_debug") == '1':
            with open('preview.png', 'wb') as f:
                f.write(image_bytes)
        ############################################################

    def delete(self, name):
        """
        delete [name]
        """
        if name in self.models:
            del self.models[name]
            if name in self.parameter:
                del self.parameter[name]
            return True, 'OK'
        else:
            return False, "%s not upload yet" % (name)

    def set(self, name, parameter):
        """
        set the position, scale, rotation... parameters
        (just record it, didn't actually compute it)
        """
        if name in self.models:
            self.parameter[name] = parameter
            return 'ok'
        else:
            return "%s not upload yet" % (name)

    def advanced_setting(self, lines):
        """
        user input  setting content
        use '#' as comment symbol (different from wiki's ini file standard)

        return error message of bad input

        """
        # TODO: close 'ignore' flag when changing some key back
        counter = 1
        lines = lines.split('\n')

        if self.version == 2:
            setting_slicer = "cura2"
            self.setting_slicer = "cura2"
        else:
            setting_slicer = "slic3r"

        bad_lines = []
        for line in lines:
            if '#' in line:  # clean up comement
                line = line[:line.index('#')].strip()
            if '=' in line:
                key, value = map(lambda x: x.strip(), line.split('=', 1))

                if setting_slicer == "cura2":
                    result = self.ini_value_check_cura2(key, value)
                else:
                    result = self.ini_value_check(key, value)

                if result == 'ok':
                    if setting_slicer == "cura2":
                        logger.info("Setting cura2 %s = %s" % (key, value))
                        self.configCura2[key] = value
                    else:
                        self.config[key] = value
                        if key == 'spiral_vase' and value == '1':
                            self.config['support_material'] = '0'
                            ini_constraint['support_material'] = [ignore]
                            self.config['fill_density'] = '0%'
                            ini_constraint['fill_density'] = [ignore]
                            self.config['perimeters'] = '1'
                            ini_constraint['perimeters'] = [ignore]
                            self.config['top_solid_layers'] = '0'
                            ini_constraint['top_solid_layers'] = [ignore]

                elif result == 'ignore':
                    # ignore this config key anyway
                    pass
                else:
                    bad_lines.append((counter, result))
            elif line != '' and line != 'default':
                bad_lines.append((counter, 'syntax error: %s' % line))
            counter += 1
        return bad_lines

    def sub_convert_path(self):
        logger.info("Converting path to json")
        m_GcodeToFcode = GcodeToFcodeCpp()
        self.path_js = Tools().path_to_js(self.path).decode()
        logger.info("Converted path to json")
        self.T = None

    def get_path(self):
        """
        """
        if self.T:
            self.T.join()
        logger.debug("Returning get path")
        return self.path_js

    def begin_slicing(self, names, ws, output_type):
        """
        :param list names: names of stl that need to be sliced
        :return:
            if success:
                gcode (binary in bytes), metadata([TIME_COST, FILAMENT_USED])
            else:
                False, [error message]
        """
        # check if names are all seted
        for n in names:
            if not (n in self.models and n in self.parameter):
                return False, 'id:%s is not setted yet' % (n)
        # tmp files
        if platform().startswith("Windows"):
            if not os.path.isdir('C:\Temp'):
                os.mkdir('C:\Temp')
            temp_dir = 'C:\Temp'
        else:
            temp_dir = None

        tmp = tempfile.NamedTemporaryFile(dir=temp_dir, suffix='.stl', delete=False)
        tmp_stl_file = tmp.name  # store gcode

        tmp = tempfile.NamedTemporaryFile(dir=temp_dir, suffix='.gcode', delete=False)
        tmp_gcode_file = tmp.name  # store gcode

        tmp = tempfile.NamedTemporaryFile(dir=temp_dir, suffix='.ini', delete=False)
        tmp_slic3r_setting_file = tmp.name  # store gcode

        m_mesh_merge = None

        for n in names:
            points, faces = self.models[n]
            m_mesh = _printer.MeshObj(points, faces)
            m_mesh.apply_transform(self.parameter[n])
            if m_mesh_merge is None:
                m_mesh_merge = m_mesh
            else:
                m_mesh_merge.add_on(m_mesh)

        if float(self.config['cut_bottom']) > 0:
            m_mesh_merge = m_mesh_merge.cut(float(self.config['cut_bottom']))

        bounding_box = m_mesh_merge.bounding_box()
        cx, cy = (bounding_box[0][0] + bounding_box[1][0]) / 2., (bounding_box[0][1] + bounding_box[1][1]) / 2.
        m_mesh_merge.write_stl(tmp_stl_file)

        self.generate_slicer_config(tmp_slic3r_setting_file, self.config, delete=ini_flux_params)

        command = [self.slic3r, tmp_stl_file]
        command += ['--output', tmp_gcode_file]
        command += ['--print-center', '%f,%f' % (cx, cy)]
        command += ['--load', tmp_slic3r_setting_file]

        logger.info('command: ' + ' '.join(command))
        self.end_slicing('slic3r begin next slicing')

        status_list = []

        from threading import Thread  # Do not expose thrading in module level
        p = Thread(target=self.slicing_worker, args=(command[:], dict(self.config), self.image, dict(self.ext_metadata), output_type, status_list, len(self.working_p)))
        self.working_p.append([p, [tmp_stl_file, tmp_gcode_file, tmp_slic3r_setting_file], status_list, False, len(self.working_p)])
        p.start()
        return True, ''

    def slicing_worker(self, command, config, image, ext_metadata, output_type, status_list, p_index):
        tmp_gcode_file = command[3]
        fail_flag = False

        try:
            subp = subprocess.Popen(command, stderr=subprocess.STDOUT, stdout=subprocess.PIPE, universal_newlines=True)
        except Exception:
            logger.exception("Slic3r start failed")
            status_list.append([False, [None, None], None])
            return

        path = None
        self.working_p[p_index].append(subp)
        logger.info("#%d Real slicing started", p_index)

        progress = 0.2
        slic3r_error = False
        slic3r_out = [None, None]

        while subp.poll() is None:
            line = subp.stdout.readline().strip()
            if line:
                logger.info(line)
                if line.startswith('=> ') and not line.startswith('=> Exporting'):
                    progress += 0.11
                    status_list.append('{"slice_status": "computing", "message": "%s", "percentage": %.2f}' % ((line.rstrip())[3:], progress))
                elif "Unable to close this loop" in line:
                    slic3r_error = True
                slic3r_out = [5, line]  # errorcode 5

        subp_returned = subp.poll()
        if self.is_aborted(p_index):
            return logger.info('Worker #%d aborted' % p_index)
        if subp_returned != 0:
            logger.info('#%d Slic3r returned abnormal %d ' % (p_index, subp_returned))
            fail_flag = True

        if not fail_flag:
            # analying gcode(even transform)
            status_list.append('{"slice_status": "computing", "message": "Analyzing Metadata", "percentage": 0.99}')

            fcode_output = BytesIO()

            if config['flux_calibration'] == '0':
                ext_metadata['CORRECTION'] = 'N'

            if config['detect_filament_runout'] == '1':
                ext_metadata['FILAMENT_DETECT'] = 'Y'
            else:
                ext_metadata['FILAMENT_DETECT'] = 'N'

            tmp = 8191
            if config['detect_head_tilt'] == '0':
                tmp -= 32
            if config['detect_head_shake'] == '0':
                tmp -= 16
            ext_metadata['HEAD_ERROR_LEVEL'] = str(tmp)

            with open(tmp_gcode_file, 'r') as f:
                status_list.append('{"slice_status": "computing", "message": "Analyzing Metadata++", "percentage": 0.99}')
                m_GcodeToFcode = GcodeToFcodeCpp(ext_metadata=ext_metadata)
                m_GcodeToFcode.config = config
                m_GcodeToFcode.image = image
                m_GcodeToFcode.process(f, fcode_output)
                path = m_GcodeToFcode.trim_ends(m_GcodeToFcode.path)
                metadata = m_GcodeToFcode.md
                metadata = [float(metadata['TIME_COST']), float(metadata['FILAMENT_USED'].split(',')[0])]
                if slic3r_error or len(m_GcodeToFcode.empty_layer) > 0:
                    status_list.append('{"slice_status": "warning", "message" : "%s"}' % ("{} empty layers, might be error when slicing {}".format(len(m_GcodeToFcode.empty_layer), repr(m_GcodeToFcode.empty_layer))))

                if float(m_GcodeToFcode.md['MAX_R']) >= HW_PROFILE['model-1']['radius']:
                    fail_flag = True
                    slic3r_out = [6, "Gcode area was too big"]  # errorcode 6

                del m_GcodeToFcode

            if output_type == '-g':
                with open(tmp_gcode_file, 'rb') as f:
                    output = f.read()
            elif output_type == '-f':
                output = fcode_output.getvalue()
            else:
                raise('wrong output type, only support gcode and fcode')

            ##################### fake code ###########################
            if os.environ.get("flux_debug") == '1':
                with open('output.gcode', 'wb') as f:
                    with open(tmp_gcode_file, 'rb') as f2:
                        f.write(f2.read())
                tmp_stl_file = command[1]
                with open(tmp_stl_file, 'rb') as f:
                    with open('merged.stl', 'wb') as f2:
                        f2.write(f.read())

                with open('output.fc', 'wb') as f:
                    f.write(fcode_output.getvalue())

                StlSlicer.generate_slicer_config("output.ini", config)
            ###########################################################

            # # clean up tmp files
            fcode_output.close()

        if fail_flag:
            try:
                path
            except:
                path = None
            status_list.append([False, slic3r_out, path])
        else:
            status_list.append([output, metadata, path])

    def end_slicing(self, exit_reason=""):
        """
        when being called, end every working slic3r process
        but couldn't kill the thread
        """
        logger.debug("Abort slicing:: %s", exit_reason)
        for p in self.working_p:
            if type(p[-1]) == (subprocess.Popen):
                if p[-1].poll() is None:
                    logger.debug("Aborting 'process' managed by worker:: #%d ", p[4])
                    p[-1].terminate()
                    p[3] = True
            else:
                # Turn on abort tag
                if not p[3]:
                    logger.debug("Aborting 'thread' managed by worker:: #%d ", p[4])
                    p[3] = True
                pass
            for filename in p[1]:
                try:
                    if ".stl" not in filename:
                        os.remove(filename)
                except:
                    pass
        # self.working_p = []

    def report_slicing(self):
        """
        report the slicing state
        find the last working process(self.working_p)
        and return the message in it
        """
        ret = []
        if self.working_p:
            for _ in range(len(self.working_p[-1][2])):
                message = self.working_p[-1][2][0]
                self.working_p[-1][2].pop(0)
                if type(message) == str:
                    ret.append(message)
                else:

                    if message[0]:
                        self.output = message[0]
                        self.metadata = message[1]
                        msg = '{"slice_status": "complete", "length": %d, "time": %.3f, "filament_length": %.2f}' % (len(self.output), self.metadata[0], self.metadata[1])
                    else:
                        self.output = None
                        self.metadata = None
                        if message[1][0] is None:
                            msg = '{"slice_status": "error", "error": "%d", "info": "%s"}' % (16, 'UNDEFINED_SLICING_ERROR')
                        else:
                            msg = '{"slice_status": "error", "error": "%d", "info": "%s"}' % (message[1][0], message[1][1])

                    self.path = message[2]

                    if self.path:
                        self.path_js = None
                        from threading import Thread  # Do not expose thrading in module level
                        self.T = Thread(target=self.sub_convert_path)
                        self.T.start()
                        ret.append(msg)
                    else:
                        ret.append('{"slice_status": "error", "error": "%d", "info": "%s"}' % (16, 'UNDEFINED_SLICING_ERROR'))
                        logger.info("Worker didn't returned path.. can not continue")
        return ret

    @classmethod
    def parse_config(cls, data):
        """
        data[in]: [str] indicating a file path or [list of str] indicating lines of ini file
        read-in .ini file setting file as default settings
        return a dict
        """
        result = {}
        if type(data) == str:
            # file path
            f = open(data, 'r')
            lines = f.readlines()
        else:
            lines = data

        for i in lines:
            if i[0] == '#':
                pass
            elif '=' in i:
                tmp = i.rstrip().split('=')
                result[tmp[0].strip()] = tmp[1].strip()
            else:
                logger.error(i)
                raise ValueError('not ini file?')
        return result

    def ini_value_check(self, key, value):
        """
        key[in]: str
        value[out]: str
        return: 'ok' or [error message]
        check whether (key, value) pair is valid according to the constraint
        """
        if key in self.config:
            if value.strip() == 'default':
                return 'ok'
            if ini_constraint[key]:
                return ini_constraint[key][0](key, value, *ini_constraint[key][1:])
            else:
                return 'ok'
        else:
            return 'Key not exists: %s' % key

    def ini_value_check_cura2(self, key, value):
        """
        key[in]: str
        value[out]: str
        return: 'ok' or [error message]
        check whether (key, value) pair is valid according to the constraint
        """
        if key in self.configCura2:
            return 'ok'
        return 'Key not exists: %s' % key

    @classmethod
    def generate_slicer_config(cls, file_path, content, delete=None):
        """
        file_path[in]: str, output file_path
        content[in]: dict
        write a .ini file
        specify delete not to write some key in content
        """

        if int(content.get('raft', '1')) == 0:
            logger.info("Raft off, remove raft_layers")
            content['temp_raft_layers'] = content['raft_layers']
            content['raft_layers'] = '0'
        elif not content.get('raftMargin'):
            content['raft_layers'] = content.get('temp_raft_layers', 4)

        if content.get('start_gcode'):
            content['start_gcode'] = "M109 S[first_layer_temperature]\\n" + content.get('start_gcode', '')

        with open(file_path, 'w') as f:
            for i in content:
                if delete and any(j in i for j in delete):
                    pass
                else:
                    print("%s=%s" % (i, content[i]), file=f)

    @classmethod
    def ascii_or_binary(cls, data, byte_order):
        """
        data[in]: bytes data of a stl file
        byte_order[in]: '@' or '<', for different input source
        check what kind of stl file it is
        return False -> binary
        return True -> ascii
        """
        if not data.startswith(b'solid '):
            return False

        length = unpack(byte_order + 'I', data[80:84])[0]
        if len(data) == 80 + 4 + length * 50:
            return False
        return True

    @classmethod
    def read_stl(cls, file_data):
        """
        file_data[in]: string indicating a a file path, or a bytes that is the content of stl file
        read in stl
        """
        # ref: https://en.wikipedia.org/wiki/STL_(file_format)
        if type(file_data) == str:
            with open(file_data, 'rb') as f:
                file_data = f.read()
                byte_order = '@'
        elif type(file_data) == bytes:
            byte_order = '<'
        else:
            raise ValueError('wrong stl data type: %s' % str(type(file_data)))

        points_list = []
        points_map = {}  # key: points, value: index
        counter = 0
        faces = np.zeros(0, dtype=np.int)
        if cls.ascii_or_binary(file_data, byte_order):
            tFaces = []
            # ascii stl file
            instl = StringIO(file_data.decode('utf8'))

            read_until(instl)  # read in: "solid [name]"
            while True:
                t = read_until(instl)  # read in: "facet normal 0 0 0"

                # if not t.startswith('endsolid'):  # end of file
                if t.startswith('endsolid'):
                    tt = read_until(instl)
                    if len(tt) == 0:
                        break
                    else:
                        continue

                read_normal = tuple(map(float, (t.split()[-3:])))
                read_until(instl)   # outer loop
                v0 = tuple(map(float, (read_until(instl).split()[-3:])))
                v1 = tuple(map(float, (read_until(instl).split()[-3:])))
                v2 = tuple(map(float, (read_until(instl).split()[-3:])))
                if dotX(v0, v1, v2, read_normal) < 0:
                    v1, v2 = v2, v1

                read_until(instl)  # read in: "endloop"
                read_until(instl)  # read in: "endfacet"

                for v in [v0, v1, v2]:
                    pIdx = points_map.get(v, None)
                    if pIdx is None:
                        points_map[v] = counter
                        pIdx = counter
                        points_list.append(v)
                        counter += 1
                    tFaces.append(pIdx)
            # Compact python list to nparray
            logger.info("np array convert (ascii) %d " % len(tFaces))
            faces = np.array(tFaces, dtype=np.int)
            del tFaces
            gc.collect()
        else:
            # binary stl file
            header = file_data[:80]
            length = unpack(byte_order + 'I', file_data[80:84])[0]

            patten = byte_order + 'fff'
            index = 84

            # declare static type np array
            faces = np.zeros(length * 3, dtype=np.int)
            fptr = 0

            for i in range(length):
                read_normal = unpack(patten, file_data[index + (4 * 3 * 0):index + (4 * 3 * 1)])
                v0 = unpack(patten, file_data[index + (4 * 3 * 1):index + (4 * 3 * 2)])
                v1 = unpack(patten, file_data[index + (4 * 3 * 2):index + (4 * 3 * 3)])
                v2 = unpack(patten, file_data[index + (4 * 3 * 3):index + (4 * 3 * 4)])
                if dotX(v0, v1, v2, read_normal) < 0:
                    v1, v2 = v2, v1

                for v in [v0, v1, v2]:
                    pIdx = points_map.get(v, None)
                    if pIdx is None:
                        points_map[v] = counter
                        pIdx = counter
                        points_list.append(v)
                        counter += 1
                    faces[fptr] = pIdx
                    fptr = fptr + 1
                index += 50
            logger.info("np array convert (bin) %d" % faces.size)

        logger.debug("Faces[0] type %s ", type(faces).__name__)
        return _printer.MeshCloud(points_list), faces

    @classmethod
    def read_obj(cls, file_data):
        if type(file_data) == str:
            with open(file_data, 'rb') as f:
                file_data = f.read()
        elif type(file_data) == bytes:
            pass
        else:
            raise ValueError('wrong stl data type: %s' % str(type(file_data)))

        in_obj = StringIO(file_data.decode('utf8'))
        points_list = []
        faces = []
        while True:
            t = read_until(in_obj)
            if len(t) == 0:
                break
            elif t.startswith('#'):  # comment
                continue
            t = t.split()
            if t[0] == 'v':
                points_list.append(tuple(float(t[j]) for j in range(1, 4)))
            elif t[0] == 'f':
                t = [i.split('/') for i in t]
                faces.append([int(t[j][0]) for j in range(1, 4)])
            else:
                pass
        for i in range(len(faces)):
            for j in range(3):
                if faces[i][j] > 0:
                    faces[i][j] -= 1
                else:
                    faces[i][j] = len(points_list) + faces[i][j]

        logger.debug("Faces[0] type %s ", type(faces).__name__)
        return _printer.MeshCloud(points_list), faces


class StlSlicerCura(StlSlicer):
    def __init__(self, slicer, version=1):
        super(StlSlicerCura, self).__init__(slicer)
        self._working_dir = tempfile.mkdtemp()
        self.transform_file = os.path.join(self._working_dir, "temp.transform")
        self.stl_cache_file = os.path.join(self._working_dir, "stl.cache")
        self.slicer = slicer
        self.version = version
        self.now_type = 3

    """
    :param list names: names of stl that need to be sliced

    :return:
        if success:
            gcode (binary in bytes), metadata([TIME_COST, FILAMENT_USED])
        else:
            False, [error message]
    """
    def begin_slicing(self, names, ws, output_type):
        # End other slicing process once called
        self.end_slicing('cura next slicing')

        logger.info('Begin slicing (CuraEngine)')
        # check if names are all seted
        for n in names:
            if not (n in self.models and n in self.parameter):
                return False, 'id:%s is not setted yet' % (n)

        status_list = []
        mergedConfig = self.configCura2 if self.version == 2 else self.config

        self.end_slicing()
        from threading import Thread  # Do not expose thrading in module level
        p = Thread(target=self._slicing_worker, args=(dict(mergedConfig), self.image, dict(self.ext_metadata), output_type, status_list, names, ws, len(self.working_p)))
        # thread, files, status_list
        self.working_p.append([p, [], status_list, False, len(self.working_p)])
        p.start()
        return True, ''

    def _slicing_worker(self, *args, **kw):
        try:
            self.slicing_worker(*args, **kw)
        except Exception:
            logger.exception("slicing error")

    def slicing_worker(self, config, image, ext_metadata, output_type, status_list, names, ws, p_index):
        tmp = tempfile.NamedTemporaryFile(dir=self._working_dir, suffix='.stl', delete=False)
        tmp_stl_file = tmp.name  # store gcode

        tmp = tempfile.NamedTemporaryFile(dir=self._working_dir, suffix='.gcode', delete=False)
        tmp_gcode_file = tmp.name  # store gcode

        tmp = tempfile.NamedTemporaryFile(dir=self._working_dir, suffix='.ini', delete=False)
        tmp_slicer_setting_file = tmp.name  # store gcode

        m_mesh_merge = None

        cura2 = self.version == 2

        # Read old transform to see if we need to regenerate the stl
        old_transform = ""

        if os.path.exists(self.transform_file):
            f = open(self.transform_file, 'r+')
            old_transform = f.read()
            f.close()

        params = {}

        for n in names:
            params[n] = self.parameter[n]
        current_transform = json.dumps({'p': params, 'sink': float(config['cut_bottom'])})

        status_list.append('{"slice_status": "computing", "message": "Comparing Transformation", "percentage": 0.025}')

        if self.is_aborted(p_index):
            return logger.info('Worker #%d aborted' % p_index)

        if old_transform != current_transform:  # Need to regenerate new stl
            logger.info('Generating transformed stl')
            # Applying transform to each mesh object, and merge to m_mesh_merge
            for n in names:
                m_mesh = _printer.MeshObj(self.models[n][0], self.models[n][1])

                if self.is_aborted(p_index):
                    return logger.info('Worker #%d aborted' % p_index)

                m_mesh.apply_transform(self.parameter[n])

                if self.is_aborted(p_index):
                    return logger.info('Worker #%d aborted' % p_index)

                if m_mesh_merge is None:
                    m_mesh_merge = m_mesh
                else:
                    m_mesh_merge.add_on(m_mesh)

            if self.is_aborted(p_index):
                return logger.info('Worker #%d aborted' % p_index)

            if float(config['cut_bottom']) > 0:
                status_list.append('{"slice_status": "computing", "message": "Performing cut_bottom", "percentage": 0.04}')
                m_mesh_merge = m_mesh_merge.cut(float(config['cut_bottom']))

            logger.info('Writing new stl')
            status_list.append('{"slice_status": "computing", "message": "Writing new stl", "percentage": 0.05}')
            m_mesh_merge.write_stl(tmp_stl_file)
            # Save new file name for same transform

            # Remove old file
            if os.path.exists(self.stl_cache_file):
                f = open(self.stl_cache_file, 'r+')
                old_stl = f.read()
                f.close()
                if os.path.exists(old_stl):
                    os.remove(old_stl)

            f = open(self.stl_cache_file, 'w+')
            f.write(tmp_stl_file)
            f.close()

            f = open(self.transform_file, 'w+')
            f.write(current_transform)
            f.close()
        else:
            # Read old stl
            f = open(self.stl_cache_file, 'r+')
            tmp_stl_file = f.read()
            logger.info('Using last stl %s' % tmp_stl_file)
            f.close()

        logger.info('Writing ini to %s' % tmp_slicer_setting_file)
        command = []

        if cura2:
            self.generate_cura2_config(tmp_slicer_setting_file, config, delete=ini_flux_params)
            # Call CuraEngine in command line
            binary_path = self.slicer

            src_dir = os.path.dirname(binary_path)
            target_dir = os.path.dirname(tmp_slicer_setting_file)
            for fn in ("fd1p.def.json", "fdmprinter.def.json", "fdmextruder.def.json"):
                config_src = os.path.join(src_dir, fn)
                shutil.copy(config_src, target_dir)

            command = [binary_path, 'slice', '-v', '-j',
                       tmp_slicer_setting_file, '-o', tmp_gcode_file, '-l', tmp_stl_file]
        else:
            self.cura_ini_writer(tmp_slicer_setting_file, config, delete=ini_flux_params)
            # Call CuraEngine in command line
            command = [self.slicer, '-o', tmp_gcode_file, '-c', tmp_slicer_setting_file]
            command.append(tmp_stl_file)
            command.append('-v')

        logger.info('command: ' + ' '.join(command))

        if self.is_aborted(p_index):
            fail_flag = True
            logger.info('Worker #%d aborted' % p_index)

        status_list.append('{"slice_status": "computing", "message": "Submitting model to slicing engine", "percentage": 0.10}')
        logger.info('Starting CuraEngine')

        # tmp_gcode_file = command[2]
        # tmp_slicer_setting_file = command[4]
        fail_flag = False
        try:
            my_env = os.environ.copy()
            my_env["CURA_ENGINE_SEARCH_PATH"] = os.path.dirname(self.slicer) + "/resources"
            my_env["LD_LIBRARY_PATH"] = os.path.dirname(self.slicer) + "/resources"
            subp = subprocess.Popen(command, stderr=subprocess.STDOUT, stdout=subprocess.PIPE,
                                    universal_newlines=True, bufsize=0, env=my_env)
            self.working_p[p_index].append(subp)
            logger.info("#%d Real slicing started" % (p_index))

            progress = 0.2
            slicer_error = False
            slicer_out = [None, None]
            while subp.poll() is None:
                chunck = subp.stdout.readline()
                for line in chunck.split('\n'):
                    line = line.rstrip()
                    if line:
                        logger.info(line)
                        if line.endswith('s'):
                            progress += 0.12
                            status_list.append('{"slice_status": "computing", "message": "%s", "percentage": %.2f}' % (line, progress))
                        elif "Unable to close this loop" in line:
                            slicer_error = True
                        slicer_out = [5, line]  # errorcode 5
                    # break
            cura_result = subp.poll()
            if self.is_aborted(p_index):
                fail_flag = True
                logger.info('Worker #%d aborted' % p_index)
            if cura_result != 0:
                logger.info("#%d CuraEngine: Exited abnormally %d" % (p_index, cura_result))
                fail_flag = True
        except Exception as ex:
            fail_flag = True
            logger.info("CuraEngine: initialization failed %s" % str(type(ex)))
            logger.exception("CuraEngine: initialization failed %s", ex)
            slicer_out = [5, 'CuraEngine: Failed']  # errorcode 5

        if not fail_flag:
            # analying gcode(even transform)
            status_list.append('{"slice_status": "computing", "message": "Analyzing Metadata++", "percentage": 0.95}')

            fcode_output = BytesIO()
            if config['flux_calibration'] == '0':
                ext_metadata['CORRECTION'] = 'N'

            if config['detect_filament_runout'] == '1':
                ext_metadata['FILAMENT_DETECT'] = 'Y'
            else:
                ext_metadata['FILAMENT_DETECT'] = 'N'

            tmp = 8191
            if config['detect_head_tilt'] == '0':
                tmp -= 32
            if config['detect_head_shake'] == '0':
                tmp -= 16
            ext_metadata['HEAD_ERROR_LEVEL'] = str(tmp)

            with open(tmp_gcode_file, 'r') as f:
                m_GcodeToFcode = GcodeToFcodeCpp(ext_metadata=ext_metadata)
                m_GcodeToFcode.engine = 'cura'
                # m_GcodeToFcode.process_path = self.process_path
                m_GcodeToFcode.config = config
                m_GcodeToFcode.image = image
                m_GcodeToFcode.process(f, fcode_output)
                path = m_GcodeToFcode.trim_ends(m_GcodeToFcode.path)
                metadata = m_GcodeToFcode.md
                metadata = [float(metadata['TIME_COST']), float(metadata['FILAMENT_USED'].split(',')[0])]
                if slicer_error or len(m_GcodeToFcode.empty_layer) > 0:
                    status_list.append('{"slice_status": "warning", "message" : "%s"}' % ("{} empty layers, might be error when slicing {}".format(len(m_GcodeToFcode.empty_layer), repr(m_GcodeToFcode.empty_layer))))

                if float(m_GcodeToFcode.md['MAX_R']) >= HW_PROFILE['model-1']['radius']:
                    logger.info("CuraEngine: gcode out of range")
                    fail_flag = True
                    slicer_out = [6, "Gcode area too big MAX_R=%s" % str(m_GcodeToFcode.md['MAX_R'])]  # errorcode 6

                del m_GcodeToFcode

            if output_type == '-g':
                with open(tmp_gcode_file, 'rb') as f:
                    output = f.read()
            elif output_type == '-f':
                output = fcode_output.getvalue()
            else:
                raise('wrong output type, only support gcode and fcode')

            ##################### debug code ###########################
            if os.environ.get("flux_debug") == '1':
                with open('output.gcode', 'wb') as f:
                    with open(tmp_gcode_file, 'rb') as f2:
                        f.write(f2.read())

                tmp_stl_file = command[5]
                with open(tmp_stl_file, 'rb') as f:
                    with open('merged.stl', 'wb') as f2:
                        f2.write(f.read())

                with open('output.fc', 'wb') as f:
                    f.write(fcode_output.getvalue())

                with open(tmp_slicer_setting_file, 'rb') as f:
                    with open('output.ini', 'wb') as f2:
                        f2.write(f.read())
            ###########################################################

            # # clean up tmp files
            fcode_output.close()
        if fail_flag:
            try:
                path
            except:
                path = None
            status_list.append([False, slicer_out, path])
            logger.info("CuraEngine: Appended path to status_list (failed)")
        else:
            logger.info("CuraEngine: Appended path to status_list")
            status_list.append([output, metadata, path])

    @classmethod
    def generate_cura2_config(cls, file_path, content, delete=None):
        """
        file_path[in]: str, output file_path
        content[in]: dict
        write a .json file
        specify delete not to write some key in content
        ref: https://github.com/Ultimaker/Cura/blob/master/resources/definitions/fdmprinter.def.json
        """

        add_multi_line = lambda x: '"""\n' + x.replace('\\n', '\n') + '\n"""\n'

        definition = {
            "id": "fdp1",
            "version": 2,
            "name": "FLUX Delta+",
            "inherits": "fdmprinter",
            "metadata": {
                "visible": True,
                "author": "Jim Yu",
                "manufacturer": "FLUX",
                "category": "Other",
                "file_formats": "text/x-gcode",
                "icon": "icon_ultimaker2",
                "platform": "kossel_pro_build_platform.stl",
                "platform_offset": [0, 0, 0]
            },
            "overrides": {
                "machine_heated_bed": {
                    "default_value": True
                },
                "machine_width": {
                    "default_value": 170
                },
                "machine_height": {
                    "default_value": 170
                },
                "machine_depth": {
                    "default_value": 210
                },
                "machine_center_is_zero": {
                    "default_value": True
                },
                "machine_nozzle_size": {
                    "default_value": 0.40
                },
                "material_diameter": {
                    "default_value": 1.75
                },
                "machine_nozzle_heat_up_speed": {
                    "default_value": 2
                },
                "machine_nozzle_cool_down_speed": {
                    "default_value": 2
                },
                "machine_gcode_flavor": {
                    "default_value": "RepRap (Marlin/Sprinter)"
                },
                "machine_start_gcode": {
                    "default_value": add_multi_line('M109 S{}\n'.format(content['material_print_temperature_layer_0']) + content['machine_start_gcode'])
                },
                "machine_end_gcode": {
                    "default_value": add_multi_line(content['machine_end_gcode'])
                },
                "machine_name": {"default_value": "DeltaBot style"},
                "machine_shape": {
                    "default_value": "elliptic"
                },
                "adhesion_type": {'default_value': 'none'}
                # "raft_speed": int(content['first_layer_speed']),
                # "raft_surface_speed": { 'default_value': int(content['first_layer_speed']) },
                # "layer_height": { 'default_value': float(content['layer_height']) },
                # "layer_height_0": { 'default_value': float(content['first_layer_height']) },
                # "wall_line_count": { 'default_value': int(content['perimeters']) },
                # "support_enable": { 'default_value': content['support_material'] != '0' },
                # "support_angle": { 'default_value': (90 - int(content['support_material_threshold'])) } , # 0 -> No support 90 -> Many support ( For Cura 0 = All supported, 90 = No support })
                # "support_top_distance": { 'default_value': float(content['support_material_contact_distance']) },
                # "support_xy_distance": { 'default_value': float(content['support_material_spacing']) },
                # "support_pattern": { 'default_value': {'GRID': 'grid', 'LINES': 'lines', 'ZIGZAG': 'zigzag'}.get(content['support_material_pattern'], 'zigzag') },
                # "support_type": { 'default_value': {1: 'everywhere', 0: 'buildplate'}.get(int(content['support_everywhere']), 1) },
                # "support_infill_rate": { 'default_value': 20},
                # "support_line_distance": { 'default_value': 1.75},
                # "infill_pattern": { 'default_value' : {'AUTOMATIC': 'zigzag', 'ZIGZAG': 'zigzag', 'GRID': 'grid', 'LINES': 'lines', 'CONCENTRIC': 'concentric'}.get(content['fill_pattern'], 'zigzag') },
                # "skirt_line_count": { 'default_value': int(content['skirts']) },
                # "skirt_gap": { 'default_value': float(content['skirt_distance']) },
                # "brim_line_count": { 'default_value': int(content['brim_width']) },
                # "top_layers": { 'default_value': int(content['top_solid_layers']) },
                # "bottom_layers": { 'default_value': int(content['bottom_solid_layers']) },
                # "infill_line_distance": { 'default_value': 0.4 * 100 * 2 / float(content['fill_density'].rstrip('%')) },
                # "speed_travel": { 'default_value': float(content['travel_speed']) },
                # "speed_infill": { 'default_value': float(content['infill_speed']) },
                # "speed_support_infill": { 'default_value': float(content['support_material_speed']) },
                # "speed_support_interface": { 'default_value': float(content['support_material_speed']) / 1.5 },
                # "speed_wall_x": { 'default_value': float(content['perimeter_speed']) },
                # "speed_wall_0": { 'default_value': float(content['external_perimeter_speed']) },
                # "infill_overlap_mm": { 'default_value': 0.4 * float(content['infill_overlap'].rstrip('%')) / 100 },
                # "speed_topbottom": { 'default_value': float(content['solid_infill_speed']) },
                # "speed_print_layer_0": { 'default_value': float(content['first_layer_speed']) },
                # "speed_travel_layer_0": { 'default_value': float(content['infill_speed'])  * float(content['travel_speed']) / float(content['first_layer_speed']) },
                # "cool_min_layer_time": { 'default_value': int(content['slowdown_below_layer_time']) },
                # "retraction_hop": { 'default_value': float(content['retract_lift']) },
                # "retraction_hop_enabled": { 'default_value': True },
                # "retraction_amount": { 'default_value': float(content['retract_length']) },
                # "retraction_prime_speed": { 'default_value': int(content['retract_speed']) },
                # "retraction_retract_speed": { 'default_value': int(content['retract_speed']) },
                # "adhesion_type": { 'default_value': 'none' },
                # "xy_offset": { 'default_value': float(content['xy_size_compensation']) },
                # "raft_margin": { 'default_value': 5 },
                # "raft_airgap": { 'default_value': 0.3 },
                # "raft_base_line_width": { 'default_value': 1.0 },
                # "raft_surface_line_width": { 'default_value': 0.4 },
                # "raft_surface_layers" : { 'default_value': int(content['raft_layers']) },
                # "raft_interface_thickness": { 'default_value': 0.27 }
            }
        }

        for key in content:
            if delete and any(j in key for j in delete):
                pass
            definition['overrides'][key] = {'default_value': content[key]}

        definition['overrides']['machine_start_gcode']['default_value'] = add_multi_line('M109 S{}\n'.format(content['material_print_temperature_layer_0']) + content['machine_start_gcode'])
        definition['overrides']['machine_end_gcode']['default_value'] = add_multi_line(content['machine_end_gcode'])

        # Override cut_bottom
        definition['overrides']["mesh_position_z"] = {'default_value' : str(-float(content['cut_bottom']))}

        # Override raft and skirt over brim
        if int(content.get('raft', '1')) == 1:
            definition['overrides']['adhesion_type']['default_value'] = 'raft'
        elif int(content['brim_line_count']) == 0:  # skirt
            definition['overrides']['adhesion_type']['default_value'] = 'skirt'
        else:
            definition['overrides']['adhesion_type']['default_value'] = 'brim'

        # Compatible with old keyword
        if definition['overrides']['support_type']['default_value'] == 'touching_build_plate':
            definition['overrides']['support_type'] = {'default_value': 'buildplate'}

        logger.info(json.dumps(definition))
        with open(file_path, 'w') as f:
            f.write(json.dumps(definition))

        return

    @classmethod
    def cura_ini_writer(cls, file_path, content, delete=None):
        """
        file_path[in]: str, output file_path
        content[in]: dict
        write a .ini file
        specify delete not to write some key in content
        ref: https://github.com/daid/Cura/blob/b878f7dc28698d4d605a5fe8401f9c5a57a55367/Cura/util/sliceEngine.py
        """
        thousand = lambda x: float(x) * 1000

        new_content = {}
        new_content['extrusionWidth'] = int(thousand(content['nozzle_diameter']))

        new_content['objectPosition.X'] = -10.0
        new_content['objectPosition.Y'] = -10.0
        new_content['autoCenter'] = 0
        new_content['gcodeFlavor'] = 0

        new_content['raftMargin'] = 5000
        new_content['raftLineSpacing'] = 3000
        new_content['raftBaseThickness'] = 300
        new_content['raftBaseLinewidth'] = 1000

        new_content['raftInterfaceThickness'] = 270
        new_content['raftInterfaceLinewidth'] = 400
        new_content['raftInterfaceLineSpacing'] = new_content['raftInterfaceLinewidth'] * 2
        new_content['raftAirGap'] = 0

        new_content['raftAirGapLayer0'] = thousand(content['support_material_contact_distance'])
        new_content['raftBaseSpeed'] = content['first_layer_speed']

        new_content['raftFanSpeed'] = 0
        new_content['raftSurfaceThickness'] = 300
        new_content['raftSurfaceLinewidth'] = 400
        new_content['raftSurfaceLineSpacing'] = new_content['raftSurfaceLinewidth']
        new_content['raftSurfaceLayers'] = 10
        new_content['raftSurfaceSpeed'] = content['first_layer_speed']

        new_content['layerThickness'] = thousand(content['layer_height'])
        new_content['initialLayerThickness'] = thousand(content['first_layer_height'])
        new_content['insetCount'] = content['perimeters']

        if content['support_material'] == '0':
            new_content['supportAngle'] = -1
        else:
            new_content['supportAngle'] = 90 - int(content['support_material_threshold'])
        new_content['supportZDistance'] = thousand(content['support_material_contact_distance'])
        new_content['supportLineDistance'] = thousand(content['support_material_spacing'])
        new_content['supportType'] = {'GRID': 0, 'LINES': 1}.get(content['support_material_pattern'], 0)
        new_content['supportEverywhere'] = int(content['support_everywhere'])

        if int(content.get('raft', '1')) == 0:
            logger.info("Raft off, remove raft_layers")
            new_content['raftSurfaceLayers'] = 0
        else:
            logger.info("Raft on %d" % int(content['raft_layers']))
            new_content['raftSurfaceLayers'] = int(content['raft_layers'])

        if int(new_content['raftSurfaceLayers']) == 0:
            new_content['raftBaseThickness'] = 0
            new_content['raftInterfaceThickness'] = 0

        new_content['infillPattern'] = {'AUTOMATIC': 0, 'GRID': 1, 'LINES': 2, 'CONCENTRIC': 3}.get(content['fill_pattern'], 0)

        if int(content['brim_width']) == 0:  # skirt
            new_content['skirtLineCount'] = content['skirts']
            new_content['skirtDistance'] = thousand(content['skirt_distance'])
        else:  # brim
            new_content['skirtLineCount'] = content['brim_width']
            new_content['skirtDistance'] = 0

        # other
        new_content['upSkinCount'] = content['top_solid_layers']
        new_content['downSkinCount'] = content['bottom_solid_layers']

        fill_density = float(content['fill_density'].rstrip('%'))
        if fill_density == 0:
            new_content['sparseInfillLineDistance'] = -1
        elif fill_density == 100:
            new_content['sparseInfillLineDistance'] = new_content['extrusionWidth']
            new_content['downSkinCount'] = 10000
            new_content['upSkinCount'] = 10000
        else:
            new_content['sparseInfillLineDistance'] = int(100 * float(content['nozzle_diameter']) * 1000 / fill_density)

        # speed
        new_content['moveSpeed'] = content['travel_speed']

        # support speed
        new_content['printSpeed'] = content['support_material_speed']

        new_content['inset0Speed'] = content['external_perimeter_speed']  # WALL-OUTER
        new_content['insetXSpeed'] = content['perimeter_speed']  # WALL-INNER

        new_content['infillSpeed'] = content['infill_speed']
        new_content['infillOverlap'] = content['infill_overlap'].rstrip('%')

        new_content['fanSpeedMin'] = content['min_fan_speed'].rstrip('%')
        new_content['fanSpeedMax'] = content['max_fan_speed'].rstrip('%')

        # speed top bottom
        if fill_density == 100:
            new_content['skinSpeed'] = max(int(content['solid_infill_speed']), 4)
        else:
            new_content['skinSpeed'] = content['solid_infill_speed']

        new_content['initialLayerSpeed'] = content['first_layer_speed']

        new_content['nozzleSize'] = 400
        new_content['filamentDiameter'] = 1750

        new_content['retractionSpeed'] = content['retract_speed']
        new_content['retractionAmount'] = thousand(content['retract_length'])
        new_content['retractionZHop'] = thousand(content['retract_lift'])

        new_content['minimalExtrusionBeforeRetraction'] = 200

        new_content['filamentFlow'] = 97
        new_content['minimalLayerTime'] = int(content['slowdown_below_layer_time'])

        new_content['startCode'] = 'M109 S{}\n'.format(content['first_layer_temperature']) + content['start_gcode']
        new_content['endCode'] = content['end_gcode']

        new_content['layer0extrusionWidth'] = 600
        new_content['skirtMinLength'] = 150000
        new_content['fanFullOnLayerNr'] = 3

        add_multi_line = lambda x: '"""\n' + x.replace('\\n', '\n') + '\n"""\n'
        # special function for cura's setting file
        # replace '\\n' by '\n', add two lines of '""" indicating multiple lines of settings
        # in slicer's setting file:
        # startCode=aaa\nbbb\nccc
        #
        # in cura's setting file:
        #   startCode="""
        #   aaa
        #   bbb
        #   """

        new_content['startCode'] = add_multi_line(new_content['startCode'])
        new_content['endCode'] = add_multi_line(new_content['endCode'])

        cls.generate_slicer_config(file_path, new_content, delete)
        return
