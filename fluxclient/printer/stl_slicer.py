# !/usr/bin/env python3

import struct
import io
import subprocess
import tempfile
import os
import sys

from PIL import Image

try:
    import fluxclient.printer._printer as _printer
except:
    pass
from fluxclient.fcode.g_to_f import GcodeToFcode
from fluxclient.scanner.tools import dot, normal


class StlSlicer(object):
    """slicing objects"""
    def __init__(self):
        super(StlSlicer, self).__init__()
        self.reset()

    def reset(self):
        self.models = {}  # models data
        self.parameter = {}  # model's parameter
        self.user_setting = {}  # slcing setting
        self.slic3r = '../Slic3r/slic3r.pl'  # slic3r's location
        self.slic3r_setting = './fluxghost/assets/flux_slicing.ini'
        self.config = self.my_ini_parser(self.slic3r_setting)
        self.config['gcode_comments'] = '1'  # force open comment in gcode generated
        self.path = None
        self.image = b''

    def upload(self, name, buf):
        """
        upload a model's data in stl as bytes data
        """
        self.models[name] = buf

    def upload_image(self, buf):
        b = io.BytesIO()
        b.write(buf)
        img = Image.open(b)
        img = img.resize((640, 640))  # resize preview image

        b = io.BytesIO()
        img.save(b, 'png')
        image_bytes = b.getvalue()
        self.image = image_bytes
        ######################### fake code ###################################
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
        else:
            raise ValueError("%s not upload yet" % (name))

    def set_params(self, key, value):
        """
        basic printing parameter in front end
        """
        if key in ['printSpeed', 'material', 'raft', 'support', 'layerHeight', 'infill', 'travelingSpeed', 'extrudingSpeed', 'temperature']:
            self.user_setting[key] = value
            return True
        else:
            return False

    def advanced_setting(self, lines):
        """
        user input  setting content
        use '#' as comment symbol (different from wiki's ini file standard)
        return the line number of bad input
        """
        counter = 0
        bad_lines = []
        for line in lines:
            if '#' in line:  # clean up comement
                line = line[:line.index('#')]
            line = line.strip()
            if '=' in line:
                key, value = map(lambda x: x.strip(), line.split('=', 1))
                if key in self.config:
                    self.config[key] = value
                else:
                    bad_lines.append(counter)
            elif line != '':
                bad_lines.append(counter)
            counter += 1
        return bad_lines

    def get_path(self):
        if self.path is None:
            return ''
        else:
            result = []
            for layer in self.path:
                tmp = []
                for p in layer:
                    tmp.append('{"t":%d, "p":[%.3f, %.3f, %.3f]}' % (p[3], p[0], p[1], p[2]))
                result.append('[' + ','.join(tmp) + ']')
            return '[' + ','.join(result) + ']'

    def gcode_generate(self, names, ws, output_type):
        """
        input: names of stl that need to be sliced
        output:
            if success:
                gcode (binary in bytes), metadata([TIME_COST, FILAMENT_USED])
            else:
                False, [error message]
        """

        # check if names are all seted
        for n in names:
            if not (n in self.models and n in self.parameter):
                return False, '%s not set yet' % (n)

        ws.send_progress('merging', 0.2)
        m_mesh_merge = _printer.MeshObj([], [])
        for n in names:
            points, faces = self.read_stl(self.models[n])
            m_mesh = _printer.MeshObj(points, faces)
            m_mesh.apply_transform(self.parameter[n])
            m_mesh_merge.add_on(m_mesh)

        bounding_box = m_mesh_merge.bounding_box()
        cx, cy = (bounding_box[0][0] + bounding_box[1][0]) / 2., (bounding_box[0][1] + bounding_box[1][1]) / 2.

        tmp = tempfile.NamedTemporaryFile(suffix='.stl', delete=False)
        tmp_stl_file = tmp.name  # store merged stl

        m_mesh_merge.write_stl(tmp_stl_file)

        tmp = tempfile.NamedTemporaryFile(suffix='.gcode', delete=False)
        tmp_gcode_file = tmp.name  # store gcode

        tmp = tempfile.NamedTemporaryFile(suffix='.ini', delete=False)
        tmp_slic3r_setting_file = tmp.name  # store gcode

        command = [self.slic3r, tmp_stl_file]

        command += ['--output', tmp_gcode_file]
        command += ['--print-center', '%f,%f' % (cx, cy)]

        for key in self.user_setting:
            if self.user_setting[key] != "default":
                if key == 'printSpeed':
                    pass  # TODO
                elif key == 'material':
                    pass  # TODO
                elif key == 'raft':
                    if self.user_setting[key] == '0':
                        self.config['raft_layers'] = '0'
                    elif self.user_setting[key] == '1':
                        self.config['raft_layers'] = '4'  # TODO?
                elif key == 'support':
                    self.config['support_material'] = self.user_setting[key]
                elif key == 'layerHeight':
                    self.config['first_layer_height'] = self.user_setting[key]
                    self.config['layer_height'] = self.user_setting[key]
                elif key == 'infill':
                    fill_density = float(self.user_setting[key]) * 100
                    fill_density = max(min(fill_density, 99), 0)
                    self.config['fill_density'] = str(fill_density) + '%'
                elif key == 'travelingSpeed':
                    self.config['travel_speed'] = self.user_setting[key]
                elif key == 'extrudingSpeed':
                    self.config['perimeter_speed'] = self.user_setting[key]
                    self.config['infill_speed'] = self.user_setting[key]
                elif key == 'temperature':
                    self.config['temperature'] = self.user_setting[key]

        self.my_ini_writer(tmp_slic3r_setting_file, self.config)

        command += ['--load', tmp_slic3r_setting_file]
        print('command:', ' '.join(command), file=sys.stderr)

        fail_flag = False

        p = subprocess.Popen(command, stderr=subprocess.STDOUT, stdout=subprocess.PIPE, universal_newlines=True)
        progress = 0.2
        while p.poll() is None:
            line = p.stdout.readline()
            print(line, file=sys.stderr, end='')
            sys.stderr.flush()
            if line:
                if line.startswith('=> ') and not line.startswith('=> Exporting'):
                    progress += 0.12
                    ws.send_progress((line.rstrip())[3:], progress)
                slic3r_out = line
        if p.poll() != 0:
            fail_flag = True

        # analying gcode(even transform)
        ws.send_progress('analying metadata', 0.99)

        fcode_output = io.BytesIO()
        with open(tmp_gcode_file, 'r') as f:
            m_GcodeToFcode = GcodeToFcode()
            m_GcodeToFcode.image = self.image
            m_GcodeToFcode.process(f, fcode_output)

            self.path = m_GcodeToFcode.path
            metadata = m_GcodeToFcode.md
            metadata = [float(metadata['TIME_COST']), float(metadata['FILAMENT_USED'].split(',')[0])]

            del m_GcodeToFcode

        if output_type == 'g':
            with open(tmp_gcode_file, 'rb') as f:
                output = f.read()
        elif output_type == 'f':
            output = fcode_output.getvalue()
        else:
            raise('wrong output type, only support gcode and fcode')

        ##################### fake code ###########################
        with open('output.gcode', 'wb') as f:
            with open(tmp_gcode_file, 'rb') as f2:
                f.write(f2.read())

        with open(tmp_stl_file, 'rb') as f:
            with open('merged.stl', 'wb') as f2:
                f2.write(f.read())

        with open('output.fcode', 'wb') as f:
            f.write(fcode_output.getvalue())
        ###########################################################

        # clean up tmp files
        fcode_output.close()
        os.remove(tmp_stl_file)
        os.remove(tmp_gcode_file)
        os.remove(tmp_slic3r_setting_file)
        if fail_flag:
            return False, slic3r_out
        else:
            return output, metadata

    @classmethod
    def my_ini_parser(cls, file_path):
        """
        read-in .ini setting file as default settings
        return a dict
        """
        result = {}
        with open(file_path, 'r') as f:
            for i in f.readlines():
                if i[0] == '#':
                    pass
                elif '=' in i:
                    tmp = i.rstrip().split('=')
                    result[tmp[0].rstrip()] = tmp[1].rstrip()
                else:
                    print(i, file=sys.stderr)
                    raise ValueError('not ini file?')
        return result

    @classmethod
    def my_ini_writer(cls, file_path, content):
        """
        write to [file_path] with dict(content)
        """
        with open(file_path, 'w') as f:
            for i in content:
                print("%s=%s" % (i, content[i]), file=f)
        return

    @classmethod
    def ascii_or_binary(cls, data, byte_order):
        """
        check what kind of stl file it is
        return False -> binary
        return True -> ascii
        """
        if not data.startswith(b'solid '):
            return False

        length = struct.unpack(byte_order + 'I', data[80:84])[0]
        if len(data) == 80 + 4 + length * 50:
            return False
        return True

    @classmethod
    def read_stl(cls, file_data):
        """
        read in stl
        """
        # https://en.wikipedia.org/wiki/STL_(file_format)
        if type(file_data) == str:
            with open(file_data, 'rb') as f:
                file_data = f.read()
                byte_order = '@'
        elif type(file_data) == bytes:
            byte_order = '<'
        else:
            raise ValueError('wrong stl data type:%s' % str(type(file_data)))
        points = {}  # key: points, value: index
        faces = []
        counter = 0
        if cls.ascii_or_binary(file_data, byte_order):
            # ascii stl file
            instl = io.StringIO(file_data.decode('utf8'))
            instl.readline()  # read in: "solid [name]"

            while True:
                t = instl.readline()  # read in: "facet normal 0 0 0"
                if t[:8] != 'endsolid':  # end of file
                    read_normal = tuple(map(float, (t.split()[-3:])))
                    instl.readline()   # outer loop
                    v0 = tuple(map(float, (instl.readline().split()[-3:])))
                    v1 = tuple(map(float, (instl.readline().split()[-3:])))
                    v2 = tuple(map(float, (instl.readline().split()[-3:])))
                    right_hand_mormal = normal([v0, v1, v2])
                    if dot(right_hand_mormal, read_normal) < 0:
                        v1, v2 = v2, v1

                    instl.readline()  # read in: "endloop"
                    instl.readline()  # read in: "endfacet"

                else:
                    break
                face = []
                for v in [v0, v1, v2]:
                    if v not in points:
                        points[v] = counter
                        points[counter] = v
                        counter += 1
                    face.append(points[v])
                faces.append(face)

        else:
            # binary stl file
            header = file_data[:80]
            length = struct.unpack(byte_order + 'I', file_data[80:84])[0]

            for i in range(length):
                index = i * 50 + 84
                read_normal = struct.unpack(byte_order + 'fff', file_data[index + (4 * 3 * 0):index + (4 * 3 * 1)])
                v0 = struct.unpack(byte_order + 'fff', file_data[index + (4 * 3 * 1):index + (4 * 3 * 2)])
                v1 = struct.unpack(byte_order + 'fff', file_data[index + (4 * 3 * 2):index + (4 * 3 * 3)])
                v2 = struct.unpack(byte_order + 'fff', file_data[index + (4 * 3 * 3):index + (4 * 3 * 4)])
                right_hand_mormal = normal([v0, v1, v2])
                if dot(right_hand_mormal, read_normal) < 0:
                    v1, v2 = v2, v1

                face = []
                for v in [v0, v1, v2]:
                    if v not in points:
                        points[v] = counter
                        points[counter] = v
                        counter += 1
                    face.append(points[v])
                faces.append(face)

        points_list = []
        for i in range(counter):
            points_list.append(points[i])
        return points_list, faces
