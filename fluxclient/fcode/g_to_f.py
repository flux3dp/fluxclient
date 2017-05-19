# !/usr/bin/env python3

import logging
import struct
import sys
from zlib import crc32
from math import sqrt, sin, cos, pi, atan2
import time
from re import findall
from getpass import getuser
from threading import Thread

from fluxclient.fcode.fcode_base import FcodeBase, POINT_TYPE
from fluxclient.hw_profile import HW_PROFILE

logger = logging.getLogger(__name__)

class GcodeToFcode(FcodeBase):
    """transform from gcode to fcode

    this should done several thing:
      transform gcode into fcode
      analyze metadata
    """
    def __init__(self, version=1, head_type="EXTRUDER", ext_metadata={}):
        super(GcodeToFcode, self).__init__()

        self.tool = 0  # tool number set by T command
        self.absolute = True  # whether using absolute position
        self.unit = 1  # how many mm is one unit in gcode, might be mm or inch(2.54)
        self.crc = 0  # computing crc32, use for generating fcode

        self.current_speed = 1  # current speed (set by F), mm/minute
        self.image = None  # png image that will store in fcode as perview image, should be a bytes obj

        self.G92_delta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # X, Y, Z, E1, E2, E3 -> recording the G92 delta for each axis

        self.time_need = 0.  # recording time the printing process need, in sec
        self.distance = 0.  # recording distance the tool head will go through
        self.max_range = [0., 0., 0., 0.]  # recording max coordinate, [x,  y, z, r]
        self.filament = [0., 0., 0.]  # recording the filament each extruder needed, in mm
        self.previous = [0., 0., 0.]  # recording previous filament/path

        self.pause_at_layers = []

        self.md = {'HEAD_TYPE': head_type, 'TIME_COST': 0, 'FILAMENT_USED': '0,0,0'}  # basic metadata, use extruder as default

        self.md.update(ext_metadata)

        self.record_path = True  # to speed up, set this flag to False
        self.layer_now = 0  # record the current layer toolhead is

        self._config = None  # config dict(given from fluxstudio)

        self.backed_to_normal_temperature = False

        self.has_config = False
        self.highlight_layer = -1

    def get_metadata(self):
        """
        Gets the metadata
        """
        return self.md

    def get_img(self):
        """
        Gets the preview image
        """
        return self.image

    def header(self):
        """
        Returns header for fcode version 1
        """
        return b'FC' + b'x0001' + b'\n'

    def offset(self, x=0.0, y=0.0, z=0.0):
        self.G92_delta[0] += x
        self.G92_delta[1] += y
        self.G92_delta[2] += z

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self, value):
        self._config = value
        self.offset(z=float())
        for auto_pause_layer in self._config.get('pause_at_layers', '').split(','):
            if auto_pause_layer.isdigit():
                self.pause_at_layers.append(int(auto_pause_layer))
        self.has_config = True

    def write_metadata(self, stream):
        """
        Writes fcode's metadata
        including a dict, and a png image
        """
        md_join = '\x00'.join([i + '=' + self.md[i] for i in self.md]).encode()

        stream.write(struct.pack('<I', len(md_join)))
        stream.write(md_join)
        stream.write(struct.pack('<I', crc32(md_join)))

        if self.image is None:
            stream.write(struct.pack('<I', 0))
        else:
            stream.write(struct.pack('<I', len(self.image)))
            stream.write(self.image)

    def XYZEF(self, input_list):
        """
        Parses data into a list: [F, X, Y, Z, E1, E2, E3]
        and forms the proper command
        element will be None if not provided
        input_list : ['G1', 'F200', 'X1.0', 'Y-2.0']
        """
        command = 0
        number = [None] * 7
        for i in input_list[1:]:
            if i.startswith('F'):
                command |= (1 << 6)
                number[0] = float(i[1:])
            elif i.startswith('X'):
                command |= (1 << 5)
                number[1] = float(i[1:]) * self.unit
            elif i.startswith('Y'):
                command |= (1 << 4)
                number[2] = float(i[1:]) * self.unit
            elif i.startswith('Z'):
                command |= (1 << 3)
                number[3] = float(i[1:]) * self.unit
            elif i.startswith('E'):
                command |= (1 << (2 - self.tool))
                number[4 + self.tool] = float(i[1:]) * self.unit
            else:
                logger.debug('XYZEF fail:' + i)

        return command, number

    def G2_G3(self, input_list):
        sample_n = 100
        if input_list[0] == 'G2':
            clock = True
        else:
            clock = False
        command, d = self.XYZEF(input_list)

        if d[0]:
            self.current_speed = d[0]
        c_delta = [0.0, 0.0, 0.0]
        for i in input_list:
            if i.startswith('I'):
                c_delta[0] = float(i[1:]) * self.unit
            elif i.startswith('J'):
                c_delta[1] = float(i[1:]) * self.unit

        p_1 = self.current_pos[:3]

        E_index = None
        for i in range(4, len(d)):
            if d[i] is not None:
                E_index = i
        E_split = [None] * 3
        E_final = [None] * 3
        p_2 = p_1[:]
        if self.absolute:
            # p_2 = d[1:4]
            for k in range(1, 4):
                if d[k]:
                    p_2[k - 1] = d[k]
            if E_index:
                E_split[E_index - 4] = (d[E_index] - self.current_pos[E_index - 1]) / sample_n
                E_final = d[4:]
        else:
            for k in range(1, 4):
                if d[k]:
                    p_2[k - 1] = self.current_pos[k - 1] + d[k]
            # p_2 = [self.current_pos[i] + d[i + 1] for i in range(3)]
            if E_index:
                E_split[E_index - 4] = d[E_index] / sample_n
                E_final[E_index - 4] = d[E_index] + self.current_pos[E_index - 1]

        p_c = [p_1[i] + c_delta[i] for i in range(3)]

        sub_g1 = arc(p_1, p_2, p_c, clock, sample_n)
        for i in range(3, 7):
            command |= (1 << i)  # F, X, Y, Z

        if E_index:
            command |= (1 << (2 - self.tool))

        for i in range(len(sub_g1)):
            sub_g1[i].insert(0, self.current_speed)
            tmp = [None] * 3
            if E_index:
                tmp[E_index - 4] = self.current_pos[E_index - 1] + i * E_split[E_index - 4]
            sub_g1[i].extend(tmp)

        sub_g1.append([self.current_speed] + p_2 + E_final)

        return command, sub_g1

    def analyze_metadata(self, input_list, comment):
        """
        input_list: [F, X, Y, Z, E1, E2, E3]
        compute filament need for each extruder
        compute time needed
        """
        if input_list[0] is not None:
            self.current_speed = input_list[0]

        tmp_path = 0.
        moveflag = False  # record if position change in this command
        for i in range(1, 4):  # position
            if input_list[i] is not None:
                moveflag = True
                if self.absolute:
                    tmp_path += (input_list[i] - self.current_pos[i - 1]) ** 2
                    self.current_pos[i - 1] = input_list[i]
                else:
                    tmp_path += (input_list[i] ** 2)
                    self.current_pos[i - 1] += input_list[i]

                if abs(self.current_pos[i - 1]) > self.max_range[i - 1]:
                    self.max_range[i - 1] = abs(self.current_pos[i - 1])
        tmp_path = sqrt(tmp_path)

        pos_r = self.current_pos[0] ** 2 + self.current_pos[1] ** 2
        if pos_r > self.max_range[3]:  # compute MAX_R, sqrt() later
            self.max_range[3] = pos_r

        extrudeflag = False
        for i in range(4, 7):  # extruder
            if input_list[i] is not None and input_list[i] > 0:  # TODO: retract?
                extrudeflag = True
                if self.absolute:
                    # a special bug from slicer/cura
                    if self.has_config and self._config['flux_refill_empty'] == '1' and tmp_path != 0:
                        if input_list[i] - self.current_pos[i - 1] == 0:
                            input_list[i] = self.previous[i - 4] * tmp_path + self.current_pos[i - 1]
                            self.G92_delta[i - 1] += self.previous[i - 4] * tmp_path
                        else:
                            self.previous[i - 4] = (input_list[i] - self.current_pos[i - 1]) / tmp_path

                    self.filament[i - 4] += input_list[i] - self.current_pos[i - 1]
                    self.current_pos[i - 1] = input_list[i]
                else:
                    if self.has_config and self._config['flux_refill_empty'] == '1' and tmp_path != 0:
                        if input_list[i] == 0:
                            input_list[i] = self.previous[i - 4] * tmp_path
                        else:
                            self.previous[i - 4] = input_list[i] / tmp_path

                    self.filament[i - 4] += input_list[i]
                    self.current_pos[i - 1] += input_list[i]
                # TODO:clean up this part?, self.extrude_absolute flag
        
        self.distance += tmp_path
        self.time_need += (tmp_path / min(9000, self.current_speed * 0.92)) * 60  # from minute to sec
        # fill in self.path
        if self.record_path:
            self.process_path(comment, moveflag, extrudeflag)
        return input_list

    def writer(self, buf, stream):
        """
        Writes data into stream, update length and crc
        """
        self.script_length += len(buf)
        stream.write(buf)
        self.crc = crc32(buf, self.crc)

    def process(self, input_stream, output_stream):
        """
        Process a input_stream consist of gcode strings and write the fcode into output_stream
        """
        # Point type in constants
        PY_TYPE_NEW_LAYER, PY_TYPE_INFILL, PY_TYPE_PERIMETER, PY_TYPE_SUPPORT, PY_TYPE_MOVE, PY_TYPE_SKIRT, PY_TYPE_INNER_WALL, PY_TYPE_BRIM, PY_TYPE_RAFT, PY_TYPE_SKIN, PY_TYPE_HIGHLIGHT = [x for x in range(-1,10)]

        packer = lambda x: struct.pack('<B', x)  # easy alias for struct.pack('<B', x)
        packer_f = lambda x: struct.pack('<f', x)  # easy alias for struct.pack('<f', x)

        try:
            output_stream.write(self.header())
            output_stream.write(struct.pack('<I', 0))  # script length, will be modify in the end

            self.script_length = 0  # lengh of script block in fcode

            comment_list = []  # recorad a list of comments wrritten in gcode

            for line in input_stream:
                if line.startswith(':'):  # visible comment, starts with a ':'
                    line = ''
                    comment = line[1:]
                    comment_list.append(comment)
                if ';' in line:  # in line comment
                    line, comment = line.split(';', 1)
                    comment_list.append(comment)
                else:
                    comment = ''

                # split "G1 X2 Y1" into ["G1", "X2", "Y1"]
                line = findall('[A-Z][+-]?[0-9]+[.]?[0-9]*', line)

                if line:
                    # move command, put it at first since it's the most likely command
                    if line[0] == 'G1' or line[0] == 'G0':
                        subcommand, data = self.XYZEF(line)
                        # data: [F, X, Y, Z, E1, E2, E3]
                        if self.absolute:  # dealing with previous G92 command, add the offset back
                            for i in range(1, 7):
                                if data[i] is not None:
                                    data[i] += self.G92_delta[i - 1]

                        # auto pause at layers
                        if self.layer_now in self.pause_at_layers:
                            if self.highlight_layer != self.layer_now:
                                self.highlight_layer = self.layer_now
                                #unload filament
                                self.writer(packer(128 | (1 << 6) | (1 << 2) ), output_stream) #F 2000 E -1
                                self.writer(packer_f(2000), output_stream)
                                self.writer(packer_f(-1 + self.current_pos[3] + self.G92_delta[3]), output_stream)
                                self.writer(packer(128 | (1 << 3) ), output_stream) #Z +5
                                self.writer(packer_f(5 + self.current_pos[2] + self.G92_delta[2]), output_stream)
                                self.writer(packer(128 | (1 << 2) ), output_stream) #E +3
                                self.writer(packer_f(3 + self.current_pos[3] + self.G92_delta[3]), output_stream)
                                self.writer(packer(128 | (1 << 2) ), output_stream) #E -5
                                self.writer(packer_f(-5 + self.current_pos[3] + self.G92_delta[3]), output_stream)
                                #pause
                                self.writer(packer(5), output_stream)
                                #back to normal
                                self.G92_delta[3] += -5;
                        
                        #overwrite following layer temperature
                        if (self.layer_now == 2) and self.has_config and float(self._config['temperature']) > 0 and not self.backed_to_normal_temperature:
                            self.writer(packer(16), output_stream)
                            self.writer(packer_f(float(self._config['temperature'])), output_stream)
                            logger.error("Setting toolhead temperature back to normal #" + str(self.layer_now) + " to " + str(float(self._config['temperature'])));
                            self.backed_to_normal_temperature = True

                        # fix on slic3r bug slowing down in raft but not in real printing
                        if self.has_config and self.layer_now == int(self._config['raft_layers']) and self._config['flux_first_layer'] == '1':
                            data[0] = float(self._config['first_layer_speed']) * 60
                            subcommand |= (1 << 6)
                            logger.error("Oh no speed overwrite first_layer. #" + str(self.layer_now) + " to " + str(data[0]));

                        # this will change the data base on serveral settings
                        data = self.analyze_metadata(data, comment)
                        command = 128 | subcommand
                        self.writer(packer(command), output_stream)

                        for i in data:
                            if i is not None:
                                self.writer(packer_f(i), output_stream)

                    elif line[0] == 'G2' or line[0] == 'G3':
                        subcommand, sub_g1 = self.G2_G3(line)

                        command = 128 | subcommand
                        tmp_absolute = self.absolute  # record this flag

                        self.absolute = True
                        self.writer(packer(2), output_stream)  # set to absolute

                        for data in sub_g1:
                            data = self.analyze_metadata(data, comment)
                            self.writer(packer(command), output_stream)

                            for i in data:
                                if i is not None:
                                    self.writer(packer_f(i), output_stream)

                        if not tmp_absolute:
                            self.absolute = tmp_absolute
                            self.writer(packer(3), output_stream)

                    elif line[0] == 'X2':  # laser toolhead command
                        command = 32  # only one laser so far
                        self.writer(packer(command), output_stream)

                        if line[1].startswith('O'):
                            strength = float(line[1].lstrip('O')) / 255.
                        else:  # bad gcode!!
                            strength = 0
                        self.writer(packer_f(strength), output_stream)

                        if 'HEAD_TYPE' not in self.md:
                            self.md['HEAD_TYPE'] = 'LASER'

                    elif line[0] == 'G28':  # home
                        self.writer(packer(1), output_stream)
                        for i in range(2):
                            self.current_pos[i] = 0
                        self.current_pos[2] = HW_PROFILE['model-1']['height']

                    elif line[0] == 'G90':  # set to absolute
                        self.writer(packer(2), output_stream)
                        self.absolute = True
                    elif line[0] == 'G91':  # set to relative
                        self.absolute = False
                        self.writer(packer(3), output_stream)

                    elif line[0] == 'M82':  # set extruder to absolute
                        self.extrude_absolute = True
                    elif line[0] == 'M83':  # set extruder to relative
                        self.extrude_absolute = False

                    elif line[0] == 'G92':  # set position
                    # this command will not write into fcode
                    # but use self.G92_delta to record the position
                        sub_command, data = self.XYZEF(line)
                        if all(i is None for i in data):  # A G92 without coordinates will reset all axes to zero.
                            for i in range(1, 7):
                                data[i] = 0.0
                        else:
                            for i in range(1, len(data)):
                                if data[i] is not None:
                                    self.G92_delta[i - 1] = self.current_pos[i - 1] - data[i]

                    elif line[0] == 'G4':  # dwell
                        self.writer(packer(4), output_stream)
                        # P:ms or S:sec
                        for sub_line in line[1:]:
                            if sub_line.startswith('P'):
                                ms = float(line[1][1:])
                            elif sub_line.startswith('S'):
                                ms = float(line[1][1:]) * 1000
                        if ms < 0:
                            ms = 0
                        self.writer(packer_f(ms), output_stream)
                        self.time_need += ms / 1000

                    elif line[0] == 'M104' or line[0] == 'M109':  # set extruder temperature
                        command = 16
                        if line[0] == 'M109':
                            command |= (1 << 3)
                        for i in line:
                            if i.startswith('S'):
                                temp = float(i[1:])
                            elif i.startswith('T'):
                                self.tool = int(i[1:])
                                if self.tool > 7 or self.tool < 0:
                                    raise ValueError('too many extruder! %d' % self.tool)
                        command |= self.tool
                        self.writer(packer(command), output_stream)
                        self.writer(packer_f(temp), output_stream)

                    # set for inch or mm
                    elif line[0] == 'G20':  # inch
                        self.unit = 25.4
                    elif line[0] == 'G21':  # mm
                        self.unit = 1

                    # change tool
                    elif line[0] == 'T0':
                        self.tool = 0
                    elif line[0] == 'T1':
                        self.tool = 1

                    elif line[0] == 'M107' or line[0] == 'M106':  # fan control
                        command = 48
                        command |= 0  # TODO: change this part, consder muti-fan control protocol
                        self.writer(packer(command), output_stream)
                        if line[0] == 'M107':  # close the fan
                            self.writer(packer_f(0.0), output_stream)
                        elif line[0] == 'M106':
                            if len(line) != 1:
                                self.writer(packer_f(float(line[1][1:]) / 255.), output_stream)
                            else:
                                self.writer(packer_f(1.), output_stream)

                    elif line[0] in ['M84', 'M140']:  # loosen the motor
                        pass  # should only appear when printing done, not define in fcode yet

                    elif line[0] == 'M25' or line[0] == 'M0':  # pause by gcode
                        command = 5
                        self.writer(packer(command), output_stream)

                    else:
                        if line[0] in ['M400']:  # TODO: define a white list
                            pass
                        else:
                            logger.info('Undefine gcode: {}'.format(line))
                else:
                    if self.engine == 'cura':
                        if 'FILL' in comment:
                            self.now_type = PY_TYPE_INFILL
                        elif 'SUPPORT' in comment:
                            self.now_type = PY_TYPE_SUPPORT
                        elif 'LAYER:' in comment:
                            self.now_type = PY_TYPE_NEW_LAYER
                        elif 'WALL-OUTER' in comment:
                            self.now_type = PY_TYPE_PERIMETER
                            if self.highlight_layer == self.layer_now:
                                self.now_type = PY_TYPE_HIGHLIGHT
                        elif 'WALL-INNER' in comment:
                            self.now_type = PY_TYPE_INNER_WALL
                        elif 'RAFT' in comment:
                            self.now_type = PY_TYPE_RAFT
                        elif 'SKIRT' in comment:
                            self.now_type = PY_TYPE_SKIRT
                        elif 'SKIN' in comment:
                            self.now_type = PY_TYPE_SKIN

            self.T = Thread(target=self.sub_convert_path)
            self.T.start()
            # write back crc and lengh info
            output_stream.write(struct.pack('<I', self.crc))
            output_stream.seek(len(self.header()), 0)
            output_stream.write(struct.pack('<I', self.script_length))
            output_stream.seek(0, 2)  # go back to file end

            if len(self.empty_layer) > 0 and self.empty_layer[0] == 0:  # clean up first empty layer
                self.empty_layer.pop(0)

            # warning: fileformat didn't consider multi-extruder, use first extruder instead
            if self.filament[0] and 'HEAD_TYPE' not in self.md:
                self.md['HEAD_TYPE'] = 'EXTRUDER'

            if self.md['HEAD_TYPE'] == 'EXTRUDER':
                self.md['FILAMENT_USED'] = ','.join(map(str, self.filament))
                # self.md['CORRECTION'] = 'A'
                self.md['SETTING'] = str(comment_list[-137:])
            else:
                self.md['CORRECTION'] = 'N'

            self.md['TRAVEL_DIST'] = str(self.distance)

            self.max_range[3] = sqrt(self.max_range[3])
            for v, k in enumerate(['X', 'Y', 'Z', 'R']):
                self.md['MAX_' + k] = str(self.max_range[v])

            self.md['TIME_COST'] = str(self.time_need)
            self.md['CREATED_AT'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.localtime(time.time()))
            self.md['AUTHOR'] = getuser()  # TODO: use fluxstudio user name?

            self.write_metadata(output_stream)

        except Exception as e:
            import traceback
            logger.info('G_to_F fail')
            traceback.print_exc(file=sys.stdout)
            return 'broken'


def arc(p_1, p_2, p_c, clock=True, sample_n=100):
    """
    a function dealing with G2, G3 commands
    ref: http://www.cnccookbook.com/CCCNCGCodeArcsG02G03Part2.htm
    """
    _p_1 = [p_1[i] - p_c[i] for i in range(2)]
    _p_2 = [p_2[i] - p_c[i] for i in range(2)]

    r_1 = sqrt(sum(i ** 2 for i in _p_1))
    r_2 = sqrt(sum(i ** 2 for i in _p_2))

    # assert r_1 - r_2 < 0.001  # should be the same

    theta_1 = atan2(_p_1[1], _p_1[0])

    theta_2 = atan2(_p_2[1], _p_2[0])
    if theta_2 > theta_1 and clock:
        theta_2 -= 2 * pi
    elif theta_2 < theta_1 and not clock:
        theta_2 += 2 * pi

    ret = []
    r = r_1
    for t in range(sample_n + 1):
        ratio = t / sample_n
        theta = ratio * (theta_2) + (1 - ratio) * (theta_1)
        np = [p_c[0] + r * cos(theta), p_c[1] + r * sin(theta), ratio * (p_2[2]) + (1 - ratio) * (p_1[2])]
        ret.append(np)

    return ret
