# !/usr/bin/env python3

import sys
from re import findall
from json import dumps

from fluxclient.hw_profile import HW_PROFILE

POINT_TYPE = {'infill': 0,
              'perimeter': 1,
              'support': 2,
              'move': 3,
              'skirt': 4,
              'inner-wall': 5,
              'brim': 5,
              'raft': 2}


class FcodeBase(object):
    """
    class dealing with gcode <-> fcode... etc
    fcode format: https://github.com/flux3dp/fluxmonitor/wiki/Flux-Device-Control-Describe-File-V1
    """
    def __init__(self):
        super(FcodeBase, self).__init__()
        self.filament_this_layer = [0., 0., 0.]
        self.current_pos = [0.0, 0.0, HW_PROFILE['model-1']['height'], 0.0, 0.0, 0.0]  # X, Y, Z, E1, E2, E3 -> recording the position of each axis
        self.path = [[[0.0, 0.0, HW_PROFILE['model-1']['height'], POINT_TYPE['move']]]]  # recording the path extruder go through
        self.empty_layer = []
        self.counter_between_layers = 0
        self.record_z = 0.0
        self.engine = 'slic3r'
        self.now_type = 3

    def get_path(self):
        return self.path

    def process_path(self, comment, move_flag, extrude_flag):
        """
        convert to path list(for visualizing)
        """
        # TODO?: reconsider if theese two flag necessary
        if self.engine == 'slic3r':
            already_split = False
            self.counter_between_layers += 1
            if move_flag:
                if 'infill' in comment:
                    line_type = POINT_TYPE['infill']
                elif 'support' in comment:
                    line_type = POINT_TYPE['support']
                elif 'brim' in comment:
                    line_type = POINT_TYPE['brim']
                elif 'move' in comment:
                    line_type = POINT_TYPE['move']
                    if 'to next layer' in comment:
                        self.record_z = self.current_pos[2]
                        already_split = True
                        if self.filament == self.filament_this_layer and self.counter_between_layers > 1:
                            self.empty_layer.append(self.layer_now)
                        tmp = findall('[0-9]+', comment)[-1]
                        self.counter_between_layers = 0
                        self.layer_now = int(tmp)
                        self.path.append([self.path[-1][-1][:3] + [line_type]])
                        self.filament_this_layer = self.filament[:]
                elif 'perimeter' in comment:
                    line_type = 1
                elif 'skirt' in comment:
                    line_type = 4
                elif 'draw' in comment:
                    line_type = 0
                else:   # no data in comment
                    if extrude_flag:
                        line_type = 1
                    else:
                        line_type = POINT_TYPE['move']

                self.path[-1].append(self.current_pos[:3] + [line_type])

                if len(comment) == 0 and not already_split and self.current_pos[2] - self.record_z > 0.3:  # 0.3 is the max layer height in fluxstudio
                    self.path.append([self.path[-1][-1][:3] + [line_type]])
                    self.record_z = self.current_pos[2]
                    self.layer_now = len(self.path)

        elif self.engine == 'cura':
            already_split = False
            self.counter_between_layers += 1
            line_type = POINT_TYPE['move']
            if self.now_type == -1:
                self.now_type = POINT_TYPE['move']
                self.record_z = self.current_pos[2]
                already_split = True
                if self.filament == self.filament_this_layer and self.counter_between_layers > 1:
                    self.empty_layer.append(self.layer_now)
                # tmp = findall('[0-9]+', comment)[-1]
                self.counter_between_layers = 0
                # self.layer_now = int(tmp)
                self.layer_now = len(self.path)
                self.path.append([self.path[-1][-1][:3] + [self.now_type]])
                self.filament_this_layer = self.filament[:]

            if move_flag:
                if extrude_flag:
                    line_type = self.now_type
                elif not extrude_flag:
                    line_type = POINT_TYPE['move']

                self.path[-1].append(self.current_pos[:3] + [line_type])

    @classmethod
    def path_to_js(cls, path):
        """
        transform path:[[[],[]]] to js object
        """
        if path is None:
                return ''
        else:
            result = []
            for layer in path:
                tmp = []
                for p in layer:
                    tmp.append({'t': p[3], 'p': [round(p[0], 2), round(p[1], 2), round(p[2], 2)]})
                result.append(tmp)
            return dumps(result)

    @classmethod
    def trim_ends(cls, path):
        """
        trim the moving(non-extruding) path in path's both end
        """
        for layer in [0, -1]:
            while True:
                if path[layer]:
                    if path[layer][layer][3] == POINT_TYPE['move']:
                        path[layer].pop(layer)
                    else:
                        break
                else:
                    path.pop(layer)
        return path
