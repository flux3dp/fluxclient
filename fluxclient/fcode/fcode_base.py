# !/usr/bin/env python3

import sys
from re import findall
from json import dumps

from fluxclient.hw_profile import HW_PROFILE
from fluxclient.utils._utils import Tools

# define point type
POINT_TYPE = {'new layer': -1,
              'infill': 0,
              'perimeter': 1,  # outer-wall
              'support': 2,
              'move': 3,
              'skirt': 4,
              'inner-wall': 5,
              'brim': 6,
              'raft': 7,  # raft
              'skin': 8  # top and bottom solid layer
              }


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
        self.now_type = POINT_TYPE['move']
        self.path_js = None
        self.highlight_layer = -1

    def sub_convert_path(self):
        # self.path_js = FcodeBase.path_to_js(self.path)
        self.path_js = Tools().path_to_js(self.path).decode()

    def get_path(self, path_type='js'):
        if path_type == 'js':
            self.T.join()
            return self.path_js
        else:
            if self.path:
                return self.path
            else:
                return None

    def process_path(self, comment, move_flag, extrude_flag):
        """
        convert to path list(for visualizing)
        """
        # Point type in constants
        PY_TYPE_NEW_LAYER, PY_TYPE_INFILL, PY_TYPE_PERIMETER, PY_TYPE_SUPPORT, PY_TYPE_MOVE, PY_TYPE_SKIRT, PY_TYPE_INNER_WALL, PY_TYPE_BRIM, PY_TYPE_RAFT, PY_TYPE_SKIN = [x for x in range(-1,9)]

        # Real comparison
        if self.engine == 'slic3r':
            already_split = False
            self.counter_between_layers += 1
            if move_flag:
                if 'infill' in comment:
                    line_type = PY_TYPE_INFILL
                elif 'support' in comment:
                    line_type = PY_TYPE_SUPPORT
                elif 'brim' in comment:
                    line_type = PY_TYPE_BRIM
                elif 'move' in comment:
                    line_type = PY_TYPE_MOVE
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
                    line_type = PY_TYPE_PERIMETER
                    if self.highlight_layer == self.layer_now:
                        self.now_type = PY_TYPE_HIGHLIGHT
                elif 'skirt' in comment:
                    line_type = PY_TYPE_SKIRT
                elif 'draw' in comment:
                    line_type = PY_TYPE_INFILL
                else:   # no data in comment
                    if extrude_flag:
                        line_type = PY_TYPE_PERIMETER
                    else:
                        line_type = PY_TYPE_MOVE

                self.path[-1].append(self.current_pos[:3] + [line_type])

                if len(comment) == 0 and not already_split and self.current_pos[2] - self.record_z > 0.3:  # 0.3 is the max layer height in fluxstudio
                    self.path.append([self.path[-1][-1][:3] + [line_type]])
                    self.record_z = self.current_pos[2]
                    self.layer_now = len(self.path)

        elif self.engine == 'cura':
            already_split = False
            self.counter_between_layers += 1
            line_type = PY_TYPE_MOVE
            if self.now_type == PY_TYPE_NEW_LAYER:
                self.now_type = PY_TYPE_MOVE
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
                    if self.now_type != PY_TYPE_MOVE:
                        line_type = self.now_type
                    else:
                        line_type = PY_TYPE_PERIMETER
                elif not extrude_flag:
                    line_type = PY_TYPE_MOVE

                self.path[-1].append(self.current_pos[:3] + [line_type])

    @classmethod
    def path_to_js(cls, path):
        """
        * NOTE: this is deprecated, use fluxclient.utils._utils.Tools().path_to_js instead
        transform path:[[[],[]]] in to javascript string
        will round number to .2f
        """
        if path is None:
            return ''
        else:
            return dumps([[[round(p[0], 2), round(p[1], 2), round(p[2], 2), p[3]] for p in layer] for layer in path])

    @classmethod
    def trim_ends(cls, path):
        """
        trim the moving(non-extruding) part in path's both end
        """
        PY_TYPE_MOVE = 3
        
        if path is None:
            return path
        if len(path) == 0:
            return path
        for layer in [0, -1]:
            while True:
                if len(path[layer]) >= 2:
                    # define an edge's type at end point
                    # 0 * 2 + 1 = 1
                    # -1 * 2 + 1 = -1
                    if path[layer][layer * 2 + 1][3] == PY_TYPE_MOVE:
                        path[layer].pop(layer)
                    else:
                        break
                else:
                    path.pop(layer)
        return path
