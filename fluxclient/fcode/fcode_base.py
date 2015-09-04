import sys


class FcodeBase(object):
    """
    class dealing with gcode <-> fcode... etc
    fcode format: https://github.com/flux3dp/fluxmonitor/wiki/Flux-Device-Control-Describe-File-V1
    """
    def __init__(self):
        super(FcodeBase, self).__init__()

    def process_path(self, comment, moveflag, extrudeflag):
        # TODO?: reconsider is theese two flag necessary
        if self.record_path and moveflag:  # pi need no to record_path
            line_type = None
            if 'infill' in comment:
                line_type = 0
            elif 'perimeter' in comment:
                line_type = 1
            elif 'support' in comment:
                line_type = 2
            if 'move' in comment:
                line_type = 3
            elif 'skirt' in comment:
                line_type = 4
            elif 'draw' in comment:
                line_type = 1

            if line_type is None:
                if extrudeflag:
                    line_type = 1
                else:
                    line_type = 3

            if self.current_pos[2] != self.record_z:  # merge?
                # TODO: clean magic number?
                # a real model should >= 4, and deal with the case that spiral appear use 0.02 as a layer height
                if len(self.path[-1]) > 4 and abs(self.path[-1][0][2] - self.current_pos[2]) > 0.5:
                    self.path.append([self.path[-1][-1][:3] + [line_type]])

            self.path[-1].append(self.current_pos[:3] + [line_type])
            self.record_z = self.current_pos[2]
