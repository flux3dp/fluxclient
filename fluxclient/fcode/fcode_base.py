import sys


class FcodeBase(object):
    """
    class dealing with gcode <-> fcode... etc
    fcode format: https://github.com/flux3dp/fluxmonitor/wiki/Flux-Device-Control-Describe-File-V1
    """
    def __init__(self):
        super(FcodeBase, self).__init__()

    def process_path(self, comment, moveflag, extrudeflag):
        if self.record_path:  # pi need no to record_path
            if moveflag:
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
                if self.current_pos[2] != self.record_z:
                    if self.path[-1] != []:
                        self.path.append([])
                self.record_z = self.current_pos[2]
                self.path[-1].append(self.current_pos[:3] + [line_type])
