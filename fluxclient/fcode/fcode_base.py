import sys


class FcodeBase(object):
    """
    class dealing with gcode <-> fcode... etc
    fcode format: https://github.com/flux3dp/fluxmonitor/wiki/Flux-Device-Control-Describe-File-V1
    """
    def __init__(self):
        super(FcodeBase, self).__init__()

    def process_path(self, comment, moveflag, extrudeflag):
        """
        convert to path list(for visualizing)
        """
        # TODO?: reconsider if theese two flag necessary
        if moveflag:
            if 'infill' in comment:
                line_type = 0
            elif 'perimeter' in comment:
                line_type = 1
            elif 'support' in comment:
                line_type = 2
            elif 'move' in comment:
                line_type = 3
                if 'to next layer' in comment:
                    self.path.append([self.path[-1][-1][:3] + [line_type]])
            elif 'skirt' in comment:
                line_type = 4
            elif 'draw' in comment:
                line_type = 0
            else:   # no data in comment
                if extrudeflag:
                    line_type = 1
                else:
                    line_type = 3

            self.path[-1].append(self.current_pos[:3] + [line_type])
            self.record_z = self.current_pos[2]
