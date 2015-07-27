class fcode_base(object):
    """class deal with gcode <-> fcode... etc"""
    def __init__(self):
        super(fcode_base, self).__init__()

    def read_file(self, file_name):
        with open(file_name, 'r') as f:
            if file_name[-6:] == '.gcode':
                return self.gcode_to_list(f.read())
            else:
                raise ValueError('Unrecognized file format%s' % (file_name))

    def gcode_to_list(self, gcode):
        gcode = gcode.rstrip().split('\n')
        return gcode
