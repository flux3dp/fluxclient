# !/usr/bin/env python3
class StlSlicer(object):
    """slicing objects"""
    def __init__(self):
        super(StlSlicer, self).__init__()
        self.reset()

    def reset(self):
        self.models = {}
        self.parameter = {}

    def upload(self, name, buf):
        self.models[name] = buf

    def set(self, name, parameter):
        self.parameter[name] = parameter

    def generate_gcode(self, names):
        for i in names:
            self.add_in(names)

        ############### fake code ###############
        gcode = ""
        metadata = [0]
        ############### fake code ###############
        return gcode, metadata
