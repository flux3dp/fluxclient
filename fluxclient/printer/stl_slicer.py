# !/usr/bin/env python3
try:
    import fluxclient.scanner._printer as _printer
except:
    pass


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
        ## psudo code
        ## self.mesh = Mesh(pcl mesh)

        for i in names:
            m_mesh = _printer.MeshObj()
            ## self.mesh.add_on(names)
            ## in add on, do the moving and rotating

        ## mesh.store('tmp file name')
        ## io, store a fucking mesh
        ## raise cmd line command "slic3er tmp_file_name ... "

        ############### fake code ###############
        gcode = ""
        metadata = [0]
        ############### fake code ###############
        return gcode, metadata
