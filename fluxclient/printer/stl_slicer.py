# !/usr/bin/env python3
import struct

try:
    import fluxclient.scanner._printer as _printer
except:
    pass


def read_stl(file_data):
    # https://en.wikipedia.org/wiki/STL_(file_format)
    if type(file_data) == str:
        with open(file_data, 'rb') as f:
            file_data = f.read()
            Byte_Order = '@'
    elif type(file_data) == bytes:
        Byte_Order = '<'
    else:
        raise ValueError('wrong stl data type:%s' % str(type(file_data)))
    if file_data.startswith(b'solid '):
        # ascii stl file
        pass
    else:
        # binary stl file
        header = file_data[:80]
        length = struct.unpack(Byte_Order + 'I', file_data[80:84])[0]

        points = {}  # key: points, value: index
        faces = []
        counter = 0
        for i in range(length):
            index = i * 50 + 84
            # n = struct.unpack(Byte_Order + 'III', file_data[index + (4 * 3 * 0):index + (4 * 3 * 1)])
            v0 = struct.unpack(Byte_Order + 'fff', file_data[index + (4 * 3 * 1):index + (4 * 3 * 2)])
            v1 = struct.unpack(Byte_Order + 'fff', file_data[index + (4 * 3 * 2):index + (4 * 3 * 3)])
            v2 = struct.unpack(Byte_Order + 'fff', file_data[index + (4 * 3 * 3):index + (4 * 3 * 4)])
            # v = struct.unpack(Byte_Order + 'I' * 9, file_data[index + (4 * 3 * 1):index + (4 * 3 * 4)])
            face = []
            for v in [v0, v1, v2]:
                if v not in points:
                    counter
                    points[v] = counter
                    points[counter] = v
                    counter += 1
                face.append(points[v])
            faces.append(face)
        points_list = []
        for i in range(counter):
            points_list.append(points[i])
    return points_list, faces


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
            points, faces = read_stl(buf)
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
