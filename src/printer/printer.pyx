import cython
from libcpp.vector cimport vector

from fluxclient.scanner.tools import write_stl

cdef extern from "printer_module.h":
    cdef cppclass MeshPtr:
        pass
    MeshPtr createMeshPtr()
    int set_point(MeshPtr triangles, vector[vector [float]] points)
    int push_backFace(MeshPtr triangles, int v0, int v1, int v2)
    int add_on(MeshPtr base, MeshPtr new_mesh)
    int STL_to_List(MeshPtr triangles, vector[vector [vector [float]]] &data)
    int apply_transform(MeshPtr triangles, float x, float y, float z, float rx, float ry, float rz, float sc_x, float sc_y, float sc_z)
    int bounding_box(MeshPtr triangles, vector[float] &b_box);

cdef class MeshObj:
    cdef MeshPtr meshobj

    def __init__(self, point_list, face_indice):
        self.meshobj = createMeshPtr()
        set_point(self.meshobj, point_list)

        for i in face_indice:
            push_backFace(self.meshobj, i[0], i[1], i[2])

    def apply_transform(self, transform_param):
        # transform_param:  x, y, z, rx, ry, rz, scale
        x, y, z, rx, ry, rz, sc_x, sc_y, sc_z = transform_param
        apply_transform(self.meshobj, x, y, z, rx, ry, rz, sc_x, sc_y, sc_z)

    def add_on(self, MeshObj new_mesh):
        add_on(self.meshobj, new_mesh.meshobj)

    cpdef write_stl(self, file_name):
        cpdef vector[vector [vector [float]]] tri
        STL_to_List(self.meshobj, tri)
        write_stl(tri, file_name, 'ascii')

    cpdef bounding_box(self):
        cpdef vector[float] tmp_b_box
        bounding_box(self.meshobj, tmp_b_box)

        b_box = [[0. for i in range(3)] for j in range(2)]
        for i in range(6):
            b_box[i // 3][i % 3] = tmp_b_box[i]

        return b_box
