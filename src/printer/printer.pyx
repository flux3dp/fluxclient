import cython
from libcpp.vector cimport vector

cdef extern from "printer_module.h":
    cdef cppclass MeshPtr:
        pass
    MeshPtr createMeshPtr()
    int set_point()
    int set_point(MeshPtr triangles, vector[vector [float]] points)
    int push_backFace(MeshPtr triangles, int v0, int v1, int v2)
    int add_on(MeshPtr base, MeshPtr new_mesh)
    int STL_to_List(MeshPtr triangles, vector[vector [vector [float]]] &data)
    int apply_transform(MeshPtr triangles, float x, float y, float z, float rx, float ry, float rz, float scale)

cdef class MeshObj:
    cdef MeshPtr meshobj

    def __init__(self, point_list, face_indice):
        self.meshobj = createMeshPtr()
        set_point(self.meshobj, point_list)

        for i in face_indice:
            push_backFace(self.meshobj, i[0], i[1], i[2])

    def apply_transform(self, transform_param):
        # transform_param:  x, y, z, rx, ry, rz, scale
        x, y, z, rx, ry, rz, scale = transform_param
        apply_transform(self.meshobj, x, y, z, rx, ry, rz, scale)

    def add_on(self, MeshObj new_mesh):
        add_on(self.meshobj, new_mesh.meshobj)

    def bounding_box(self):
        pass
        # bounding_box = [[minx, miny, minz], [maxx, maxy, maxz]]
        # bounding_box(self.meshobj)






