import cython
from libcpp.vector cimport vector
import logging

logger = logging.getLogger(__name__)

cdef extern from "printer_module.h":
    cdef cppclass MeshPtr:
        pass
    cdef cppclass CloudPtr:
        pass
    MeshPtr createMeshPtr()
    CloudPtr createCloudPtr(vector[vector [float]] points)
    int setCloud(MeshPtr triangles, CloudPtr cloud)
    int setPoints(MeshPtr triangles, vector[vector [float]] points)
    int push_backFace(MeshPtr triangles, int v0, int v1, int v2)
    int add_on(MeshPtr base, MeshPtr new_mesh)
    int STL_to_List(MeshPtr triangles, vector[vector [vector [float]]] &data)
    int apply_transform(MeshPtr triangles, float x, float y, float z, float rx, float ry, float rz, float sc_x, float sc_y, float sc_z)
    int bounding_box(MeshPtr triangles, vector[float] &b_box)
    int cut(MeshPtr input_mesh, MeshPtr out_mesh, float floor_v)
    int mesh_len(MeshPtr input_mesh)

    int write_stl_binary(MeshPtr triangles, char* filename)

# cdef extern from "tree_support.h":
#     int add_support(MeshPtr input_mesh, MeshPtr out_mesh, float alpha)

cdef class MeshCloud:
    cdef CloudPtr cloud
    
    def __init__(self, point_list):
        logger.info("Create Cloud Ptr %d" % len(point_list))
        self.cloud = createCloudPtr(point_list)

cdef class MeshObj:
    cdef MeshPtr meshobj

    def __init__(self, MeshCloud point_list, f):
        self.meshobj = createMeshPtr()
        setCloud(self.meshobj, point_list.cloud)

        if type(f).__name__ == 'list':
            logger.info("Create STL from pylist size %d " % len(f))
            #iterate all faces as [faceIndex1, faceIndex2, faceIndex3]
            for v in f:
                push_backFace(self.meshobj, v[0], v[1], v[2])
        else:
            logger.info("Create STL from numpy array size %d " % f.size)
            # note: numpy optimized, do not use range
            i = 0
            while i < f.size:
                push_backFace(self.meshobj, f[i], f[i+1], f[i+2])
                i += 3

    # def add_support(self, alpha):
    #     out_mesh = MeshObj([], [])
    #     add_support(self.meshobj, out_mesh.meshobj, alpha)
    #     return out_mesh

    def apply_transform(self, transform_param):
        # transform_param:  x, y, z, rx, ry, rz, scale
        x, y, z, rx, ry, rz, sc_x, sc_y, sc_z = transform_param
        apply_transform(self.meshobj, x, y, z, rx, ry, rz, sc_x, sc_y, sc_z)

    def add_on(self, MeshObj new_mesh):
        add_on(self.meshobj, new_mesh.meshobj)

    def cut(self, float floor_v):
        out_mesh = MeshObj(MeshCloud([]), [])
        cut(self.meshobj, out_mesh.meshobj, floor_v)
        return out_mesh

    def __len__(self):
        return mesh_len(self.meshobj)

    cpdef write_stl(self, filename):
        write_stl_binary(self.meshobj, filename.encode())

    cpdef bounding_box(self):
        cpdef vector[float] tmp_b_box
        bounding_box(self.meshobj, tmp_b_box)

        b_box = [[0. for i in range(3)] for j in range(2)]
        for i in range(6):
            b_box[i // 3][i % 3] = tmp_b_box[i]

        return b_box
