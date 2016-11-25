import cython
import sys
from libcpp.vector cimport vector


cdef extern from "scan_module.h":
    cdef cppclass PointCloudXYZRGBPtr:
        pass
    cdef cppclass NormalPtr:
        pass
    cdef cppclass PointXYZRGBNormalPtr:
        pass
    cdef cppclass MeshPtr:
        pass

    PointCloudXYZRGBPtr createPointCloudXYZRGB()
    NormalPtr createNormalPtr()
    MeshPtr createMeshPtr()
    int loadPointCloudXYZRGB(const char* file, PointCloudXYZRGBPtr cloud)
    void dumpPointCloudXYZRGB(const char* file, PointCloudXYZRGBPtr cloud)
    void push_backPoint(PointCloudXYZRGBPtr cloud, float x, float y, float z, cython.uint r, cython.uint g, cython.uint b)
    int get_item(PointCloudXYZRGBPtr cloud, int key, vector[float] point)
    int get_w(PointCloudXYZRGBPtr cloud)
    int apply_transform(PointCloudXYZRGBPtr cloud, NormalPtr normals, PointXYZRGBNormalPtr both, float x, float y, float z, float rx, float ry, float rz)

    int clone(PointCloudXYZRGBPtr obj, PointCloudXYZRGBPtr obj2)
    int clone(NormalPtr normalObj, NormalPtr normalObj2)
    int clone(PointXYZRGBNormalPtr bothobj, PointXYZRGBNormalPtr bothobj2)
    int clone(MeshPtr meshobj, MeshPtr meshobj2)

    PointCloudXYZRGBPtr add(PointCloudXYZRGBPtr obj, PointCloudXYZRGBPtr obj2)
    NormalPtr add(NormalPtr normalObj, NormalPtr normalObj2)
    PointXYZRGBNormalPtr add(PointXYZRGBNormalPtr bothobj, PointXYZRGBNormalPtr bothobj2)

    int split(PointXYZRGBNormalPtr bothobj, PointCloudXYZRGBPtr obj, NormalPtr normalObj)

    int SOR(PointCloudXYZRGBPtr cloud, int neighbors, float thresh)
    int Euclidean_Cluster(PointCloudXYZRGBPtr cloud, float thres_dist, vector[vector [int]] &output)

    int ne(PointCloudXYZRGBPtr cloud, NormalPtr normals, float radius)
    int ne_viewpoint(PointCloudXYZRGBPtr cloud, NormalPtr normals, float radius )

    PointXYZRGBNormalPtr createPointXYZRGBNormalPtr()

    PointXYZRGBNormalPtr concatenatePointsNormal(PointCloudXYZRGBPtr cloud, NormalPtr normals)

    int POS(PointXYZRGBNormalPtr cloud_with_normals, MeshPtr triangles, PointCloudXYZRGBPtr cloud, float smooth)
    int GPT(PointXYZRGBNormalPtr cloud_with_normals, MeshPtr triangles, PointCloudXYZRGBPtr cloud)
    # int STL_to_Faces(MeshPtr, vector[vector [int]] &viewp)
    int STL_to_List(MeshPtr triangles, vector[vector[vector [float]]] &data)
    int cut(PointCloudXYZRGBPtr input, PointCloudXYZRGBPtr output, int mode, int direction, float value)

cdef class PointCloudXYZRGBObj:
    cdef PointCloudXYZRGBPtr obj
    cdef NormalPtr normalObj
    cdef PointXYZRGBNormalPtr bothobj
    cdef MeshPtr meshobj

    def __init__(self):
        self.obj = createPointCloudXYZRGB()
        self.normalObj = createNormalPtr()
        self.meshobj = createMeshPtr()
        self.bothobj = createPointXYZRGBNormalPtr()

    def __len__(self):
        return get_w(self.obj)

    def __getitem__(self, key):
        return self.get_item(key)

    cpdef add(self, PointCloudXYZRGBObj other):
        cdef PointCloudXYZRGBObj pc = PointCloudXYZRGBObj()
        pc.obj = add(self.obj, other.obj)
        pc.normalObj = add(self.normalObj, other.normalObj)
        pc.bothobj = add(self.bothobj, other.bothobj)
        return pc

    cpdef loadFile(self, unicode filename):
        if loadPointCloudXYZRGB(filename.encode(), self.obj) == -1:
            raise RuntimeError("Load failed")

    cpdef PointCloudXYZRGBObj cut(self, int mode, int direction, float value):
        cdef PointCloudXYZRGBObj pc = PointCloudXYZRGBObj()
        cut(self.obj, pc.obj, mode, direction, value);
        return pc

    cpdef PointCloudXYZRGBObj clone(self):
        cdef PointCloudXYZRGBObj pc = PointCloudXYZRGBObj()

        clone(self.obj, pc.obj)
        clone(self.normalObj, pc.normalObj)
        clone(self.bothobj, pc.bothobj)
        clone(self.meshobj, pc.meshobj)

        return pc

    cpdef apply_transform(self, x, y, z, rx, ry, rz):
        apply_transform(self.obj, self.normalObj, self.bothobj, x, y, z, rx, ry, rz)

    cpdef int split(self):
        split(self.bothobj, self.obj, self.normalObj)
        return 0

    cpdef dump(self, unicode filename):
        dumpPointCloudXYZRGB(filename.encode(), self.obj)

    cpdef push_backPoint(self, float x, float y, float z, r, g, b):
        push_backPoint(self.obj, x, y, z, r, g, b)

    cpdef get_item(self, key):
        cdef vector[float] point = [0., 0., 0., 0., 0., 0.]
        assert key < len(self), 'get index:%d out of range:%d' % (key, len(self))
        get_item(self.obj, key, point)
        return point

    cpdef int SOR(self, int neighbors, float threshold):
        return SOR(self.obj, neighbors, threshold)

    cpdef Euclidean_Cluster(self, thres_dist):
        cdef vector[vector [int]] output
        Euclidean_Cluster(self.obj, thres_dist, output)
        return output

    # cpdef PointCloudXYZRGBObj subcloud(self, vector [int] indeces):
    #     cdef PointCloudXYZRGBObj pc = PointCloudXYZRGBObj()
    #     for i in indeces:
    #         self.obj, pc.obj)
    #     return pc

    cpdef int ne(self, radius):
        return ne(self.obj, self.normalObj, radius)

    cpdef int ne_viewpoint(self, radius):
        return ne_viewpoint(self.obj, self.normalObj, radius)

    cpdef int concatenatePointsNormal(self):
        self.bothobj = concatenatePointsNormal(self.obj, self.normalObj)
        return 0

    cpdef to_mesh(self, param, method='POS'):

        self.concatenatePointsNormal()
        new_c = PointCloudXYZRGBObj()
        if method == 'POS':
            POS(self.bothobj, self.meshobj, self.obj, param[0])
        elif method == 'GPT':
            GPT(self.bothobj, self.meshobj, self.obj)
        return 0

    # cpdef STL_to_Faces(self):
    #    cdef vector[vector [int]] viewp
    #    STL_to_Faces(self.meshobj, viewp)
    #    return viewp

    cpdef STL_to_List(self):
        cdef vector[vector[vector [float]]] data
        STL_to_List(self.meshobj, data)
        return data



# reg part
cdef extern from "scan_module.h":
    cdef cppclass PointNT:
        pass
    cdef cppclass M4f: # Eigen::Matrix4f (might cause namespace error)
        pass
    cdef cppclass FeatureCloudTPtr:
        pass


    int loadPointNT(const char* file, PointXYZRGBNormalPtr cloud)
    void dumpPointNT(const char* file, PointXYZRGBNormalPtr cloud)
    FeatureCloudTPtr createFeatureCloudTPtr()
    int FE(PointXYZRGBNormalPtr cloud, FeatureCloudTPtr cloud_features, float radius)
    # int SCP(PointXYZRGBNormalPtr object, FeatureCloudTPtr object_features, PointXYZRGBNormalPtr scene, FeatureCloudTPtr scene_features, M4f &transformation, float leaf) except +
    int SCP(PointXYZRGBNormalPtr object, FeatureCloudTPtr object_features, PointXYZRGBNormalPtr scene, FeatureCloudTPtr scene_features, PointXYZRGBNormalPtr object_align, float leaf) except +


cdef class RegCloud:
    cdef PointXYZRGBNormalPtr scene, obj
    cdef FeatureCloudTPtr scene_f, obj_f
    cdef M4f transformation


    def __init__(self, PointCloudXYZRGBObj obj, PointCloudXYZRGBObj scene, setting):
        obj.ne_viewpoint(setting.NeighborhoodDistance)
        obj.concatenatePointsNormal()
        self.obj = createPointXYZRGBNormalPtr()
        clone(obj.bothobj, self.obj)

        scene.ne_viewpoint(setting.NeighborhoodDistance)
        scene.concatenatePointsNormal()
        self.scene = createPointXYZRGBNormalPtr()
        clone(scene.bothobj, self.scene)

        self.obj_f = createFeatureCloudTPtr()
        self.scene_f = createFeatureCloudTPtr()

        # self.object_align = createPointXYZRGBNormalPtr()

    cpdef loadFile(self, unicode filename_scene, unicode filename_obj):
        if loadPointNT(filename_scene.encode(), self.scene) == -1:
            raise RuntimeError("Load failed")
        if loadPointNT(filename_obj.encode(), self.obj) == -1:
            raise RuntimeError("Load failed")

    cpdef dump_o(self, unicode filename):
        dumpPointNT(filename.encode(), self.obj)

    cpdef dump_s(self, unicode filename):
        dumpPointNT(filename.encode(), self.scene)

    cpdef dump(self, unicode filename_1, unicode filename_2):
        self.dump_o(filename_1)
        self.dump_s(filename_2)

    # cpdef int FE(self, float radius):
    #     FE(self.scene, self.scene_f, radius)
    #     FE(self.obj, self.obj_f, radius)
    #     return 0

    cpdef SCP(self, float leaf = 3):
        cdef PointCloudXYZRGBObj pc = PointCloudXYZRGBObj()
        is_converge = SCP(self.obj, self.obj_f, self.scene, self.scene_f, pc.bothobj, leaf)
        pc.split()
        return [is_converge, pc]
