cdef class MeshObj:
    cdef MeshPtr meshobj

    def __init__(self, point_list, face_indice):
        self.meshobj = createMeshPtr()
        set_point(self.meshobj, point_list)

        for i in face_indice:
            push_backFace(self.meshobj, i[0], i[1], i[2])

    def apply_transform(self, transform_matrix):




