import cython
import sys
from libcpp.vector cimport vector
cimport libc.stdlib
from libcpp.string cimport string


cdef extern from "utils_module.h":
    string path_to_js(vector[vector[vector [float]]] output)


cdef class Tools:
    def __init__(self):
        pass

    cpdef path_to_js(self, path):
        return path_to_js(path)
