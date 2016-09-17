import cython
import sys
from libcpp.vector cimport vector
cimport libc.stdlib
from libcpp.string cimport string


cdef extern from "utils_module.h":
	ctypedef struct PathVector:
		pass
	string path_to_js(vector[vector[vector [float]]] output)
	string path_to_js_cpp(vector[vector[PathVector]] output)


cdef class Tools:
	def __init__(self):
		pass

	cpdef path_to_js(self, path):
		cdef vector[vector[vector [float]]] origin;
		cdef vector[vector[PathVector]] native;
		if(type(path) == type(origin)):
			return path_to_js(path)

		if(type(path) == type(native)):
			return path_to_js_cpp(path)
