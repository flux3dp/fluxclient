
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.pair cimport pair
from libcpp cimport bool

cdef extern from "toolpath.h" namespace "FLUX":
    cdef cppclass ToolpathProcessor:
        void moveto(int flags, float feedrate, float x, float y, float z, float e0, float e1, float e2) nogil
        void sleep(float seconds) nogil
        void enable_motor() nogil
        void disable_motor() nogil
        void pause(bool to_standby_position) nogil
        void home() nogil
        void set_toolhead_heater_temperature(float temperature, bool wait) nogil
        void set_toolhead_fan_speed(float strength) nogil
        void set_toolhead_pwm(float strength) nogil
        void append_comment(const char* message, size_t length) nogil
        void on_error(bool critical, const char* message, size_t length) nogil
        void terminated() nogil


cdef extern from "gcode.h" namespace "FLUX":
    cdef cppclass GCodeParser:
        GCodeParser() nogil except +
        void set_processor(ToolpathProcessor*) nogil
        void parse_from_file(const char*) nogil except +
        void parse_command(const char*, size_t) nogil except +

        float feedrate
        float position[3]
        float position_offset[3]
        float filaments[3]
        float filaments_offset[3]
        int T
        bool from_inch
        bool absolute

    cdef cppclass GCodeMemoryWriter:
        GCodeMemoryWriter() nogil
        string get_buffer() nogil

    cdef cppclass GCodeFileWriter:
        GCodeFileWriter(const char* filename) nogil except +


cdef extern from "fcode.h" namespace "FLUX":
    cdef cppclass FCodeV1MemoryWriter:
        FCodeV1MemoryWriter(string*, vector[pair[string, string]]*, vector[string]*) nogil
        string get_buffer() nogil
        vector[pair[string, string]] *metadata
        vector[string] *previews
        vector[string] errors
        double travled
        double time_cost

    cdef cppclass FCodeV1FileWriter:
        FCodeV1FileWriter(const char*, string*, vector[pair[string, string]]*, vector[string]*) nogil
        vector[pair[string, string]] *metadata
        vector[string] *previews
        vector[string] errors


cdef extern from "py_processor.h" namespace "FLUX":
    cdef cppclass PythonToolpathProcessor:
        PythonToolpathProcessor(object) nogil
