
#include<stdexcept>
#include "py_processor.h"


FLUX::PythonToolpathProcessor::PythonToolpathProcessor(PyObject *python_callback) {
    callback = python_callback;
}


void FLUX::PythonToolpathProcessor::moveto(int flags, float feedrate, float x, float y, float z, float e0, float e1, float e2) {
    PyObject *arglist = Py_BuildValue("(s)", "moveto");
    PyObject *dictlist = Py_BuildValue("{s:i,s:f,s:f,s:f,s:f,s:(fff)}",
                                       "flags", flags, "feedrate", feedrate,
                                       "x", x, "y", y, "z", z,
                                       "e", e0, e1, e2);
    PyObject_Call(callback, arglist, dictlist);
    Py_DECREF(arglist);
    Py_DECREF(dictlist);
    if(PyErr_Occurred()) {
        throw std::runtime_error("PYERROR");
    }
}

void FLUX::PythonToolpathProcessor::sleep(float milliseconds) {
    PyObject *arglist = Py_BuildValue("(s)", "sleep");
    PyObject *dictlist = Py_BuildValue("{s:f}", "milliseconds", milliseconds);
    PyObject_Call(callback, arglist, dictlist);
    Py_DECREF(arglist);
    Py_DECREF(dictlist);
    if(PyErr_Occurred()) {
        throw std::runtime_error("PYERROR");
    }
}

void FLUX::PythonToolpathProcessor::enable_motor(void) {
    PyObject *arglist = Py_BuildValue("(s)", "enable_motor");
    PyObject_CallObject(callback, arglist);
    Py_DECREF(arglist);
    if(PyErr_Occurred()) {
        throw std::runtime_error("PYERROR");
    }
}

void FLUX::PythonToolpathProcessor::disable_motor(void) {
    PyObject *arglist = Py_BuildValue("(s)", "disable_motor");
    PyObject_CallObject(callback, arglist);
    Py_DECREF(arglist);
    if(PyErr_Occurred()) {
        throw std::runtime_error("PYERROR");
    }
}

void FLUX::PythonToolpathProcessor::pause(bool to_standby_position) {
    PyObject *arglist = Py_BuildValue("(s)", "pause");
    PyObject *dictlist = Py_BuildValue("{s:b}", "to_standby_position", to_standby_position);
    PyObject_Call(callback, arglist, dictlist);
    Py_DECREF(arglist);
    Py_DECREF(dictlist);
    if(PyErr_Occurred()) {
        throw std::runtime_error("PYERROR");
    }
}

void FLUX::PythonToolpathProcessor::home(void) {
    PyObject *arglist = Py_BuildValue("(s)", "home");
    PyObject_CallObject(callback, arglist);
    Py_DECREF(arglist);
    if(PyErr_Occurred()) {
        throw std::runtime_error("PYERROR");
    }
}

void FLUX::PythonToolpathProcessor::set_toolhead_heater_temperature(float temperature, bool wait) {
    PyObject *arglist = Py_BuildValue("(s)", "set_toolhead_heater_temperature");
    PyObject *dictlist = Py_BuildValue("{s:f,s:b}", "temperature", temperature, "wait", wait);
    PyObject_Call(callback, arglist, dictlist);
    Py_DECREF(arglist);
    Py_DECREF(dictlist);
    if(PyErr_Occurred()) {
        throw std::runtime_error("PYERROR");
    }
}

void FLUX::PythonToolpathProcessor::set_toolhead_fan_speed(float strength) {
    PyObject *arglist = Py_BuildValue("(s)", "set_toolhead_fan_speed");
    PyObject *dictlist = Py_BuildValue("{s:f}", "strength", strength);
    PyObject_Call(callback, arglist, dictlist);
    Py_DECREF(arglist);
    Py_DECREF(dictlist);
    if(PyErr_Occurred()) {
        throw std::runtime_error("PYERROR");
    }
}

void FLUX::PythonToolpathProcessor::set_toolhead_pwm(float strength) {
    PyObject *arglist = Py_BuildValue("(s)", "set_toolhead_pwm");
    PyObject *dictlist = Py_BuildValue("{s:f}", "strength", strength);
    PyObject_Call(callback, arglist, dictlist);
    Py_DECREF(arglist);
    Py_DECREF(dictlist);
    if(PyErr_Occurred()) {
        throw std::runtime_error("PYERROR");
    }
}

void FLUX::PythonToolpathProcessor::append_anchor(uint32_t value) {
    PyObject *arglist = Py_BuildValue("(s)", "append_anchor");
    PyObject *dictlist = Py_BuildValue("{s:i}", "value", value);
    PyObject_Call(callback, arglist, dictlist);
    Py_DECREF(arglist);
    Py_DECREF(dictlist);
    if(PyErr_Occurred()) {
        throw std::runtime_error("PYERROR");
    }
}

void FLUX::PythonToolpathProcessor::append_comment(const char* message, size_t length) {
    PyObject *arglist = Py_BuildValue("(s)", "append_comment");
    PyObject *dictlist = Py_BuildValue("{s:s#}", "message", message, length);
    PyObject_Call(callback, arglist, dictlist);
    Py_DECREF(arglist);
    Py_DECREF(dictlist);
    if(PyErr_Occurred()) {
        throw std::runtime_error("PYERROR");
    }
}

void FLUX::PythonToolpathProcessor::on_error(bool critical, const char* message, size_t length) {
    PyObject *arglist = Py_BuildValue("(s)", "on_error");
    PyObject *dictlist = Py_BuildValue("{s:b,s:s#}",
                                       "critical", critical,
                                       "message", message, length);
    PyObject_Call(callback, arglist, dictlist);
    Py_DECREF(arglist);
    Py_DECREF(dictlist);
    if(PyErr_Occurred()) {
        throw std::runtime_error("PYERROR");
    }
}

void FLUX::PythonToolpathProcessor::terminated(void) {

}
