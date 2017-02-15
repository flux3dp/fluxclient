
#include<Python.h>
#include "toolpath.h"

namespace FLUX {
    class PythonToolpathProcessor: FLUX::ToolpathProcessor {
    public:
        PyObject *callback;
        PythonToolpathProcessor(PyObject *python_callback);

        virtual void moveto(int flags, float feedrate, float x, float y, float z, float e0, float e1, float e2);
        virtual void sleep(float milliseconds);
        virtual void enable_motor(void);
        virtual void disable_motor(void);
        virtual void pause(bool to_standby_position);
        virtual void home(void);
        virtual void set_toolhead_heater_temperature(float temperature, bool wait);
        virtual void set_toolhead_fan_speed(float strength);
        virtual void set_toolhead_pwm(float strength);

        virtual void append_anchor(uint32_t value);
        virtual void append_comment(const char* message, size_t length);

        virtual void on_error(bool critical, const char* message, size_t length);

        virtual void terminated(void);
    };

}