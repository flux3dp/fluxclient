
#ifndef _TOOLPATH_H
#define _TOOLPATH_H

#include <stdint.h>

#define FLAG_HAS_FEEDRATE 64
#define FLAG_HAS_X 32
#define FLAG_HAS_Y 16
#define FLAG_HAS_Z 8
#define FLAG_HAS_AXIS(A) ( 1 << (5 - A + 'X')) 
#define FLAG_HAS_E(T) (1 << (2 - T))


namespace FLUX {
    class ToolpathProcessor {
    public:
        virtual void moveto(int flags, float feedrate, float x, float y, float z, float e0, float e1, float e2) = 0;
        virtual void sleep(float seconds) = 0;
        virtual void enable_motor(void) = 0;
        virtual void disable_motor(void) = 0;
        virtual void pause(bool to_standby_position) = 0;
        virtual void home(void) = 0;
        virtual void set_toolhead_heater_temperature(float temperature, bool wait) = 0;
        virtual void set_toolhead_fan_speed(float strength) = 0;
        virtual void set_toolhead_pwm(float strength) = 0;

        virtual void append_anchor(uint32_t value) = 0;
        virtual void append_comment(const char* message, size_t length) = 0;

        virtual void on_error(bool critical, const char* message, size_t length) = 0;

        virtual void terminated(void) = 0;
    };
}

#endif
