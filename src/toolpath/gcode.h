
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "toolpath.h"


namespace FLUX {
    class GCodeParser {
    public:
        float feedrate;

        // AXIS[X, Y, Z]
        float position[3];
        float position_offset[3];

        // AXIS E[T0, T1, T2]
        float filaments[3];
        float filaments_offset[3];

        int T;

        // true if G0/G1/G92 command unit is inch
        bool from_inch;
        // true if G0/G1 command unit is absolute
        bool absolute;

        GCodeParser(void);
        void set_processor(FLUX::ToolpathProcessor* handler);
        void parse_from_file(const char* filepth);
        void parse_command(const char* linep, size_t size);

    protected:
        FLUX::ToolpathProcessor* handler;

        void parse_comment(const char* linep, size_t offset, size_t size);

        int handle_g0g1(const char* linep, int offset, int size);
        int handle_g4(const char* linep, int offset, int size);
        int handle_g28(const char* linep, int offset, int size);
        int handle_g92(const char* linep, int offset, int size);
        int handle_m17(const char* linep, int offset, int size);
        int handle_m18m84(const char* linep, int offset, int size);
        int handle_m24m25m226(const char* linep, int offset, int size);
        int handle_m104m109(const char* linep, int offset, int size, bool wait);
        int handle_m106(const char* linep, int offset, int size);
        int handle_m107(const char* linep, int offset, int size);
        int handle_x2(const char* linep, int offset, int size);
    };

    class GCodeWriterBase : public FLUX::ToolpathProcessor {
    public:
        int t;
        char buffer[32];

        GCodeWriterBase();
        virtual void write(const char* buf, size_t size) = 0;
        virtual void terminated(void) = 0;

        virtual void moveto(int flags, float feedrate, float x, float y, float z, float e0, float e1, float e2);
        virtual void sleep(float seconds);
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
    };


    class GCodeMemoryWriter : public GCodeWriterBase {
    protected:
        bool opened;
        std::stringstream *stream;
    public:
        GCodeMemoryWriter(void);
        ~GCodeMemoryWriter(void);
        std::string get_buffer(void);
        virtual void write(const char* buf, size_t size);
        virtual void terminated(void);
    };

    class GCodeFileWriter : public GCodeWriterBase {
    protected:
        std::ofstream *stream;
    public:
        GCodeFileWriter(const char* filename);
        ~GCodeFileWriter(void);
        virtual void write(const char* buf, size_t size);
        virtual void terminated(void);
    };
}


static inline double inch2mm(float inch) {
    return inch * 25.4;
}
