
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "toolpath.h"


namespace FLUX {
    class FCodeV1Base : public FLUX::ToolpathProcessor {
    protected:
        std::ostream *stream;
        unsigned long script_crc32;
        virtual void write(const char* buf, size_t size, unsigned long *crc32);
        void write(float value, unsigned long *crc32);
        void write(uint32_t value, unsigned long *crc32);
        void write_command(unsigned char cmd, unsigned long *crc32);
    public:
        std::vector<std::string> errors;
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
        virtual void terminated(void) = 0;
    };

    class FCodeV1 : public FLUX::FCodeV1Base {
    protected:
        int script_offset;
        // Return metadata crc32
        unsigned long write_metadata(void);
        void begin(void);
    public:
        std::string *head_type;
        double travled;
        double time_cost;
        float home_x, home_y, home_z;
        float current_feedrate, current_x, current_y, current_z;
        float max_x, max_y, max_z, max_r, filament[3];

        std::vector<std::pair<std::string, std::string>> *metadata;
        std::vector<std::string> *previews;

        FCodeV1(std::string *type, std::vector<std::pair<std::string, std::string>> *file_metadata,
            std::vector<std::string> *image_previews);

        virtual void moveto(int flags, float feedrate, float x, float y, float z, float e0, float e1, float e2);
        virtual void sleep(float seconds);
        virtual void home(void);
        virtual void terminated(void);
    };

    class FCodeV1MemoryWriter : public FLUX::FCodeV1 {
    protected:
        bool opened;
    public:
        FCodeV1MemoryWriter(
            std::string *type, std::vector<std::pair<std::string, std::string>> *file_metadata,
            std::vector<std::string> *image_previews);
        ~FCodeV1MemoryWriter(void);
        std::string get_buffer(void);
        virtual void write(const char* buf, size_t size, unsigned long *crc32);
        virtual void terminated(void);
    };

    class FCodeV1FileWriter : public FLUX::FCodeV1 {
    public:
        FCodeV1FileWriter(const char* filename,
            std::string *type, std::vector<std::pair<std::string, std::string>> *file_metadata,
            std::vector<std::string> *image_previews);
        ~FCodeV1FileWriter(void);
        virtual void write(const char* buf, size_t size, unsigned long *crc32);
        virtual void terminated(void);
    };
}