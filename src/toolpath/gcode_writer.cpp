
#include <stdexcept>
#include "gcode.h"

FLUX::GCodeWriterBase::GCodeWriterBase() {
    t = 0;
}

void FLUX::GCodeWriterBase::moveto(int flags, float feedrate, float x, float y, float z, float e0, float e1, float e2) {
    int e_count = 0,
        new_t = -1,
        size;

    for(int i=0;i<3;i++) {
        if(flags & FLAG_HAS_E(i)) {
            new_t = i;
            e_count++;
        }
    }
    if(e_count > 1) {
        throw std::runtime_error("CAN_NOT_HANDLE_MULTI_E");
    } else if(e_count == 1) {
        if(new_t != t) {
            t = new_t;
            size = snprintf(buffer, 32, "T%i\n", t);
            write(buffer, size);
        }
    }

    write("G1", 2);
    if(flags & FLAG_HAS_FEEDRATE) {
        size = snprintf(buffer, 32, " F%.4f", feedrate);
        write(buffer, size);
    }
    if(flags & FLAG_HAS_X) {
        size = snprintf(buffer, 32, " X%.4f", x);
        write(buffer, size);
    }
    if(flags & FLAG_HAS_Y) {
        size = snprintf(buffer, 32, " Y%.4f", y);
        write(buffer, size);
    }
    if(flags & FLAG_HAS_Z) {
        size = snprintf(buffer, 32, " Z%.4f", z);
        write(buffer, size);
    }

    if(e_count == 1) {
        switch(t) {
            case 0:
                size = snprintf(buffer, 32, " E%.4f", e0);
                break;
            case 1:
                size = snprintf(buffer, 32, " E%.4f", e1);
                break;
            case 2:
                size = snprintf(buffer, 32, " E%.4f", e2);
                break;
            default:
                size = 0;
        }
        write(buffer, size);
    }

    write("\n", 1);
}

void FLUX::GCodeWriterBase::sleep(float seconds) {
    int size;
    if(seconds > 1 && (((int)(seconds * 1000) % 1000) < 1)) {
        size = snprintf(buffer, 32, "G4 S%i", (int)(seconds));
        write(buffer, size);
    } else if(seconds > 0) {
        size = snprintf(buffer, 32, "G4 P%i", (int)(seconds * 1000));
        write(buffer, size);
    }

    write("\n", 1);
}

void FLUX::GCodeWriterBase::enable_motor(void) {
    write("M17", 3);
    write("\n", 1);
}

void FLUX::GCodeWriterBase::disable_motor(void) {
    write("M84", 3);
    write("\n", 1);
}

void FLUX::GCodeWriterBase::pause(bool to_standby_position) {
    if(to_standby_position) {
        write("M25\n", 5);
    } else {
        write("M25 Z0\n", 4);
    }
}

void FLUX::GCodeWriterBase::home(void) {
    write("G28", 3);
    write("\n", 1);
}

void FLUX::GCodeWriterBase::set_toolhead_heater_temperature(float temperature, bool wait) {
    int size;
    if(wait) {
        size = snprintf(buffer, 32, "M109 S%.1f", temperature);
    } else {
        size = snprintf(buffer, 32, "M104 S%.1f", temperature);
    }
    write(buffer, size);
    write("\n", 1);
}

void FLUX::GCodeWriterBase::set_toolhead_fan_speed(float strength) {
    if(strength > 0) {
        int size;
        size = snprintf(buffer, 32, "M106 S%i", (int)(strength * 255));
        write(buffer, size);
        write("\n", 1);
    } else {
        write("M107\n", 5);
    }
}

void FLUX::GCodeWriterBase::set_toolhead_pwm(float strength) {
    int size;
    size = snprintf(buffer, 32, "X2O%i", (int)(strength * 255));
    write(buffer, size);
    write("\n", 1);
}


void FLUX::GCodeWriterBase::append_anchor(uint32_t value) {
    int size;
    size = snprintf(buffer, 32, ";anchor=%i\n", value);
    write(buffer, size);
    write("\n", 1);
}

void FLUX::GCodeWriterBase::append_comment(const char* message, size_t length) {
    write(";", 1);
    write(message, length);
    write("\n", 1);
}

void FLUX::GCodeWriterBase::on_error(bool critical, const char* message, size_t length) {
    if(critical) {
        write("\n; >>>>>>>>>> ERROR: ", 21);
    } else {
        write("\n; >>>>>>>>>> WARNING: ", 23);
    }
    write(message, length);
    write("\n", 1);
}


// GCodeMemoryWriter
FLUX::GCodeMemoryWriter::GCodeMemoryWriter(void) {
    stream = new std::stringstream();
    opened = true;
}


FLUX::GCodeMemoryWriter::~GCodeMemoryWriter(void) {
    delete stream;
}


void FLUX::GCodeMemoryWriter::write(const char* buf, size_t size) {
    if(opened) {
        stream->write(buf, size);
    }
}


void FLUX::GCodeMemoryWriter::terminated(void) {
    opened = false;
}


std::string FLUX::GCodeMemoryWriter::get_buffer(void) {
    return stream->str();
}

// GCodeFileWriter
FLUX::GCodeFileWriter::GCodeFileWriter(const char* filename) {
    stream = new std::ofstream(filename);
    if(stream->fail()) {
        throw std::runtime_error("OPEN FILE ERROR");
    }
}


FLUX::GCodeFileWriter::~GCodeFileWriter(void) {
    if(stream->is_open()) {
        terminated();
    }
    delete stream;
}

void FLUX::GCodeFileWriter::write(const char* buf, size_t size) {
    if(stream->is_open()) {
        stream->write(buf, size);
    }
}


void FLUX::GCodeFileWriter::terminated(void) {
    if(stream->is_open()) stream->close();
}
