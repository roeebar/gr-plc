#include "defs.h"

namespace light_plc {

void set_field(vector_int &bit_vector, int bit_offset, int bit_width, unsigned long new_value) {
    for (int i=0; i<bit_width; i++) {
        bit_vector[bit_offset + i] = new_value & 0x1;
        new_value >>= 1;
    }
    return;
}

unsigned long get_field(const vector_int &bit_vector, int bit_offset, int bit_width) {
    unsigned long value = 0;
    for (int i=0; i<bit_width; i++)  {
        value |= bit_vector[bit_offset + i] << i;
    }
    return value;
}

} /* namespace light_plc */
