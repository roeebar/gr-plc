/*
 * Gr-plc - IEEE 1901 module for GNU Radio
 * Copyright (C) 2016 Roee Bar <roeeb@ece.ubc.ca>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include "defs.h"

namespace light_plc {

void set_field(vector_int &bit_vector, int bit_offset, int bit_width, unsigned long new_value) {
    for (int i=0; i<bit_width; i++) {
        bit_vector[bit_offset + i] = new_value & 0x1;
        new_value >>= 1;
    }
    assert(new_value == 0);
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
