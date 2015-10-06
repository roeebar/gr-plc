#!/usr/bin/env python
def set_numeric_field(frame, value, offset, width = 1):
    shift = offset % 8
    shifted_length = ((shift + width - 1) / 8) + 1
    prev_byte = 0
    for i in range(shifted_length):
        byte = (value & (0xFF << i*8)) >> i*8
        print byte
        frame[offset/8 + i] |= (byte << shift) & 0xFF | (prev_byte >> (8-shift))
        prev_byte = byte

frame=bytearray(10)
set_numeric_field(frame, 2, 8 ,16)
print ''.join('{:02x}'.format(x) for x in frame)

