#ifndef _LIGHT_PLC_DEFS
#define _LIGHT_PLC_DEFS

#include "ieee1901.inc"
#include <vector>
#include <complex>
#include <array>
#include <assert.h>

namespace light_plc {

enum code_rate_t {
    RATE_1_2 = 0,
    RATE_16_21 = 1,
    RATE_16_18 = 2,
};

enum pb_size_t {
    PB16 = 0,
    PB136 = 1,
    PB520 = 2,
};

enum tone_mode_t {
    TM_STD_ROBO = 0,
    TM_HS_ROBO = 1,
    TM_MINI_ROBO = 2,
    TM_NO_ROBO = 3
};

enum modulation_type_t {
    MT_NULLED = 0,
    MT_BPSK  = 1,
    MT_QPSK  = 2,
    MT_QAM8  = 3,
    MT_QAM16 = 4,
    MT_QAM64 = 5,
    MT_QAM256 = 6,
    MT_QAM1024 = 7,
    MT_QAM4096 = 8
};

enum channel_est_t {
    CE_SOUND = 0,
    CE_PAYLOAD = 1,
    CE_PREAMBLE = 2
};

typedef std::vector<int> vector_int;
typedef std::vector<float> vector_float;
typedef std::complex<float> complex;
typedef std::vector<complex> vector_complex;

typedef std::array<modulation_type_t, IEEE1901_NUMBER_OF_CARRIERS> tone_map_t;
typedef std::array<bool, IEEE1901_NUMBER_OF_CARRIERS> tone_mask_t;
typedef std::array<float, IEEE1901_NUMBER_OF_CARRIERS> tones_float_t;
typedef std::array<complex, IEEE1901_NUMBER_OF_CARRIERS> tones_complex_t;
typedef std::array<bool, IEEE1901_SYNCP_SIZE> sync_tone_mask_t;

typedef struct stats_t {
    float ber;
    size_t n_bits;
    tones_complex_t channel;
    tones_float_t noise_psd;
    tone_mode_t tone_mode;
    tones_float_t snr;
} stats_t;


void set_field(vector_int &bit_vector, int bit_offset, int bit_width, unsigned long new_value);
unsigned long get_field(const vector_int &bit_vector, int bit_offset, int bit_width);

} /* namespace light_plc */

#endif /* _LIGHT_PLC_DEFS */
