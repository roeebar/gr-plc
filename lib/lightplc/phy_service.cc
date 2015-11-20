#include "phy_service.h"
#include "debug.h"
#include <iostream>
#include <algorithm>
#include <functional>
#include <assert.h>
#include <cmath>
#include <itpp/comm/modulator.h>

namespace light_plc {

#include "mapping.inc"

const int phy_service::TCENCODER_SEED_PB16[IEEE1901_TCENCODER_SEED_PB16_N] = {IEEE1901_TCENCODER_SEED_PB16};
const int phy_service::TCENCODER_SEED_PB136[IEEE1901_TCENCODER_SEED_PB136_N] = {IEEE1901_TCENCODER_SEED_PB136};
const int phy_service::TCENCODER_SEED_PB520[IEEE1901_TCENCODER_SEED_PB520_N] = {IEEE1901_TCENCODER_SEED_PB520};

const int phy_service::CHANNEL_INTERLEAVER_OFFSET[3][3] = {IEEE1901_CHANNEL_INTERLEAVER_OFFSET};
const int phy_service::CHANNEL_INTERLEAVER_STEPSIZE[3][3] = {IEEE1901_CHANNEL_INTERLEAVER_STEPSIZE};
static_assert(RATE_1_2==0 && RATE_16_21==1 && RATE_16_18==2,"Interleaving parameters error");
static_assert(PB16==0 && PB136==1 && PB520==2,"Interleaving parameters error");

const std::array<bool, phy_service::NUMBER_OF_CARRIERS+1> phy_service::TRANSMIT_MASK = {{IEEE1901_TRANSMIT_MASK}};
const std::array<bool, phy_service::NUMBER_OF_CARRIERS+1> phy_service::CARRIERS_BROADCAST_MASK = {{IEEE1901_TRANSMIT_MASK}};
const int phy_service::CARRIERS_ANGLE_NUMBER[NUMBER_OF_CARRIERS+1] = {IEEE1901_CARRIERS_ANGLE_NUMBER};
const float phy_service::ROLLOFF_WINDOW_RISE[ROLLOFF_INTERVAL] = {IEEE1901_ROLLOFF_WINDOW_RISE};
const float phy_service::ROLLOFF_WINDOW_FALL[ROLLOFF_INTERVAL] = {IEEE1901_ROLLOFF_WINDOW_FALL};
const float phy_service::SCALE_FACTOR_PAYLOAD = IEEE1901_SCALE_FACTOR_PAYLOAD;
const float phy_service::SCALE_FACTOR_FC = IEEE1901_SCALE_FACTOR_FC;

const phy_service::ModulationMap phy_service::MODULATION_MAP[8] = { 
                         {MAP_BPSK_NBITS, MAP_BPSK, MAP_BPSK_SCALE}, 
                         {MAP_QPSK_NBITS, MAP_QPSK, MAP_QPSK_SCALE}, 
                         {MAP_QAM8_NBITS, MAP_QAM8, MAP_QAM8_SCALE}, 
                         {MAP_QAM16_NBITS, MAP_QAM16, MAP_QAM16_SCALE},
                         {MAP_QAM64_NBITS, MAP_QAM64, MAP_QAM64_SCALE},
                         {MAP_QAM256_NBITS, MAP_QAM256, MAP_QAM256_SCALE},
                         {MAP_QAM1024_NBITS, MAP_QAM1024, MAP_QAM1024_SCALE},
                         {MAP_QAM4096_NBITS, MAP_QAM4096, MAP_QAM4096_SCALE} };

const complex phy_service::ANGLE_NUMBER_TO_VALUE[16] = {   complex(1.000000000000000,0.000000000000000),
                                                    complex(0.923879532511287,0.382683432365090),
                                                    complex(0.707106781186548,0.707106781186547),
                                                    complex(0.382683432365090,0.923879532511287),
                                                    complex(0.000000000000000,1.000000000000000),
                                                    complex(-0.382683432365090,0.923879532511287),
                                                    complex(-0.707106781186547,0.707106781186548),
                                                    complex(-0.923879532511287,0.382683432365090),
                                                    complex(-1.000000000000000,0.000000000000000),
                                                    complex(-0.923879532511287,-0.382683432365090),
                                                    complex(-0.707106781186548,-0.707106781186547),
                                                    complex(-0.382683432365090,-0.923879532511287),
                                                    complex(-0.000000000000000,-1.000000000000000),
                                                    complex(0.382683432365090,-0.923879532511287),
                                                    complex(0.707106781186547,-0.707106781186548),
                                                    complex(0.923879532511287,-0.382683432365090)};

const int phy_service::SYNCP_CARRIERS_ANGLE_NUMBER [SYNCP_SIZE / 2 + 1] = {IEEE1901_SYNCP_CARRIERS_ANGLE_NUMBER};
const bool phy_service::SYNCP_CARRIERS_MASK [SYNCP_SIZE / 2 + 1] = {IEEE1901_SYNCP_CARRIERS_MASK};

const std::array<float, phy_service::NUMBER_OF_CARRIERS*2> phy_service::HAMMING_WINDOW = phy_service::create_hamming_window();
const phy_service::Carriers phy_service::BROADCAST_CARRIERS = phy_service::build_broadcast_carriers();
const int phy_service::N_BROADCAST_CARRIERS = phy_service::count_non_masked_carriers(CARRIERS_BROADCAST_MASK.begin(), CARRIERS_BROADCAST_MASK.end());

phy_service::phy_service (bool debug) : d_debug(debug), d_modulation(QPSK), d_code_rate(RATE_1_2), d_robo_mode(NO_ROBO), d_n0(1), PREAMBLE(calc_preamble()), SYNCP_FREQ(calc_syncp_fft(PREAMBLE)), TURBO_INTERLEAVER_SEQUENCE(calc_turbo_interleaver_sequence()) {
    static_assert(BPSK==0 && QPSK==1 && QAM8==2 && QAM16==3 && QAM64==4 && QAM256==5 && QAM1024==6 && QAM4096==7, "Mapping parameters error");
    create_fftw_vars();
    init_turbo_codec();
    SOUND_PB136 = create_payload_symbols(vector_int(136*8), PB136, MINI_ROBO);
    SOUND_PB520 = create_payload_symbols(vector_int(520*8), PB520, STD_ROBO);
    SOUND_PB136_FREQ = create_payload_symbols_freq(vector_int(136*8), PB136, MINI_ROBO);
    SOUND_PB520_FREQ = create_payload_symbols_freq(vector_int(520*8), PB520, STD_ROBO);
    d_broadcast_channel_response.mask = BROADCAST_CARRIERS.mask;
    d_broadcast_channel_response.amplitude.fill(1);
}

phy_service::phy_service (const phy_service &obj) : 
        d_debug(obj.d_debug),
        d_modulation(obj.d_modulation),
        d_code_rate(obj.d_code_rate),
        d_robo_mode(obj.d_robo_mode),
        d_n0(obj.d_n0),
        PREAMBLE(obj.PREAMBLE), 
        SYNCP_FREQ(obj.SYNCP_FREQ), 
        TURBO_INTERLEAVER_SEQUENCE(obj.TURBO_INTERLEAVER_SEQUENCE),
        SOUND_PB136(obj.SOUND_PB136),
        SOUND_PB520(obj.SOUND_PB520),
        SOUND_PB136_FREQ(obj.SOUND_PB136_FREQ),
        SOUND_PB520_FREQ(obj.SOUND_PB520_FREQ),
        d_broadcast_channel_response(obj.d_broadcast_channel_response) {
    create_fftw_vars();
    init_turbo_codec();
}

phy_service& phy_service::operator=(const phy_service& rhs) {
    phy_service tmp(rhs);
    std::swap(d_ifft_input, tmp.d_ifft_input);
    std::swap(d_ifft_output, tmp.d_ifft_output);
    std::swap(d_fftw_rev_plan, tmp.d_fftw_rev_plan);
    std::swap(d_fft_input, tmp.d_fft_input);
    std::swap(d_fft_output, tmp.d_fft_output);
    std::swap(d_fftw_fwd_plan, tmp.d_fftw_fwd_plan);
    std::swap(d_fft_syncp_input, tmp.d_fft_syncp_input);
    std::swap(d_fft_syncp_output, tmp.d_fft_syncp_output);
    std::swap(d_fftw_syncp_fwd_plan, tmp.d_fftw_syncp_fwd_plan);
    std::swap(d_broadcast_channel_response, tmp.d_broadcast_channel_response);
    std::swap(d_robo_mode, tmp.d_robo_mode);
    std::swap(d_code_rate, tmp.d_code_rate);
    std::swap(d_modulation, tmp.d_modulation);
    std::swap(d_n0, tmp.d_n0);
    std::swap(d_debug, tmp.d_debug);
    return *this;
}

phy_service::~phy_service (void){
    fftwf_free(d_ifft_input);
    fftwf_free(d_ifft_output);    
    fftwf_destroy_plan(d_fftw_rev_plan);    
    fftwf_free(d_fft_input);
    fftwf_free(d_fft_output);
    fftwf_destroy_plan(d_fftw_fwd_plan);
    fftwf_free(d_fft_syncp_input);
    fftwf_free(d_fft_syncp_output);
    fftwf_destroy_plan(d_fftw_syncp_fwd_plan);
}

vector_float phy_service::create_sof_ppdu(const unsigned char *mpdu_payload_bin, size_t len) {
    vector_int mpdu_payload_int(unpack_into_bitvector(mpdu_payload_bin, len));
    return create_sof_ppdu(mpdu_payload_int);
}

vector_float phy_service::create_sof_ppdu(const vector_int &mpdu_payload) {
    DEBUG_ECHO("Encoding payload blocks (SOF)...")

    DEBUG_VECTOR(mpdu_payload);

    // Determine PB size according to number of payload bits
    PbSize pb_size;
    (mpdu_payload.size() > 136*8) ? pb_size = PB520 : pb_size = PB136;

    // Encode payload blocks
    vector_symbol payload_symbols = create_payload_symbols(mpdu_payload, pb_size, d_robo_mode, d_code_rate, d_modulation);

    DEBUG_ECHO("Encoding frame control (SOF)...")
    // Encode frame control
    vector_int mpdu_fc = create_sof_frame_control(payload_symbols.size(), d_robo_mode, d_modulation, pb_size);
    DEBUG_VECTOR(mpdu_fc);
    vector_symbol fc_symbols = create_frame_control_symbol(mpdu_fc);

    DEBUG_ECHO("Creating final data stream...")
    // Create the final symbols array
    vector_symbol symbols;
    symbols.reserve(fc_symbols.size() + payload_symbols.size() + 1);
    symbols.push_back(PREAMBLE);
    symbols.insert(symbols.end(), fc_symbols.begin(), fc_symbols.end());
    symbols.insert(symbols.end(), payload_symbols.begin(), payload_symbols.end());

    // Concatenate the symbols into a continuous stream
    vector_float datastream = symbols_to_datastream(symbols, IEEE1901_RIFS_DEFAULT * SAMPLE_RATE);
    DEBUG_VECTOR(datastream);
    DEBUG_ECHO("Done.")
    return datastream;
}

vector_float phy_service::create_sack_ppdu(const unsigned char *sackd_bin, size_t len) {
    vector_int sackd_int(unpack_into_bitvector(sackd_bin, len));
    return create_sack_ppdu(sackd_int);
}

vector_float phy_service::create_sack_ppdu(const vector_int sackd) {
    DEBUG_ECHO("Encoding frame control (SACK)...")
    // Encode frame control
    vector_int fc = create_sack_frame_control(sackd);
    DEBUG_VECTOR(fc);
    vector_symbol fc_symbols = create_frame_control_symbol(fc);

    DEBUG_ECHO("Creating final data stream...")
    // Create the final symbols array
    vector_symbol symbols;
    symbols.reserve(fc_symbols.size());
    symbols.push_back(PREAMBLE);
    symbols.insert(symbols.end(), fc_symbols.begin(), fc_symbols.end());

    // Concatenate the symbols into a continuous stream
    vector_float datastream = symbols_to_datastream(symbols, IEEE1901_RIFS_DEFAULT * SAMPLE_RATE);
    DEBUG_VECTOR(datastream);
    DEBUG_ECHO("Done.")
    return datastream;
}

vector_float phy_service::create_sound_ppdu(RoboMode robo_mode) {
    DEBUG_ECHO("Encoding payload blocks (Sound)...")
    // Encode payload blocks
    assert (robo_mode == MINI_ROBO || robo_mode == STD_ROBO); // Only robo mode is supported in sound mpdu
    vector_symbol payload_symbols;
    PbSize pb_size = PB520;
    if (robo_mode == MINI_ROBO) {
        payload_symbols = SOUND_PB136;
        pb_size = PB136;
    } else if (robo_mode == STD_ROBO) {
        payload_symbols = SOUND_PB520;
        pb_size = PB520;
    }

    DEBUG_ECHO("Encoding frame control (Sound) ...")
    // Encode frame control
    vector_int mpdu_fc = create_sound_frame_control(payload_symbols.size(), pb_size);
    DEBUG_VECTOR(mpdu_fc);
    vector_symbol fc_symbols = create_frame_control_symbol(mpdu_fc);

    DEBUG_ECHO("Creating final data stream...")
    // Create the final symbols array
    vector_symbol symbols;
    symbols.reserve(fc_symbols.size() + payload_symbols.size() + 1);
    symbols.push_back(PREAMBLE);
    symbols.insert(symbols.end(), fc_symbols.begin(), fc_symbols.end());
    symbols.insert(symbols.end(), payload_symbols.begin(), payload_symbols.end());

    // Concatenate the symbols into a continuous stream
    vector_float datastream = symbols_to_datastream(symbols, IEEE1901_RIFS_DEFAULT * SAMPLE_RATE);
    DEBUG_VECTOR(datastream);
    DEBUG_ECHO("Done.")
    return datastream;
}    

vector_int phy_service::create_sof_frame_control (unsigned int n_symbols, RoboMode robo_mode, modulation_type modulation, PbSize pb_size)  {
    vector_int frame_control(FRAME_CONTROL_NBITS,0);

    // Set the pbsz bit
    if (pb_size == PB520)
        set_field(frame_control, IEEE1901_FRAME_CONTROL_SOF_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOF_PBSZ_WIDTH, 0);
    else
        set_field(frame_control, IEEE1901_FRAME_CONTROL_SOF_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOF_PBSZ_WIDTH, 1);

    // Set delimiter type to SOF    
    set_field(frame_control, IEEE1901_FRAME_CONTROL_DT_IH_OFFSET, IEEE1901_FRAME_CONTROL_DT_IH_WIDTH, 1);
    
    set_field(frame_control, IEEE1901_FRAME_CONTROL_SOF_CFS_OFFSET, IEEE1901_FRAME_CONTROL_SOF_CFS_WIDTH, 1);

    if (n_symbols<=2)
        set_field(frame_control, IEEE1901_FRAME_CONTROL_SOF_NUMSYM_OFFSET, IEEE1901_FRAME_CONTROL_SOF_NUMSYM_WIDTH, n_symbols);
    else 
        set_field(frame_control, IEEE1901_FRAME_CONTROL_SOF_NUMSYM_OFFSET, IEEE1901_FRAME_CONTROL_SOF_NUMSYM_WIDTH, 3);
    
    // Set tone map index
    int tmi = 0;
    switch (robo_mode) {
        case STD_ROBO: tmi=0; break;
        case HS_ROBO: tmi=1; break;
        case MINI_ROBO: tmi=2; break;
        case NO_ROBO: 
            switch (modulation) {
                case BPSK: tmi = 3; break;
                case QPSK: tmi = 4; break;
                case QAM8: tmi = 5; break;
                case QAM16: tmi = 6; break;
                case QAM64: tmi = 7; break;
                case QAM256: tmi = 8; break;
                case QAM1024: tmi = 9; break;
                case QAM4096: tmi = 10; break;
            }
            break;
        default: tmi=0; break;
    }
    set_field(frame_control, IEEE1901_FRAME_CONTROL_SOF_TMI_OFFSET, IEEE1901_FRAME_CONTROL_SOF_TMI_WIDTH, tmi);
    
    // Calculate and set frame width
    int fl_width = std::ceil((n_symbols * (NUMBER_OF_CARRIERS*2.0/SAMPLE_RATE) + IEEE1901_RIFS_DEFAULT)/1.28);
    set_field(frame_control, IEEE1901_FRAME_CONTROL_SOF_FL_OFFSET, IEEE1901_FRAME_CONTROL_SOF_FL_WIDTH, fl_width);

    // Calculate and set CRC24
    set_field(frame_control, IEEE1901_FRAME_CONTROL_FCCS_OFFSET, IEEE1901_FRAME_CONTROL_FCCS_WIDTH, crc24(vector_int(frame_control.begin(),frame_control.end()-24))); 

    return frame_control;
}

vector_int phy_service::create_sack_frame_control (const vector_int sackd)  {
    vector_int frame_control(FRAME_CONTROL_NBITS,0);

    // Set delimiter type to SACK
    set_field(frame_control, IEEE1901_FRAME_CONTROL_DT_IH_OFFSET, IEEE1901_FRAME_CONTROL_DT_IH_WIDTH, 2);

    // Contention free session bit
    set_field(frame_control, IEEE1901_FRAME_CONTROL_SACK_CFS_OFFSET, IEEE1901_FRAME_CONTROL_SACK_CFS_WIDTH, 1);

    // SACK version number
    set_field(frame_control, IEEE1901_FRAME_CONTROL_SACK_SVN_OFFSET, IEEE1901_FRAME_CONTROL_SACK_SVN_WIDTH, 0);

    // Assign the SACK data bits
    int j = 0;
    for (vector_int::const_iterator iter = sackd.begin(); iter != sackd.end(); iter++)
        frame_control[IEEE1901_FRAME_CONTROL_SACK_SACKD_OFFSET + j++] = *iter;

    // Calculate and set CRC24
    set_field(frame_control, IEEE1901_FRAME_CONTROL_FCCS_OFFSET, IEEE1901_FRAME_CONTROL_FCCS_WIDTH, crc24(vector_int(frame_control.begin(),frame_control.end()-24))); 

    return frame_control;
}


vector_int phy_service::create_sound_frame_control (unsigned int n_symbols, PbSize pb_size)  {
    vector_int frame_control(FRAME_CONTROL_NBITS,0);

    // Set the pbsz bit
    if (pb_size == PB520)
        set_field(frame_control, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_WIDTH, 0);
    else
        set_field(frame_control, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_WIDTH, 1);

    // Set delimiter type to Sound    
    set_field(frame_control, IEEE1901_FRAME_CONTROL_DT_IH_OFFSET, IEEE1901_FRAME_CONTROL_DT_IH_WIDTH, 4);
    
    set_field(frame_control, IEEE1901_FRAME_CONTROL_SOUND_CFS_OFFSET, IEEE1901_FRAME_CONTROL_SOUND_CFS_WIDTH, 1);

    // Calculate and set frame width
    int fl_width = std::ceil((n_symbols * (NUMBER_OF_CARRIERS*2.0/SAMPLE_RATE) + IEEE1901_RIFS_DEFAULT)/1.28);
    set_field(frame_control, IEEE1901_FRAME_CONTROL_SOUND_FL_OFFSET, IEEE1901_FRAME_CONTROL_SOUND_FL_WIDTH, fl_width);

    // Calculate and set CRC24
    set_field(frame_control, IEEE1901_FRAME_CONTROL_FCCS_OFFSET, IEEE1901_FRAME_CONTROL_FCCS_WIDTH, crc24(vector_int(frame_control.begin(),frame_control.end()-24))); 

    return frame_control;
}

void phy_service::set_field(vector_int &bit_vector, int bit_offset, int bit_width, unsigned long new_value) {
    for (int i=0; i<bit_width; i++) 
        bit_vector[bit_offset + bit_width - i - 1] = (new_value & (1<<i)) >> i;
    return;
}

void phy_service::pack_bitvector(vector_int::const_iterator iter, vector_int::const_iterator end, unsigned char* array) {
    assert ((end-iter) % 8 == 0);
    int i = 0;
    while (iter!=end) {
        unsigned char byte = 0;
        for (int offset=7; offset>=0; offset --)
            byte |= *iter++ << offset;
        array[i++] = byte;
    }
}

vector_int phy_service::unpack_into_bitvector (const unsigned char *data, size_t c) {
    vector_int bit_vector(c*8, 0);
    for (unsigned int i=0; i<c; i++)
    {
        unsigned char byte = data[i];
        int offset = 7;
        while (byte)
        {
            bit_vector[i*8+offset--] = byte & 1;
            byte >>= 1;
        }
    }
    return bit_vector;
}

unsigned long phy_service::crc32(unsigned char *icp, int icnt) {
    //C704DD7B

    //crc32 table for polynom=04C11DB7
    static unsigned long crc32tab[256] = {
        0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9,
        0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005,
        0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
        0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd,
        0x4c11db70, 0x48d0c6c7, 0x4593e01e, 0x4152fda9,
        0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
        0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011,
        0x791d4014, 0x7ddc5da3, 0x709f7b7a, 0x745e66cd,
        0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,
        0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5,
        0xbe2b5b58, 0xbaea46ef, 0xb7a96036, 0xb3687d81,
        0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
        0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49,
        0xc7361b4c, 0xc3f706fb, 0xceb42022, 0xca753d95,
        0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1,
        0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d,
        0x34867077, 0x30476dc0, 0x3d044b19, 0x39c556ae,
        0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
        0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16,
        0x018aeb13, 0x054bf6a4, 0x0808d07d, 0x0cc9cdca,
        0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde,
        0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02,
        0x5e9f46bf, 0x5a5e5b08, 0x571d7dd1, 0x53dc6066,
        0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
        0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e,
        0xbfa1b04b, 0xbb60adfc, 0xb6238b25, 0xb2e29692,
        0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6,
        0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a,
        0xe0b41de7, 0xe4750050, 0xe9362689, 0xedf73b3e,
        0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
        0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686,
        0xd5b88683, 0xd1799b34, 0xdc3abded, 0xd8fba05a,
        0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637,
        0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb,
        0x4f040d56, 0x4bc510e1, 0x46863638, 0x42472b8f,
        0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
        0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47,
        0x36194d42, 0x32d850f5, 0x3f9b762c, 0x3b5a6b9b,
        0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff,
        0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623,
        0xf12f560e, 0xf5ee4bb9, 0xf8ad6d60, 0xfc6c70d7,
        0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
        0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f,
        0xc423cd6a, 0xc0e2d0dd, 0xcda1f604, 0xc960ebb3,
        0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7,
        0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b,
        0x9b3660c6, 0x9ff77d71, 0x92b45ba8, 0x9675461f,
        0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
        0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640,
        0x4e8ee645, 0x4a4ffbf2, 0x470cdd2b, 0x43cdc09c,
        0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8,
        0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24,
        0x119b4be9, 0x155a565e, 0x18197087, 0x1cd86d30,
        0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
        0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088,
        0x2497d08d, 0x2056cd3a, 0x2d15ebe3, 0x29d4f654,
        0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0,
        0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c,
        0xe3a1cbc1, 0xe760d676, 0xea23f0af, 0xeee2ed18,
        0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
        0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0,
        0x9abc8bd5, 0x9e7d9662, 0x933eb0bb, 0x97ffad0c,
        0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668,
        0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4,
    };

    unsigned long crc = -1; // init value to all ones
    unsigned char *cp = icp;
    int cnt = icnt;

    while(cnt--)
        crc=((crc<<8)&0xffffff00)^crc32tab[((crc>>24)&0xff)^*cp++];
    
    return(crc ^ 0xffffffff);
}    

unsigned long phy_service::crc24(const vector_int &bit_vector) {
    //800FE3

    //crc24 table for polynom=800063
    static unsigned long crc24tab[256] = {
        0x000000, 0x800063, 0x8000a5, 0x0000c6, 0x800129, 0x00014a, 0x00018c, 0x8001ef,
        0x800231, 0x000252, 0x000294, 0x8002f7, 0x000318, 0x80037b, 0x8003bd, 0x0003de,
        0x800401, 0x000462, 0x0004a4, 0x8004c7, 0x000528, 0x80054b, 0x80058d, 0x0005ee,
        0x000630, 0x800653, 0x800695, 0x0006f6, 0x800719, 0x00077a, 0x0007bc, 0x8007df,
        0x800861, 0x000802, 0x0008c4, 0x8008a7, 0x000948, 0x80092b, 0x8009ed, 0x00098e,
        0x000a50, 0x800a33, 0x800af5, 0x000a96, 0x800b79, 0x000b1a, 0x000bdc, 0x800bbf,
        0x000c60, 0x800c03, 0x800cc5, 0x000ca6, 0x800d49, 0x000d2a, 0x000dec, 0x800d8f,
        0x800e51, 0x000e32, 0x000ef4, 0x800e97, 0x000f78, 0x800f1b, 0x800fdd, 0x000fbe,
        0x8010a1, 0x0010c2, 0x001004, 0x801067, 0x001188, 0x8011eb, 0x80112d, 0x00114e,
        0x001290, 0x8012f3, 0x801235, 0x001256, 0x8013b9, 0x0013da, 0x00131c, 0x80137f,
        0x0014a0, 0x8014c3, 0x801405, 0x001466, 0x801589, 0x0015ea, 0x00152c, 0x80154f,
        0x801691, 0x0016f2, 0x001634, 0x801657, 0x0017b8, 0x8017db, 0x80171d, 0x00177e,
        0x0018c0, 0x8018a3, 0x801865, 0x001806, 0x8019e9, 0x00198a, 0x00194c, 0x80192f,
        0x801af1, 0x001a92, 0x001a54, 0x801a37, 0x001bd8, 0x801bbb, 0x801b7d, 0x001b1e,
        0x801cc1, 0x001ca2, 0x001c64, 0x801c07, 0x001de8, 0x801d8b, 0x801d4d, 0x001d2e,
        0x001ef0, 0x801e93, 0x801e55, 0x001e36, 0x801fd9, 0x001fba, 0x001f7c, 0x801f1f,
        0x802121, 0x002142, 0x002184, 0x8021e7, 0x002008, 0x80206b, 0x8020ad, 0x0020ce,
        0x002310, 0x802373, 0x8023b5, 0x0023d6, 0x802239, 0x00225a, 0x00229c, 0x8022ff,
        0x002520, 0x802543, 0x802585, 0x0025e6, 0x802409, 0x00246a, 0x0024ac, 0x8024cf,
        0x802711, 0x002772, 0x0027b4, 0x8027d7, 0x002638, 0x80265b, 0x80269d, 0x0026fe,
        0x002940, 0x802923, 0x8029e5, 0x002986, 0x802869, 0x00280a, 0x0028cc, 0x8028af,
        0x802b71, 0x002b12, 0x002bd4, 0x802bb7, 0x002a58, 0x802a3b, 0x802afd, 0x002a9e,
        0x802d41, 0x002d22, 0x002de4, 0x802d87, 0x002c68, 0x802c0b, 0x802ccd, 0x002cae,
        0x002f70, 0x802f13, 0x802fd5, 0x002fb6, 0x802e59, 0x002e3a, 0x002efc, 0x802e9f,
        0x003180, 0x8031e3, 0x803125, 0x003146, 0x8030a9, 0x0030ca, 0x00300c, 0x80306f,
        0x8033b1, 0x0033d2, 0x003314, 0x803377, 0x003298, 0x8032fb, 0x80323d, 0x00325e,
        0x803581, 0x0035e2, 0x003524, 0x803547, 0x0034a8, 0x8034cb, 0x80340d, 0x00346e,
        0x0037b0, 0x8037d3, 0x803715, 0x003776, 0x803699, 0x0036fa, 0x00363c, 0x80365f,
        0x8039e1, 0x003982, 0x003944, 0x803927, 0x0038c8, 0x8038ab, 0x80386d, 0x00380e,
        0x003bd0, 0x803bb3, 0x803b75, 0x003b16, 0x803af9, 0x003a9a, 0x003a5c, 0x803a3f,
        0x003de0, 0x803d83, 0x803d45, 0x003d26, 0x803cc9, 0x003caa, 0x003c6c, 0x803c0f,
        0x803fd1, 0x003fb2, 0x003f74, 0x803f17, 0x003ef8, 0x803e9b, 0x803e5d, 0x003e3e
    };

    unsigned long crc = -1; // init value to all ones
    vector_int::const_iterator iter = bit_vector.begin();
    assert (bit_vector.size() % 8 == 0);

    while(iter != bit_vector.end()) {
        unsigned char cp;
        cp = 0;
        for (int offset=7; offset>=0; offset--) 
            cp |= (*iter++) << offset;
        crc = ((crc << 8) & 0xffff00) ^ crc24tab[((crc >> 16) & 0xff) ^ cp];
    }

    return (crc ^ 0xffffff);
}    

vector_symbol_freq phy_service::create_payload_symbols_freq(const vector_int &payload_bits, PbSize pb_size, RoboMode robo_mode, code_rate rate, modulation_type modulation) {
    // Determine number of blocks and blocks size
    int block_n_bits = (pb_size == PB520) ? 520*8 : 136*8;
    assert (pb_size == PB520 || pb_size == PB136); // Cannot have payload blocks of 16 octets

    int n_blocks = payload_bits.size() / block_n_bits;
    assert ((payload_bits.size() % block_n_bits) == 0);

    int block_size = calc_interleaved_block_size(robo_mode, rate, pb_size);
    // Encode PLCP blocks
    phy_service::Carriers carriers;
    if (robo_mode == NO_ROBO)
        carriers = build_broadcast_carriers(modulation);
    else {
        carriers = calc_robo_carriers(robo_mode);
        rate = RATE_1_2;
    }

    vector_int block_bits, encoded_payload_bits(block_size * n_blocks);
    vector_int::iterator iter_out = encoded_payload_bits.begin();
    int scrambler_state = scrambler_init(); // This inits the scrambler state
    vector_int::const_iterator iter_in = payload_bits.begin();
    while (iter_in != payload_bits.end()) {
        block_bits = vector_int(iter_in, iter_in + block_n_bits);
        // Scrambler
        vector_int scrambled = scrambler(block_bits, scrambler_state);
        DEBUG_VECTOR(scrambled);

        // Turbo-convolution encoder
        vector_int parity = tc_encoder(scrambled, pb_size, rate);
        DEBUG_VECTOR(parity);

        // Channel interleaver
        vector_int interleaved = channel_interleaver(scrambled, parity, pb_size, rate);
        DEBUG_VECTOR(interleaved);

        if (robo_mode != NO_ROBO) {
            interleaved = robo_interleaver(interleaved, robo_mode);
            DEBUG_VECTOR(interleaved);
        }

        iter_out = std::copy(interleaved.begin(), interleaved.end(), iter_out);
        iter_in += block_n_bits;
    }

    // Mapping and split to symbols
    return modulate(encoded_payload_bits, carriers);
}

vector_symbol phy_service::create_payload_symbols(const vector_int &payload_bits, PbSize pb_size, RoboMode robo_mode, code_rate rate, modulation_type modulation) {
    vector_symbol_freq symbols_freq = create_payload_symbols_freq(payload_bits, pb_size, robo_mode, rate, modulation);

    // Perform IFFT to get a time domain symbol
    vector_symbol symbols(symbols_freq.size());
    for (unsigned int i = 0; i < symbols_freq.size(); i++) {
        symbols[i] = ifft_real(symbols_freq[i].begin(), symbols_freq[i].end());
    }

    // Add cyclic prefix and windowing of each symbol.
    // Dividing by N to compensate for the above IFFT which implicitly multiply by N
    cyclic_prefix_and_window(symbols, GUARD_INTERVAL_PAYLOAD, SCALE_FACTOR_PAYLOAD / ((float)NUMBER_OF_CARRIERS*2));

    return symbols;
}

vector_symbol phy_service::create_frame_control_symbol(const vector_int &bitstream) {
    // Turbo-convolution encoder
    vector_int parity = tc_encoder(bitstream, PB16, RATE_1_2);
    DEBUG_VECTOR(parity);

    // Channel interleaver
    vector_int interleaved = channel_interleaver(bitstream, parity, PB16, RATE_1_2);
    DEBUG_VECTOR(interleaved);

    vector_int copied = copier(interleaved, N_BROADCAST_CARRIERS, 128 + 12/2); // +12/2 because of non-standard turbo encoder...
    DEBUG_VECTOR(copied);

    // Mapping and split to symbols
    vector_symbol_freq fc_symbols_freq = modulate(copied, BROADCAST_CARRIERS);

    // Perform IFFT to get a time domain symbol
    vector_symbol symbols(fc_symbols_freq.size());
    for (unsigned int i = 0; i < fc_symbols_freq.size(); i++) {
        symbols[i] = ifft_real(fc_symbols_freq[i].begin(), fc_symbols_freq[i].end());
        DEBUG_VECTOR(symbols[i]);
    }

    // Add cyclic prefix and windowing of each symbol
    // Dividing by N to compensate for the above IFFT which implicitly multiply by N
    cyclic_prefix_and_window(symbols, GUARD_INTERVAL_FC, SCALE_FACTOR_FC / ((float)NUMBER_OF_CARRIERS*2));

    return symbols;
}

vector_int phy_service::scrambler(const vector_int& bitstream, int &state) {
    int feedback;
    vector_int out(bitstream.size());
    for (unsigned int i = 0; i < bitstream.size(); i++) {
        feedback = (!!(state & 0x200)) ^ (!!(state & 0x4)); // feedback = state[2] XOR state[9]
        out[i] = feedback ^ bitstream[i];
        state = ((state << 1) & 0x3FF) | feedback;
    }
    return out;
}

int phy_service::scrambler_init(void) {
    return 0x3FF;
}

void phy_service::init_turbo_codec() {
    itpp::ivec gen(2);
    gen(0) = 013; gen(1) = 015;
    itpp::bmat puncture_matrix = "1;1;1";
    itpp::ivec interleaver_sequence_bvec;
    d_turbo_codec.set_parameters(gen, gen, 4, interleaver_sequence_bvec, puncture_matrix, 4, "LOGMAX", 1.0, true,  itpp::LLR_calc_unit());
}

vector_int phy_service::tc_encoder(const vector_int &bitstream, PbSize pb_size, code_rate rate) {
    itpp::bmat puncture_matrix;
    assert (rate == RATE_1_2); // Only Rate = 1/2 is supported in the encoder/decoder
    if (rate == RATE_1_2)
        puncture_matrix = "1 1;1 0;0 1";
    
    itpp::ivec interleaver_sequence_bvec = to_ivec(TURBO_INTERLEAVER_SEQUENCE[pb_size]);
    d_turbo_codec.set_interleaver(interleaver_sequence_bvec);
    d_turbo_codec.set_puncture_matrix(puncture_matrix);
    itpp::bvec encoded_bvec;

    d_turbo_codec.encode(to_bvec(bitstream),encoded_bvec);
    vector_int encoded = to_vector_int(encoded_bvec);
    DEBUG_VECTOR(encoded);

    vector_int parity(((encoded.size()-bitstream.size() + 3) / 4) * 4);  // The parity should be divisible by 4
    unsigned int i = 0;
    for (;i < bitstream.size(); i++) {
        parity[i] = encoded[i*2+1]; // first bit is systematic, second is parity
    }
    for (unsigned int j = i*2; j<encoded.size(); j++) {
        parity[i] = encoded[j];
        i++;
    }

    return parity;

    /* Standard encoder: 
    vector_int p,q;
    p = consistuent_encoder(bitstream, pb_size);
    DEBUG_VECTOR(p);

    q = turbo_interleaver(bitstream, pb_size);
    q = consistuent_encoder(q, pb_size);
    DEBUG_VECTOR(q);

    puncture(p, rate);
    DEBUG_VECTOR(p);

    puncture(q, rate);
    DEBUG_VECTOR(q);

    vector_int parity(p.size()*2);
    for (unsigned int i = 0; i<p.size(); i++) {
        parity[2*i] = p[i];
        parity[2*i+1] = q[i];
    }
    return parity; */
}

vector_int phy_service::tc_decoder(const vector_float &received_info, const vector_float &received_parity, PbSize pb_size, code_rate rate) {
    itpp::bmat puncture_matrix;
    assert (rate == RATE_1_2); // Only Rate = 1/2 is supported in the encoder/decoder
    if (rate == RATE_1_2)
        puncture_matrix = "1 1;1 0;0 1";

    DEBUG_VECTOR (received_info);
    DEBUG_VECTOR (received_parity);

    itpp::ivec interleaver_sequence_bvec = to_ivec(TURBO_INTERLEAVER_SEQUENCE[pb_size]);
    d_turbo_codec.set_interleaver(interleaver_sequence_bvec);
    d_turbo_codec.set_puncture_matrix(puncture_matrix);

    itpp::vec decoder_input(d_turbo_codec.get_punctured_size());
    itpp::bvec decoded_bvec;

    unsigned int i = 0;
    for (;i < received_info.size(); i++) {
        decoder_input(i*2) = received_info[i]; // systematic bit is first
        decoder_input(i*2+1) = received_parity[i]; // parity bit is second
    }
    for (int j = i*2; j < d_turbo_codec.get_punctured_size(); j++) {
        decoder_input(j) = received_parity[i++]; // put the rest of the parity bits (tail)
    }

    d_turbo_codec.decode(decoder_input, decoded_bvec);
    vector_int decoded = to_vector_int(decoded_bvec);
    DEBUG_VECTOR(decoded);

    return decoded;
}


itpp::bvec phy_service::to_bvec (const vector_int in) {
    itpp::bvec out(in.size());
    for (unsigned int i = 0; i< in.size(); i++)
        out(i) = in[i];
    return out;
}

itpp::ivec phy_service::to_ivec (const vector_int in) {
    itpp::ivec out(in.size());
    for (unsigned int i = 0; i< in.size(); i++)
        out(i) = in[i];
    return out;
}

vector_int phy_service::to_vector_int (const itpp::bvec in) {
    vector_int out(in.size());
    for (unsigned int i = 0; i< out.size(); i++)
        out[i] = in(i);
    return out;
}

vector_int phy_service::consistuent_encoder(const vector_int& in, PbSize pb_size) {
    int s1 = 0;
    int s2 = 0;
    int s3 = 0;
    int u1, u2;
    int x0;
    int s1_new, s2_new, s3_new;
    vector_int out(in.size()/2);

    // First pass
    for(unsigned int i = 0; i < in.size(); i+=2) {
        u1 = in[i];
        u2 = in[i+1];
        x0 = u1 ^ u2 ^ s3; 
        s3 = s2 ^ u2 ^ x0; // s3 = s2 ^ u1  ^ s3
        s2 = s1 ^ u1 ^ u2; 
        s1 = u1 ^ u2 ^ x0; // s1 = s3
    }

    if (pb_size != PB136) {
        s1_new = s2 ^ s3;
        s2_new = s3;
        s3_new = s1 ^ s2 ^ s3;
    } else {
        s1_new = s2;
        s2_new = s1 ^ s3;
        s3_new = s1;
    }

    s1 = s1_new;
    s2 = s2_new;
    s3 = s3_new;

    // Second pass
    for(unsigned int i = 0; i < in.size(); i+=2) {
        u1 = in[i];
        u2 = in[i+1];
        x0 = u1 ^ u2 ^ s3; 
        s3 = s2 ^ u2 ^ x0; // s3 = s2 ^ u1 ^ s3
        s2 = s1 ^ u1 ^ u2; 
        s1 = u1 ^ u2 ^ x0; // s1 = s3
        out[i/2] = x0;
    }
    return out;
}

void phy_service::puncture(vector_int &bitstream, code_rate rate) {
    switch (rate) {
        case RATE_1_2:
            break;
        case RATE_16_21: {
            vector_int new_bitstream(bitstream.size() / 16 * 5);
            vector_int::iterator iter = new_bitstream.begin();
            for (unsigned int i = 0 ; i<bitstream.size(); i++) 
                if (((i % 16) % 3 == 0) && ((i % 16) != 15)) {
                    *iter = bitstream[i];
                    iter++;
                }
            bitstream = new_bitstream;
            break;
        }
        case RATE_16_18: {
            vector_int new_bitstream(bitstream.size() / 16 * 2);
            vector_int::iterator iter = new_bitstream.begin();
            for (unsigned int i = 0 ; i<bitstream.size(); i++) 
                if ((i % 8) == 0) {
                    *iter = bitstream[i];
                    iter++;
                }
            bitstream = new_bitstream;
            break;
        }
    }
    return;
}

std::array<vector_int, 3>  phy_service::calc_turbo_interleaver_sequence(){
    std::array<vector_int, 3> turbo_interleaver_sequence;
    for (int i=0; i<3; i++) {
        int N, L, I, x;
        const int *S;
        if (i == PB16) {
            N = 8;
            L = 64;
            S = TCENCODER_SEED_PB16;
        } else if (i == PB136) {
            N = 34;
            L = 544;
            S = TCENCODER_SEED_PB136;
        } else if (i == PB520) {
            N = 40;
            L = 2080;
            S = TCENCODER_SEED_PB520;
        }
        vector_int out (L*2);
        for (x=0; x<L; x++)    {
            I = (S[x % N] -(x / N) * N + L) % L;
            if (x % 2 == 0) {
                out[2*x] = 2*I+1;
                out[2*x+1] = 2*I;
            } else {
                out[2*x] = 2*I;
                out[2*x+1] = 2*I+1;
            }
        }
        turbo_interleaver_sequence[i] = out;
    }
    return turbo_interleaver_sequence;

}


vector_int phy_service::turbo_interleaver(const vector_int &bitstream, PbSize pb_size){

    int N, L, I, x;
    const int *S;
    if (pb_size == PB16) {
        N = 8;
        L = 64;
        S = TCENCODER_SEED_PB16;
    } else if (pb_size == PB136) {
        N = 34;
        L = 544;
        S = TCENCODER_SEED_PB136;
    } else {
        N = 40;
        L = 2080;
        S = TCENCODER_SEED_PB520;
    }
    vector_int out (L*2);
    for (x=0; x<L; x++)    {
        I = (S[x % N] -(x / N) * N + L) % L;
        if (x % 2 == 0) {
            out[2*x]= bitstream[2*I+1];
            out[2*x+1] = bitstream[2*I];
        } else {
            out[2*x] = bitstream[2*I];
            out[2*x+1] = bitstream[2*I+1];
        }
    }
    return out;
}

vector_int phy_service::channel_interleaver(const vector_int& bitstream, const vector_int& parity_bitstream, PbSize pb_size, code_rate rate) {
    int step_size = CHANNEL_INTERLEAVER_STEPSIZE[pb_size][rate];
    int offset = CHANNEL_INTERLEAVER_OFFSET[pb_size][rate];
    int info_row_no = 0;
    int n_info_rows_done = 0;
    int n_parity_rows_done = 0;
    int parity_row_no = offset;
    bool parity_done = false, info_done = false;
    int nibble_no = 0;
    vector_int out(bitstream.size() + parity_bitstream.size());
    vector_int::iterator iter = out.begin();

    switch (rate) {
    case RATE_1_2:
        while (!info_done || !parity_done) {
            info_done = channel_interleaver_row(bitstream, iter, step_size, info_row_no, n_info_rows_done, nibble_no);
            parity_done = channel_interleaver_row(parity_bitstream, iter, step_size, parity_row_no, n_parity_rows_done, nibble_no);
        }
        break;

    case RATE_16_21:
        while (!info_done && !parity_done) {
            for (int i=0; i<5; i++) {
                info_done = channel_interleaver_row(bitstream, iter, step_size, info_row_no, n_info_rows_done, nibble_no);
                info_done = channel_interleaver_row(bitstream, iter, step_size, info_row_no, n_info_rows_done, nibble_no);
                info_done = channel_interleaver_row(bitstream, iter, step_size, info_row_no, n_info_rows_done, nibble_no);
                parity_done = channel_interleaver_row(parity_bitstream, iter, step_size, parity_row_no, n_parity_rows_done, nibble_no, true);
            }
            info_done = channel_interleaver_row(bitstream, iter, step_size, info_row_no, n_info_rows_done, nibble_no);
        }
        break;

    case RATE_16_18:
        while (!info_done && !parity_done) {
            info_done = channel_interleaver_row(bitstream, iter, step_size, info_row_no, n_info_rows_done, nibble_no);
            info_done = channel_interleaver_row(bitstream, iter, step_size, info_row_no, n_info_rows_done, nibble_no);
            info_done = channel_interleaver_row(bitstream, iter, step_size, info_row_no, n_info_rows_done, nibble_no);
            parity_done = channel_interleaver_row(parity_bitstream, iter, step_size, parity_row_no, n_parity_rows_done, nibble_no, true);
            info_done = channel_interleaver_row(bitstream, iter, step_size, info_row_no, n_info_rows_done, nibble_no);
            info_done = channel_interleaver_row(bitstream, iter, step_size, info_row_no, n_info_rows_done, nibble_no);
            info_done = channel_interleaver_row(bitstream, iter, step_size, info_row_no, n_info_rows_done, nibble_no);
            info_done = channel_interleaver_row(bitstream, iter, step_size, info_row_no, n_info_rows_done, nibble_no);
            info_done = channel_interleaver_row(bitstream, iter, step_size, info_row_no, n_info_rows_done, nibble_no);
        }
        break;
    }
    return out;
}

bool phy_service::channel_interleaver_row(const vector_int& bitstream, vector_int::iterator &iter, int step_size, int& row_no, int& rows_done, int& nibble_no, bool wrap) {
    int n_rows = bitstream.size()/4;
    if (rows_done < n_rows) {
        // Reading a row
        for (int i = nibble_no / 2; i < (nibble_no / 2) + 4 ; i++, iter++)
            *iter = (bitstream[row_no + (i % 4) * n_rows]); // read row row_no, element i
        rows_done++;
        nibble_no = (nibble_no + 1) % 8;

        // Calculating the next row number
        if (wrap || (row_no + step_size < n_rows))
            row_no = (row_no + step_size) % n_rows;
        else
            row_no = (row_no % step_size) + 1;
    }
    if (rows_done == n_rows) 
        return true;
    return false;
}

vector_int phy_service::robo_interleaver(const vector_int& bitstream, RoboMode robo_mode) {
    // Determine number of bits to pad at end of copy
    unsigned int n_raw = bitstream.size();
    unsigned int n_copies, bits_in_last_symbol, bits_in_segment, n_pad;
    calc_robo_parameters (robo_mode, n_raw, n_copies, bits_in_last_symbol, bits_in_segment, n_pad);

    // Set the bits shift parameters
    std::vector<int> cycle_shifts(n_copies,0);
    int l = (bits_in_last_symbol - 1) / bits_in_segment;
    switch (n_copies) {
        case 2:  // l \in {0,1}
            if (l==1) 
                cycle_shifts = {0,1};
            break;
        case 4:  // l \in {0,1,2,3}
            if (l==1)
                cycle_shifts = {0,0,1,1};
            else if (l==3)
                cycle_shifts = {0,1,2,3};
            break;
        case 5:  // l \in {0,1,2,3,4}
            if (l==4)
                cycle_shifts = {0,1,2,3,4};
            break;
    }

    // Perform the bits rotation and copying
    vector_int robo_bitstream((n_raw + n_pad) * n_copies);
    vector_int::iterator robo_bitstream_iter = robo_bitstream.begin();
    for (unsigned int k=0; k<n_copies; k++) {
        int start_position = (n_raw + n_pad - (cycle_shifts[k] * bits_in_segment)) % (n_raw + n_pad);
        robo_bitstream_iter = std::copy(bitstream.begin() + start_position, bitstream.end(), robo_bitstream_iter);
        robo_bitstream_iter = std::copy(bitstream.begin(), bitstream.begin() + n_pad, robo_bitstream_iter);
        robo_bitstream_iter = std::copy(bitstream.begin(), bitstream.begin() + start_position, robo_bitstream_iter);
    }
    return robo_bitstream;
}

phy_service::Carriers phy_service::calc_robo_carriers (RoboMode robo_mode) {
    // Create basic ROBO carriers consists of broadcast mask and QPSK
    phy_service::Carriers carriers = BROADCAST_CARRIERS;

    int n_copies = 0;

    // Each mode replicates the bits n_copies times
    switch (robo_mode) { 
        case STD_ROBO: n_copies = 4; break;
        case HS_ROBO: n_copies = 2; break;
        case MINI_ROBO: n_copies = 5; break;
        case NO_ROBO: break; 
    }
    assert (robo_mode == STD_ROBO || robo_mode == HS_ROBO || robo_mode == MINI_ROBO); // ROBO mode should be only one of these

    unsigned int n_carriers = N_BROADCAST_CARRIERS;
    unsigned int n_carriers_robo = n_copies * (n_carriers / n_copies);

    // Mark the unused carriers as masked
    unsigned int j = 0, i = carriers.mask.size()-1;
    while (j<(n_carriers-n_carriers_robo)) {
        if (carriers.mask[i]) {
            carriers.mask[i] = 0;
            j++;
        }
        i--;
    }
    update_carriers_capacity(carriers);

    return carriers; 
}

void phy_service::calc_robo_parameters (RoboMode robo_mode, unsigned int n_raw, unsigned int &n_copies, unsigned int &bits_in_last_symbol, unsigned int &bits_in_segment, unsigned int &n_pad) {
    // Each mode replicates the bits n_copies times
    switch (robo_mode) { 
        case STD_ROBO: n_copies = 4; break;
        case HS_ROBO: n_copies = 2; break;
        case MINI_ROBO: n_copies = 5; break;
        case NO_ROBO: break; 
    }
    assert (robo_mode == STD_ROBO || robo_mode == HS_ROBO || robo_mode == MINI_ROBO); // ROBO mode should be only one of these

    unsigned int n_carriers = N_BROADCAST_CARRIERS;
    unsigned int n_carriers_robo = n_copies * (n_carriers / n_copies);

    // Determine number of bits to pad at end of copy
    unsigned int n_carriers_in_segment = n_carriers_robo / n_copies;
    int bits_per_symbol = MODULATION_MAP[QPSK].n_bits * n_carriers_robo;
    bits_in_segment = MODULATION_MAP[QPSK].n_bits * n_carriers_in_segment;
    bits_in_last_symbol = n_raw % bits_per_symbol;
    unsigned int bits_in_last_segment = 0;
    if (bits_in_last_symbol == 0) {
        bits_in_last_symbol = bits_per_symbol;
        bits_in_last_segment = bits_in_segment;
    } else 
        bits_in_last_segment = bits_in_last_symbol - bits_in_segment * ((bits_in_last_symbol - 1) / bits_in_segment);
    n_pad = bits_in_segment - bits_in_last_segment;

    return;
}

vector_symbol_freq phy_service::modulate(const vector_int& bits, const phy_service::Carriers& carriers) {
    // Calculate number of symbols needed
    int n_symbols = (bits.size() % carriers.capacity) ? bits.size() / carriers.capacity + 1 : bits.size() / carriers.capacity;
    vector_symbol_freq symbols_freq(n_symbols);
    // Perform mapping
    vector_int::const_iterator it = bits.cbegin();
    int pn_state = pn_generator_init();
    for (int j = 0; j < n_symbols; j++) { 
        vector_complex mapped_bits = vector_complex(NUMBER_OF_CARRIERS+1);
        for (int i=0; i<=NUMBER_OF_CARRIERS; i++) {
            if (!TRANSMIT_MASK[i]) // if regulations do not allow the carrier to transmit
                continue;
            else if (carriers.mask[i]) {  // If carrier is ON
                ModulationMap modulation_map = MODULATION_MAP[carriers.modulation[i]];
                int n_bits = modulation_map.n_bits;
                int decimal = 0;
                int bit_no = 0;
                while (bit_no < n_bits) {
                    if (it != bits.cend()) { // If there are still bits to map, use them
                        decimal |= *it << bit_no;
                        bit_no++;
                        it++;
                    } else {  // When all bits are mapped, use random bits instead
                        decimal |= pn_generator(n_bits-bit_no, pn_state) << bit_no;
                        break;
                    }
                }
                // Convert the angle number to its value
                complex p = ANGLE_NUMBER_TO_VALUE[CARRIERS_ANGLE_NUMBER[i] * 2]; 
                // Calculate the mapped value. Multiplying by the scale for unit average power.
                // Multiplying by N/2 so in time domain this will produce cos() with unit amplitude
                complex m = modulation_map.map[decimal] * modulation_map.scale * (float)NUMBER_OF_CARRIERS;
                 // Rotate the mapped value using the angle number and add it to the mapped values vector
                mapped_bits[i] = complex(m.real() * p.real() - m.imag() * p.imag(), 
                                             m.real() * p.imag() + m.imag() * p.real());                
            } else {  // If carrier is OFF use random bit with BPSK modulation
                mapped_bits[i] = MODULATION_MAP[BPSK].map[pn_generator(1, pn_state)] * MODULATION_MAP[BPSK].scale * (float)NUMBER_OF_CARRIERS;
            }
        } // Repeat until entire symbol is filled
        DEBUG_VECTOR(mapped_bits);
        symbols_freq[j] = mapped_bits;

    } // Keep add symbols until all bits are mapped
   
    return symbols_freq;
}

vector_float phy_service::ifft_real(vector_complex::const_iterator iter_begin, vector_complex::const_iterator iter_end) {
    assert (iter_end - iter_begin == NUMBER_OF_CARRIERS+1);
    std::copy(iter_begin, iter_end, (complex*)(d_ifft_input));
    fftwf_execute(d_fftw_rev_plan);
    vector_float output(NUMBER_OF_CARRIERS * 2);
    memcpy (output.data(), d_ifft_output, NUMBER_OF_CARRIERS * 2 * sizeof(float));
    return output; 
}

vector_complex phy_service::fft_real(vector_float::const_iterator iter_begin, vector_float::const_iterator iter_end) {
    assert (iter_end - iter_begin == (2 * NUMBER_OF_CARRIERS ));
    std::copy(iter_begin, iter_end, d_fft_input);
    fftwf_execute(d_fftw_fwd_plan);
    vector_complex output(NUMBER_OF_CARRIERS + 1);
    memcpy (output.data(), d_fft_output, (NUMBER_OF_CARRIERS + 1) * sizeof(fftwf_complex));
    return output; 
}

void phy_service::cyclic_prefix_and_window(vector_symbol& symbols, int gi_length, float gain) {
    vector_symbol old_symbols = symbols;
    symbols = vector_symbol(old_symbols.size());
    for(unsigned int k=0; k<old_symbols.size(); k++) { //vector_symbol::iterator it = old_symbols.begin();it != old_symbols.end(); ++it) {
        vector_symbol::value_type &symbol = symbols[k];
        vector_symbol::value_type &old_symbol = old_symbols[k];
        symbol = vector_symbol::value_type(old_symbol.size() + gi_length + ROLLOFF_INTERVAL);
        vector_symbol::value_type::iterator symbol_iter = symbol.begin();
        
        // Cyclic prefix
        symbol_iter = std::copy (old_symbol.end()-(gi_length+ROLLOFF_INTERVAL), old_symbol.end(), symbol_iter);
        symbol_iter = std::copy (old_symbol.begin(), old_symbol.end(), symbol_iter);

        // Windowing
        for (unsigned int i=0;i<ROLLOFF_INTERVAL;++i)
            symbol[i] = symbol[i] * ROLLOFF_WINDOW_RISE[i] * gain;

        for (unsigned int i=ROLLOFF_INTERVAL;i<symbol.size()-ROLLOFF_INTERVAL;++i)
            symbol[i] = symbol[i] * gain;

        for (unsigned int j=0,i=symbol.size()-ROLLOFF_INTERVAL;i<symbol.size();++i, ++j)
            symbol[i] = symbol[i] * ROLLOFF_WINDOW_FALL[j] * gain;
    }
}

vector_int phy_service::copier(const vector_int& bitstream, int n_carriers, int offset, int start) {
    /* copier should replicate and interleave the 256 bits as follows:
       Original bit number order (k): 0   1   2   3   4   5   6 ... 254 255 0   1   2   3   4   5   ... 254 255 ...
                           New order: 0   128 1   129 2   130 3 ... 127 255 128 0   129 1   130 2   ... 255 128 ...
                            Location: 0   1   2   3   4   5   6 ... 254 255 256 257 258 259 260 261 ... 510 511 ... n_carriers*2
    */
    vector_int copier_output(n_carriers*2);
    int size = bitstream.size();
    for (int i = 0; i<n_carriers; i++) {
            copier_output[2*i] = bitstream[(start + i) % size];
            copier_output[2*i+1] = bitstream[(start + i + offset) % size];
    }
    return copier_output;
}

int phy_service::pn_generator(int n_bits, int &pn_state) {
    int s = pn_state;
    pn_state = ((s<<1) & 0x3FF) | (((s & 0x4) >> 2) ^ ((s & 0x200) >> 9));
    return ((s << 10) | s) & ((1<<n_bits)-1); // returns lower n_bits in [s s]
}

int phy_service::pn_generator_init(void) {
    return 0x3FF;
}

vector_float::const_iterator phy_service::preamble() {
    return PREAMBLE.begin();
}

vector_float phy_service::calc_preamble() {
    // Calculate SYNCP
    vector_complex syncp_fft(SYNCP_SIZE / 2 + 1);
    for (unsigned int i = 0; i<syncp_fft.size(); i++) {
        syncp_fft[i] = ANGLE_NUMBER_TO_VALUE[SYNCP_CARRIERS_ANGLE_NUMBER[i]] * (float)IEEE1901_SCALE_FACTOR_PREAMBLE;
    }

    // Calculate IFFT(syncp_fft)   
    vector_float syncp(SYNCP_SIZE);
    fftwf_execute(fftwf_plan_dft_c2r_1d (SYNCP_SIZE,
                                         reinterpret_cast<fftwf_complex *>(syncp_fft.data()),
                                         syncp.data(),
                                         FFTW_ESTIMATE));

    // Calculate SYNCM
    vector_float syncm = vector_float(syncp);
    std::transform(syncm.begin(), syncm.end(), syncm.begin(), std::negate<vector_float::value_type>());

    // Build preamble: [P/2 P P P P P M M M/2]
    vector_float PREABMLEnom;
    PREABMLEnom.reserve(syncp.size() * 8);
    PREABMLEnom.insert(PREABMLEnom.end(),syncp.begin()+syncp.size()/2,syncp.end());
    PREABMLEnom.insert(PREABMLEnom.end(),syncp.begin(),syncp.end());
    PREABMLEnom.insert(PREABMLEnom.end(),syncp.begin(),syncp.end());
    PREABMLEnom.insert(PREABMLEnom.end(),syncp.begin(),syncp.end());
    PREABMLEnom.insert(PREABMLEnom.end(),syncp.begin(),syncp.end());
    PREABMLEnom.insert(PREABMLEnom.end(),syncp.begin(),syncp.end());
    PREABMLEnom.insert(PREABMLEnom.end(),syncm.begin(),syncm.end());
    PREABMLEnom.insert(PREABMLEnom.end(),syncm.begin(),syncm.end());
    PREABMLEnom.insert(PREABMLEnom.end(),syncm.begin(),std::next(syncm.begin(),syncm.size()/2));

    // Perform IFFT(FFT(preamble)*mu))
    assert (PREABMLEnom.size() == NUMBER_OF_CARRIERS * 2);
    vector_complex PREABMLEnom_fft(NUMBER_OF_CARRIERS + 1);

    // Calculate FFT(preamble)   
    fftwf_execute(fftwf_plan_dft_r2c_1d (NUMBER_OF_CARRIERS * 2,
                                         PREABMLEnom.data(),
                                         reinterpret_cast<fftwf_complex *>(PREABMLEnom_fft.data()),
                                         FFTW_ESTIMATE));
    // Multiply FFT output by mu
    for (unsigned int i=0;i<PREABMLEnom_fft.size();++i)
        if (!CARRIERS_BROADCAST_MASK[i]) 
            PREABMLEnom_fft[i] = 0;
        else
            // Compensate for fftw IFFT implictly multipication by FFT size
            PREABMLEnom_fft[i] /= (float)(NUMBER_OF_CARRIERS * 2);
 
    // Calculate IFFT. 
    vector_float PREABMLEnom_masked(NUMBER_OF_CARRIERS * 2);
    fftwf_execute(fftwf_plan_dft_c2r_1d (NUMBER_OF_CARRIERS * 2,
                                         reinterpret_cast<fftwf_complex *>(PREABMLEnom_fft.data()),
                                         PREABMLEnom_masked.data(),
                                         FFTW_ESTIMATE));

    // Building the extended preamble
    vector_float preamble = vector_float(SYNCP_SIZE * 10);
    unsigned int i=0;
    for (; i<3.5*SYNCP_SIZE; ++i) // SecA (size of 3.5)
        preamble[i] = PREABMLEnom_masked[i]; 
    unsigned int k=1.5*SYNCP_SIZE;    
    for (unsigned int j=0; j<ROLLOFF_INTERVAL; ++j, ++i, ++k) // Rolloff interval (size of RI)
        preamble[i] = PREABMLEnom_masked[i] * ROLLOFF_WINDOW_FALL[j] + PREABMLEnom_masked[k] * ROLLOFF_WINDOW_RISE[j];
    for (; k<PREABMLEnom_masked.size(); ++k, ++i) // SecB (size of 6.5-RI)
        preamble[i] = PREABMLEnom_masked[k];

    DEBUG_VECTOR(preamble);
    return preamble;
}

vector_complex phy_service::calc_syncp_fft(const vector_float &preamble) {
    // Calculating the FFT of the 2nd SYNCP
    vector_float syncp(preamble.begin() + SYNCP_SIZE / 2 + SYNCP_SIZE, preamble.begin() + SYNCP_SIZE / 2 + 2 * SYNCP_SIZE);
    vector_complex syncp_fft(SYNCP_SIZE / 2 + 1);

    fftwf_execute(fftwf_plan_dft_r2c_1d (SYNCP_SIZE,
                                         syncp.data(),
                                         reinterpret_cast<fftwf_complex *>(syncp_fft.data()),
                                         FFTW_ESTIMATE));
    return syncp_fft;
}

unsigned int phy_service::count_non_masked_carriers(carriers_mask::const_iterator begin, carriers_mask::const_iterator end) {
    return std::count(begin, end, true);
}

unsigned int phy_service::count_non_masked_carriers(bool *begin, bool *end) {
    return std::count(begin, end, true); 
}

vector_float phy_service::symbols_to_datastream (const vector_symbol &symbols, int ifs) {
    // Calculate the datastream final size
    vector_symbol::const_iterator symbols_iter=symbols.begin();
    int datastream_size = ROLLOFF_INTERVAL + ifs;  
    for (vector_symbol::const_iterator symbols_iter=symbols.begin(); symbols_iter != symbols.end(); symbols_iter++) 
        datastream_size += (symbols_iter->size() - ROLLOFF_INTERVAL);
    
    // Reserve space for entire datastream
    vector_float datastream (datastream_size);
    vector_float::iterator datastream_iter = datastream.begin();
    
    // Insert the first symbol to the datastream
    datastream_iter = std::copy(symbols_iter->begin(), symbols_iter->end(), datastream_iter);
    symbols_iter++;
    while (symbols_iter != symbols.end()) {
        // datastream[end-ROLLOFF:end] += symbol[0:ROLLOFF]        
        std::transform(datastream_iter-ROLLOFF_INTERVAL, datastream_iter, symbols_iter->begin(), datastream_iter-ROLLOFF_INTERVAL, std::plus<vector_float::value_type>());        
        // Concat symbol[ROLLOFF:end] to datastream
        datastream_iter = std::copy(symbols_iter->begin()+ROLLOFF_INTERVAL, symbols_iter->end(), datastream_iter);
        // Advance to the next symbol
        symbols_iter++;
    }

    // fill with zeros the IFS (inter frame space)
    std::fill (datastream_iter, datastream.end(), 0);
    return datastream;
}

void phy_service::process_ppdu_payload(vector_float::const_iterator iter, unsigned char *mpdu_payload_bin) {
    vector_int payload_bits(process_ppdu_payload(iter));
    pack_bitvector(payload_bits.begin(), payload_bits.end(), mpdu_payload_bin);
    return;
}

vector_int phy_service::process_ppdu_payload(vector_float::const_iterator iter) {
    // Resolve frame control symbol
    iter += GUARD_INTERVAL_PAYLOAD;
    
    // Resolve payload symbols    
    vector_symbol symbols(d_frame_parameters.n_expected_symbols);
    for (unsigned int i = 0; i < d_frame_parameters.n_expected_symbols; i++) {
        vector_float payload_symbol(NUMBER_OF_CARRIERS*2);
        vector_float::iterator payload_symbol_iter = payload_symbol.begin();
        payload_symbol_iter = std::copy(iter, iter + (NUMBER_OF_CARRIERS * 2 - ROLLOFF_INTERVAL), payload_symbol_iter);
        payload_symbol_iter = std::copy(iter - ROLLOFF_INTERVAL, iter, payload_symbol_iter);

        symbols[i] = payload_symbol;
        iter += NUMBER_OF_CARRIERS * 2 + GUARD_INTERVAL_PAYLOAD;
    }

    switch (d_frame_parameters.type) {
        case (MPDU_TYPE_SOF): {
            // Determine the decoded blocks size
            int pb_n_bits = (d_frame_parameters.pb_size == PB520) ? 520*8 : 136*8;
            assert (d_frame_parameters.pb_size == PB520 || d_frame_parameters.pb_size == PB136); // Cannot have payload blocks of 16 octets

            vector_float blocks_bits(d_frame_parameters.carriers.capacity * symbols.size());
            vector_float::iterator blocks_bits_iter = blocks_bits.begin();
            for (vector_symbol::iterator symbol_iter = symbols.begin(); symbol_iter != symbols.end(); symbol_iter++) {
                DEBUG_VECTOR((*symbol_iter));
                // Demodulate 
                vector_float new_bits = symbol_demodulate(symbol_iter->begin(), symbol_iter->end(), d_frame_parameters.carriers, d_broadcast_channel_response);
                blocks_bits_iter = std::copy(new_bits.begin(), new_bits.end(), blocks_bits_iter);
            }
            DEBUG_VECTOR(blocks_bits);

            // Determine the number of blocks contained
            int n_blocks =  blocks_bits.size() / d_frame_parameters.interleaved_block_size;

            // Deinterleave and decode blocks
            blocks_bits_iter = blocks_bits.begin();
            int scrambler_state = scrambler_init(); // Init the scrambler state
            vector_int payload_bits(n_blocks * pb_n_bits);
            vector_int::iterator payload_bits_iter = payload_bits.begin();
            for (int i = 0; i< n_blocks; i++) {
                vector_float block_bits = vector_float(blocks_bits_iter, blocks_bits_iter + d_frame_parameters.interleaved_block_size);
                vector_float received_info;
                vector_float received_parity;
                vector_int decoded_info;
         
                if (d_frame_parameters.robo_mode != NO_ROBO) {
                    vector_float robo_deinterleaved = robo_deinterleaver(block_bits, d_frame_parameters.encoded_block_size, d_frame_parameters.robo_mode);
                    DEBUG_VECTOR(robo_deinterleaved);
                    received_info = channel_deinterleaver(robo_deinterleaved, received_parity, d_frame_parameters.pb_size, d_frame_parameters.rate);
                } else {
                    received_info = channel_deinterleaver(block_bits, received_parity, d_frame_parameters.pb_size, d_frame_parameters.rate);
                }

                decoded_info = tc_decoder(received_info, received_parity, d_frame_parameters.pb_size, d_frame_parameters.rate);

                DEBUG_VECTORINT_PACK(decoded_info);

                vector_int descrambled = scrambler(decoded_info, scrambler_state);
                DEBUG_VECTOR(descrambled);

                payload_bits_iter = std::copy(descrambled.begin(),descrambled.end(), payload_bits_iter);
                blocks_bits_iter += d_frame_parameters.interleaved_block_size;
            }
            DEBUG_VECTOR(payload_bits);
            return payload_bits;
            break;
        }
        case (MPDU_TYPE_SOUND): {
            vector_symbol_freq symbols_freq(symbols.size());
            vector_symbol_freq::iterator symbols_freq_iter = symbols_freq.begin();
            for (vector_symbol::iterator symbols_iter = symbols.begin(); symbols_iter != symbols.end(); symbols_iter++, symbols_freq_iter++) {
                // Perform FFT to get the freq domain of the received signal
                *symbols_freq_iter = fft_real(symbols_iter->begin(), symbols_iter->end());
                DEBUG_VECTOR((*symbols_freq_iter));
            }
            // Get the expected sound symbol
            vector_symbol_freq ref;
            if (d_frame_parameters.pb_size == PB520) {
                ref = SOUND_PB520_FREQ;
            } else if (d_frame_parameters.pb_size == PB136) {
                ref = SOUND_PB136_FREQ;
            }

            estimate_channel_amplitude(symbols_freq.begin(), symbols_freq.end(), ref.begin(), d_broadcast_channel_response);
            DEBUG_VECTOR(d_broadcast_channel_response.amplitude);
            break;
        }
        default:
            break;
    }
    return vector_int(0);
}

void phy_service::set_noise_psd(float n0) {
    d_n0 = n0;
}

bool phy_service::process_ppdu_frame_control(vector_float::const_iterator iter) {
    // Resolve frame control symbol
    iter += GUARD_INTERVAL_FC;
    vector_float fc_symbol_data(NUMBER_OF_CARRIERS * 2);
    vector_float::iterator fc_symbol_data_iter = fc_symbol_data.begin();
    fc_symbol_data_iter = std::copy(iter, iter + (NUMBER_OF_CARRIERS * 2 - ROLLOFF_INTERVAL), fc_symbol_data_iter);
    fc_symbol_data_iter = std::copy(iter - ROLLOFF_INTERVAL, iter, fc_symbol_data_iter);
    vector_int fc_bits = resolve_frame_control_symbol (fc_symbol_data);

    // Determine parameters
    d_frame_parameters = frame_parameters();
    return parse_frame_control(fc_bits, d_frame_parameters);
}

void phy_service::process_noise(vector_float::const_iterator iter_begin, vector_float::const_iterator iter_end) {
    const int M = NUMBER_OF_CARRIERS * 2; // window length
    int R = M/2; // 1-R is the overlapping length
    int N = iter_end - iter_begin; // total signal length
    int K = (N - M) / R + 1; // number of overlapping windows fits in signal
    vector_float w(M); // signal segment multiplied by hamming window
    vector_complex fft;
    d_noise_psd = vector_float(M/2 + 1); // init vector to zero
    for (int k=0; k<K; k++) {
        vector_float::const_iterator iter = iter_begin + k*R;
        for (int i=0; i<M; i++)
            w[i] = *(iter++) + HAMMING_WINDOW[i];
        fft = fft_real(w.begin(), w.end());
        // Calculate the PSD, and average with previous values
        // The first and last frequencies do not count twice in the FFT
        d_noise_psd[0] = d_noise_psd[0] + std::norm(fft[i]) / M / K; 
        d_noise_psd[M/2] = d_noise_psd[M/2] + std::norm(fft[i]) / M / K; 
        for (int i=1; i<M/2; i++)
           d_noise_psd[i] = d_noise_psd[i] + 2 * std::norm(fft[i]) / M / K;
   }
   DEBUG_VECTOR(d_noise_psd);
   return;
}

void phy_service::set_modulation(modulation_type modulation) {
    d_modulation = modulation;
}

void phy_service::set_code_rate(code_rate rate) {
    d_code_rate = rate;
}

void phy_service::set_robo_mode(RoboMode robo_mode) {
    d_robo_mode = robo_mode;
}

int phy_service::get_mpdu_payload_size() {
    return d_frame_parameters.mpdu_payload_size/8;
}

int phy_service::get_ppdu_payload_length() {
    return d_frame_parameters.ppdu_payload_size;
}

MpduType phy_service::get_frame_type() {
    return d_frame_parameters.type;
}

vector_int phy_service::get_sackd() {
    return d_frame_parameters.sackd;
}

void phy_service::get_sackd(unsigned char *sackd_bin) {
    vector_int sackd_bits(get_sackd());
    pack_bitvector(sackd_bits.begin(), sackd_bits.end(), sackd_bin);
    return;
}

int phy_service::get_sackd_size() {
    return d_frame_parameters.sackd.size()/8;
}

int phy_service::get_inter_frame_space() {
    return d_frame_parameters.inter_frame_space;
}

bool phy_service::parse_frame_control (const vector_int &fc_bits, frame_parameters &frame_parameters) {  
    if (!crc24_check(fc_bits)) 
        return false;
    
    int dt = get_field(fc_bits, IEEE1901_FRAME_CONTROL_DT_IH_OFFSET, IEEE1901_FRAME_CONTROL_DT_IH_WIDTH);
    switch (dt) {
        // SOF frame control
        case 1: {
            frame_parameters.type = MPDU_TYPE_SOF;
            DEBUG_VAR(frame_parameters.type);

            int pbsz = get_field(fc_bits, IEEE1901_FRAME_CONTROL_SOF_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOF_PBSZ_WIDTH);
            switch (pbsz) {
                case 0: frame_parameters.pb_size = PB520; DEBUG_ECHO("PbSize = PB520"); break;
                case 1: frame_parameters.pb_size = PB136; DEBUG_ECHO("PbSize = PB136"); break;
            }

            int numsym = get_field(fc_bits, IEEE1901_FRAME_CONTROL_SOF_NUMSYM_OFFSET, IEEE1901_FRAME_CONTROL_SOF_NUMSYM_WIDTH);
            switch (numsym) {
                case 0: frame_parameters.n_expected_symbols = 0; break;
                case 1: frame_parameters.n_expected_symbols = 1; break;
                case 2: frame_parameters.n_expected_symbols = 2; break;
            }

            int tmi = get_field(fc_bits, IEEE1901_FRAME_CONTROL_SOF_TMI_OFFSET, IEEE1901_FRAME_CONTROL_SOF_TMI_WIDTH);
            switch (tmi) {
                case 0: frame_parameters.robo_mode = STD_ROBO; DEBUG_ECHO("Robo Mode = STD_ROBO"); break;
                case 1: frame_parameters.robo_mode = HS_ROBO; DEBUG_ECHO("Robo Mode = HS_ROBO"); break;
                case 2: frame_parameters.robo_mode = MINI_ROBO; DEBUG_ECHO("Robo Mode = MINI_ROBO"); break;
                case 3: frame_parameters.robo_mode = NO_ROBO; frame_parameters.modulation=BPSK; break;
                case 4: frame_parameters.robo_mode = NO_ROBO; frame_parameters.modulation=QPSK; break;
                case 5: frame_parameters.robo_mode = NO_ROBO; frame_parameters.modulation=QAM8; break;
                case 6: frame_parameters.robo_mode = NO_ROBO; frame_parameters.modulation=QAM16; break;
                case 7: frame_parameters.robo_mode = NO_ROBO; frame_parameters.modulation=QAM64; break;
                case 8: frame_parameters.robo_mode = NO_ROBO; frame_parameters.modulation=QAM256; break;
                case 9: frame_parameters.robo_mode = NO_ROBO; frame_parameters.modulation=QAM1024; break;
                case 10: frame_parameters.robo_mode = NO_ROBO; frame_parameters.modulation=QAM4096; break;
                default: frame_parameters.robo_mode = NO_ROBO; frame_parameters.modulation=QPSK; break;
            }

            int fl_width = get_field(fc_bits, IEEE1901_FRAME_CONTROL_SOF_FL_OFFSET, IEEE1901_FRAME_CONTROL_SOF_FL_WIDTH);
                frame_parameters.n_expected_symbols = (fl_width * 1.28 - IEEE1901_RIFS_DEFAULT) / (NUMBER_OF_CARRIERS * 2.0/SAMPLE_RATE);
            DEBUG_VAR(frame_parameters.n_expected_symbols);

            frame_parameters.has_payload = true;
            break;
        }

        // SACK frame control
        case 2: {
            frame_parameters.type = MPDU_TYPE_SACK;
            DEBUG_VAR(frame_parameters.type);

            frame_parameters.sackd = vector_int(IEEE1901_FRAME_CONTROL_SACK_SACKD_WIDTH);
            for (int i=0; i<IEEE1901_FRAME_CONTROL_SACK_SACKD_WIDTH; i++)
                frame_parameters.sackd[i] = fc_bits[IEEE1901_FRAME_CONTROL_SACK_SACKD_OFFSET + i];
            frame_parameters.has_payload = false;           
            break;
        }
        
        // Sound frame control
        case 4: {
            frame_parameters.type = MPDU_TYPE_SOUND;
            DEBUG_VAR(frame_parameters.type);

            int pbsz = get_field(fc_bits, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_WIDTH);
            switch (pbsz) {
                case 0: frame_parameters.pb_size = PB520; frame_parameters.robo_mode = STD_ROBO; DEBUG_ECHO("PbSize = PB520"); break;
                case 1: frame_parameters.pb_size = PB136; frame_parameters.robo_mode = MINI_ROBO; DEBUG_ECHO("PbSize = PB136"); break;
            }

            int fl_width = get_field(fc_bits, IEEE1901_FRAME_CONTROL_SOUND_FL_OFFSET, IEEE1901_FRAME_CONTROL_SOUND_FL_WIDTH);
                frame_parameters.n_expected_symbols = (fl_width * 1.28 - IEEE1901_RIFS_DEFAULT) / (NUMBER_OF_CARRIERS * 2.0 / SAMPLE_RATE);
            DEBUG_VAR(frame_parameters.n_expected_symbols);

            frame_parameters.has_payload = true;
            break;
        }

        // Any other frame type is not supported
        default:
            return false;
            break;
    }

    // Default inter frame space for all frame types
    frame_parameters.inter_frame_space = IEEE1901_RIFS_DEFAULT * SAMPLE_RATE;
    
    // If this frame contains a payload, calculate these parameters
    if (frame_parameters.has_payload) {
        if (frame_parameters.robo_mode != NO_ROBO)
            frame_parameters.carriers = calc_robo_carriers(frame_parameters.robo_mode);
        else 
            frame_parameters.carriers = build_broadcast_carriers(frame_parameters.modulation);
        frame_parameters.rate = d_code_rate;
        frame_parameters.encoded_block_size = calc_encoded_block_size(frame_parameters.rate, frame_parameters.pb_size);
        frame_parameters.interleaved_block_size = calc_interleaved_block_size(frame_parameters.robo_mode, frame_parameters.rate, frame_parameters.pb_size);
        int number_of_blocks = (frame_parameters.n_expected_symbols * frame_parameters.carriers.capacity) / frame_parameters.interleaved_block_size;
        frame_parameters.mpdu_payload_size =  number_of_blocks * calc_block_size(frame_parameters.pb_size);
        frame_parameters.ppdu_payload_size = frame_parameters.n_expected_symbols * (NUMBER_OF_CARRIERS * 2 + GUARD_INTERVAL_PAYLOAD);
    } else {
        frame_parameters.ppdu_payload_size = 0;
    }

    return true;
}

vector_int phy_service::resolve_frame_control_symbol (const vector_float& fc_data) {
    vector_float fc_descaled(fc_data);

    // Demodulate
    vector_float demodulated = symbol_demodulate(fc_data.begin(), fc_data.end(), BROADCAST_CARRIERS, d_broadcast_channel_response);
    DEBUG_VECTOR(demodulated);

    // Undo the diversity copy
    vector_float decopied = combine_copies(demodulated, FRAME_CONTROL_NBITS + 12/2, FRAME_CONTROL_NBITS*2 + 12); // +12/2 because of non standard turbo encoder
    DEBUG_VECTOR(decopied);

    // Undo channel interleaver
    vector_float received_parity;
    vector_float received_info = channel_deinterleaver(decopied, received_parity, PB16, RATE_1_2);

    // Decode
    vector_int info = tc_decoder(received_info, received_parity, PB16, RATE_1_2);
    DEBUG_VECTOR(info);

    return info;
}


bool phy_service::crc24_check(const vector_int &bit_vector) {
    return (crc24(bit_vector) == 0x7FF01C); // The one's complement of 0x800FE3
}

unsigned long phy_service::get_field(const vector_int &bit_vector, int bit_offset, int bit_width) {
    unsigned long value = 0;
    for (int i=0; i<bit_width; i++) 
        value |= bit_vector[bit_offset + i] << (bit_width - i - 1);
    return value;
}

vector_float phy_service::symbol_demodulate (vector_float::const_iterator iter_begin, vector_float::const_iterator iter_end, const Carriers& carriers, const channel_response &channel_response) {
    vector_float soft_bits(carriers.capacity);
    vector_float::iterator soft_bits_iter = soft_bits.begin();
    vector_complex ofdm_symbol = fft_real(iter_begin, iter_end); // Perform FFT to get a freq domain signal
    DEBUG_VECTOR (ofdm_symbol);
    //int j = 0;
    for (int i=0; i<=NUMBER_OF_CARRIERS; i++) { 
        if (carriers.mask[i]) {  // If carrier is ON
            complex r = ofdm_symbol[i] / channel_response.amplitude[i] / std::exp(complex(0,channel_response.phase[i])) / (float)NUMBER_OF_CARRIERS;
            complex p = ANGLE_NUMBER_TO_VALUE[CARRIERS_ANGLE_NUMBER[i]*2]; // Convert the angle number to its value
            complex mapped_value(r.real() * p.real() + r.imag() * p.imag(), 
                                             -r.real() * p.imag() + r.imag() * p.real()); // Rotate channel value by minus angel_number to get the original mapped value
            soft_bits_iter = demodulate(mapped_value, carriers.modulation[i], soft_bits_iter);
        }
    }
    DEBUG_VECTOR(soft_bits);
    return soft_bits;
}


// Calculate number of bits per symbol
void phy_service::update_carriers_capacity(phy_service::Carriers& carriers) {
    carriers.capacity = 0;
    for (unsigned int i = 0; i< NUMBER_OF_CARRIERS+1; i++)
        if (carriers.mask[i])
            carriers.capacity += MODULATION_MAP[carriers.modulation[i]].n_bits;
}

vector_float::iterator phy_service::demodulate(const complex &value, modulation_type modulation, vector_float::iterator iter) {
    ModulationMap modulation_map = MODULATION_MAP[modulation];
    switch (modulation) {
        case BPSK: {
            *iter++ = -4 * std::real(value) * modulation_map.scale / d_n0;        
            break;
        }
        case QPSK: {
            *iter++ = -4 * std::real(value) * modulation_map.scale / d_n0;
            *iter++ = -4 * std::imag(value) * modulation_map.scale / d_n0;
            break;
        }
        case QAM8: {
            iter = demodulate_helper(modulation_map.n_bits-1, std::real(value), modulation_map.scale, d_n0, iter);
            *iter++ = -4 * std::imag(value) * 1.29 * modulation_map.scale / d_n0;
            break;
        }
        case QAM16: 
        case QAM64: 
        case QAM256: 
        case QAM1024: 
        case QAM4096: {
            // itpp::cvec s(1);
            // s(0) = value;
            // itpp::Modulator<std::complex<double>> qam16;
            // itpp::cvec symbols(1<<modulation_map.n_bits);
            // itpp::ivec bits2symbols(symbols.size());
            // for (int i = 0; i < symbols.size(); i++) {
            //     bits2symbols(i) = i;
            //     symbols(i) = modulation_map.map[i] * modulation_map.scale;
            // }
            // qam16.set(symbols,bits2symbols);
            // itpp::vec b = qam16.demodulate_soft_bits(s, d_n0, itpp::APPROX);
            // for (int i = b.size() - 1; i >= 0; i--) {
            //     *iter = b(i);
            //     iter++;
            // }
            // std::cout << "s: " << s << ", b: " << b << std::endl;
            iter = demodulate_helper(modulation_map.n_bits/2, std::real(value), modulation_map.scale, d_n0, iter);
            iter = demodulate_helper(modulation_map.n_bits/2, std::imag(value), modulation_map.scale, d_n0, iter);
            break;           
        }
    }

    return iter;
}
vector_float::iterator phy_service::demodulate_helper(int n_bits, float r, float scale, float n0, vector_float::iterator iter) {
    r = r / scale;
    int k = std::round((r+1)/2)*2-1;
    int l = 1<<n_bits;
    for (int b = 0; b < n_bits; b++) {
        float d = 0;        
        for (int z=0; z<=1; z++) {
            float d_left = l*2*l*2;
            float d_right = d_left;
            // scan to the right and look for bit z (0 or 1)
            for (int i=k; i<=l-1; i+=2) {
                int dec = qam_demodulate(i,l);
                // if found the bit, calculate the distance and break
                if (((dec >> b) & 0x1) == z) {
                    d_right = (r-i)*(r-i);
                    break;
                }
            }

            // scan to the left and look for bit z (0 or 1)
            for (int i=k-2; i>=1-l; i-=2) { 
                int dec = qam_demodulate(i,l);// l-(i+1)/2) % l;
                // if found the bit, calculate the distance and break
                if (((dec >> b) & 0x1) == z) {
                    d_left = (r-i)*(r-i);
                    break;
                }
            }

            // Choose the minimum between the distances
            if (d_left > d_right)
                d = d_right - d; // this means d_final = d_second - d_first
            else 
                d = d_left - d;
        }
        *iter++ = d*scale*scale/n0;
    }
    return iter;
}

inline
int phy_service::qam_demodulate(int v, int l) {
    // The following take the point v from (...-7, -5, -3, -1, 1, 3, 5, 7 ...)
    // and then map that point to ( ... 3, 2, 1, 0, 7, 6, 5, 4 ...) accordingly.
    // This mapping represents the gray code of the target number
    int decimal = (l - ((v + 1) / 2)) % l; 

    // Convert to gray code
    return (decimal >> 1) ^ decimal;
}

vector_float phy_service::combine_copies(vector_float& bitstream, int offset, int n_bits) {
    /* copier should replicate and interleave the 256 bits as follows:
       Original bit number order (k): 0   1   2   3   4   5   6 ... 254 255 0   1   2   3   4   5   ... 254 255 ...
                           New order: 0   128 1   129 2   130 3 ... 127 255 128 0   129 1   130 2   ... 255 128 ...
                            Location: 0   1   2   3   4   5   6 ... 254 255 256 257 258 259 260 261 ... 510 511 ... n_carriers*2
    */

    vector_float decopier_output(n_bits, 0);
    for (unsigned int i = 0; i<bitstream.size()/2; i++) {
        // These are soft bits. Their combined value is their sum
        decopier_output[i % n_bits] += bitstream[i*2];
        decopier_output[(i+offset) % n_bits] += bitstream[i*2+1];
    }

    return decopier_output;
}

vector_float phy_service::channel_deinterleaver(const vector_float& bitstream, vector_float& parity_bitstream, PbSize pb_size, code_rate rate) {
    int step_size = CHANNEL_INTERLEAVER_STEPSIZE[pb_size][rate];
    int offset = CHANNEL_INTERLEAVER_OFFSET[pb_size][rate];
    int info_row_no = 0;
    int n_info_rows_done = 0;
    int n_parity_rows_done = 0;
    int parity_row_no = offset;
    bool parity_done = false, info_done = false;
    int size = bitstream.size();
    int nibble_no = 0;
    vector_float::const_iterator iter = bitstream.begin();
    vector_float info_bitstream;

    switch (rate) {
    case RATE_1_2: {
        info_bitstream = vector_float((size - 12) / 2); //  adjusted due to non standard turbo encoder...
        parity_bitstream = vector_float(size - info_bitstream.size()); 

        while (!info_done || !parity_done) {
            info_done = channel_deinterleaver_row(iter, info_bitstream, step_size, info_row_no, n_info_rows_done, nibble_no);
            parity_done = channel_deinterleaver_row(iter, parity_bitstream, step_size, parity_row_no, n_parity_rows_done, nibble_no);
        }
        break;
    }

    case RATE_16_21: {
        info_bitstream = vector_float(size * 16 / 21);
        parity_bitstream = vector_float(size * 5 / 21);
        while (!info_done && !parity_done) {
            for (int i=0; i<5; i++) {
                info_done = channel_deinterleaver_row(iter, info_bitstream, step_size, info_row_no, n_info_rows_done, nibble_no);
                info_done = channel_deinterleaver_row(iter, info_bitstream, step_size, info_row_no, n_info_rows_done, nibble_no);
                info_done = channel_deinterleaver_row(iter, info_bitstream, step_size, info_row_no, n_info_rows_done, nibble_no);
                parity_done = channel_deinterleaver_row(iter, parity_bitstream, step_size, parity_row_no, n_parity_rows_done, nibble_no, true);
            }
            info_done = channel_deinterleaver_row(iter, info_bitstream, step_size, info_row_no, n_info_rows_done, nibble_no);
        }
        break;
    }

    case RATE_16_18: {
        info_bitstream = vector_float(size * 16 / 18);
        parity_bitstream = vector_float(size * 2 / 18);
        while (!info_done && !parity_done) {
            info_done = channel_deinterleaver_row(iter, info_bitstream, step_size, info_row_no, n_info_rows_done, nibble_no);
            info_done = channel_deinterleaver_row(iter, info_bitstream, step_size, info_row_no, n_info_rows_done, nibble_no);
            info_done = channel_deinterleaver_row(iter, info_bitstream, step_size, info_row_no, n_info_rows_done, nibble_no);
            parity_done = channel_deinterleaver_row(iter, parity_bitstream, step_size, parity_row_no, n_parity_rows_done, nibble_no, true);
            info_done = channel_deinterleaver_row(iter, info_bitstream, step_size, info_row_no, n_info_rows_done, nibble_no);
            info_done = channel_deinterleaver_row(iter, info_bitstream, step_size, info_row_no, n_info_rows_done, nibble_no);
            info_done = channel_deinterleaver_row(iter, info_bitstream, step_size, info_row_no, n_info_rows_done, nibble_no);
            info_done = channel_deinterleaver_row(iter, info_bitstream, step_size, info_row_no, n_info_rows_done, nibble_no);
            info_done = channel_deinterleaver_row(iter, info_bitstream, step_size, info_row_no, n_info_rows_done, nibble_no);
        }
        break;
    }
    }
    return info_bitstream;
}

bool phy_service::channel_deinterleaver_row(vector_float::const_iterator& iter, vector_float& out, int step_size, int& row_no, int& rows_done, int& nibble_no, bool wrap) {
    int n_rows = out.size()/4;
    if (rows_done < n_rows) {
         // Reading a row
        for (int i = nibble_no / 2; i < (nibble_no / 2) + 4 ; i++, iter++)
            out[row_no + (i % 4) * n_rows] = *iter;
        rows_done++;
        nibble_no = (nibble_no + 1) % 8;

        // Calculating the next row number
        if (wrap || (row_no + step_size < n_rows))
            row_no = (row_no + step_size) % n_rows;
        else
            row_no = (row_no % step_size) + 1;
    }
    if (rows_done == n_rows) 
        return true;
    return false;
}

vector_float phy_service::robo_deinterleaver(const vector_float& bitstream, int n_raw, RoboMode robo_mode)  { 
    unsigned int n_copies, bits_in_last_symbol, bits_in_segment, n_pad;
    calc_robo_parameters (robo_mode, n_raw, n_copies, bits_in_last_symbol, bits_in_segment, n_pad);      
    assert (bitstream.size() % n_copies == 0); // input should have exactly n_copies of n_raw+n_pad
    // Set the bits shift parameters
    std::vector<int> cycle_shifts(n_copies,0);
    int l = (bits_in_last_symbol - 1) / bits_in_segment;
    switch (n_copies) {
        case 2:  // l \in {0,1}
            if (l==1) 
                cycle_shifts = {0,1};
            break;
        case 4:  // l \in {0,1,2,3}
            if (l==1)
                cycle_shifts = {0,0,1,1};
            else if (l==3)
                cycle_shifts = {0,1,2,3};
            break;
        case 5:  // l \in {0,1,2,3,4}
            if (l==4)
                cycle_shifts = {0,1,2,3,4};
            break;
    }

    // Combine robo copies
    vector_float combined_copy(n_raw);
    vector_float::const_iterator it = bitstream.begin();
    for (unsigned int k=0; k<n_copies; k++) {

        // Each copy is arranged as follows:
        // [part 2: n_shifted] [n_pad] [part 1: n_raw-n_shifted]
        // Only the first and last part are actual data bits. 
        // If n_shifted is zero than it is arranged as follows:
        // [n_raw] [n_pad]

        vector_float::iterator combined_copy_iter = combined_copy.begin();
        // Since the bits are soft bits, we add all copies to a combined soft bit
        if (cycle_shifts[k] == 0)
            combined_copy_iter = std::transform (it, it + n_raw, combined_copy_iter, combined_copy_iter, std::plus<float>());
        else {
            int start_position = cycle_shifts[k] * bits_in_segment;
            combined_copy_iter = std::transform (it + start_position, it + n_raw + n_pad, combined_copy_iter, combined_copy_iter, std::plus<float>());
            combined_copy_iter = std::transform (it, it + start_position - n_pad, combined_copy_iter, combined_copy_iter, std::plus<float>());
        }
        it += n_raw + n_pad;
    }
    return combined_copy;
}



int phy_service::calc_interleaved_block_size(RoboMode robo_mode, code_rate rate, PbSize pb_size) {
    int encoded_pb_n_bits = calc_encoded_block_size(rate, pb_size);

    // Determine parameters for ROBO mode
    unsigned int n_copies = 1, n_pad = 0, bits_in_last_symbol, bits_in_segment; // Default values when not in ROBO mode
    if (robo_mode != NO_ROBO) {
        calc_robo_parameters (robo_mode, encoded_pb_n_bits, n_copies, bits_in_last_symbol, bits_in_segment, n_pad);
        return (encoded_pb_n_bits + n_pad) * n_copies;
    }
    return encoded_pb_n_bits;
}

inline int phy_service::calc_block_size(PbSize pb_size) {
    if (pb_size == PB520) 
        return 520*8;
    else if (pb_size == PB136)
        return 136*8;
    else return 16*8;
}
int phy_service::calc_encoded_block_size(code_rate rate, PbSize pb_size) {
    // Determine number of bits in original block
    int block_n_bits = (pb_size == PB520) ? 520*8 : 136*8;
    assert (pb_size == PB520 || pb_size == PB136); // Cannot have payload blocks of 16 octets

    // Calculate number of bits in an encoded PB520 block (with no ROBO mode)
    int encoded_pb_n_bits = 0;
    switch (rate) {
        case RATE_1_2: encoded_pb_n_bits = block_n_bits * 2 + 12; break; // +12 due to non standard turbo encoder...
        case RATE_16_21: encoded_pb_n_bits = block_n_bits * 21 / 16; break;
        case RATE_16_18: encoded_pb_n_bits = block_n_bits * 18 / 16; break;
    }
    return encoded_pb_n_bits;
}

int phy_service::max_blocks (RoboMode robo_mode, code_rate rate, modulation_type modulation) {  
    int encoded_pb_n_bits = calc_interleaved_block_size(robo_mode, rate, PB520);

    float symbol_durarion = NUMBER_OF_CARRIERS * 2.0 / SAMPLE_RATE; // one symbol duration (microseconds)
    float max_frame_duration = (((1 << IEEE1901_FRAME_CONTROL_SOF_FL_WIDTH) - 1) * 1.28 - IEEE1901_RIFS_DEFAULT); // maximum of all symbols duration allowed (microseconds)
    int max_n_symbols = max_frame_duration / symbol_durarion; // maximum number of symbols
    int max_n_bits = max_n_symbols * build_broadcast_carriers(modulation).capacity; // maximum number of bits
    return max_n_bits / encoded_pb_n_bits; // maximum number of PB520 blocks
}

phy_service::Carriers phy_service::build_broadcast_carriers(modulation_type modulation) {
    phy_service::Carriers broadcast_carriers;
    broadcast_carriers.mask = CARRIERS_BROADCAST_MASK;
    broadcast_carriers.modulation.fill(modulation);
    update_carriers_capacity(broadcast_carriers);
    return broadcast_carriers;
}

void phy_service::create_fftw_vars () {
    // Create fftw3 plan (used by fft_real and ifft_real routine)
    d_ifft_input = fftwf_alloc_complex(NUMBER_OF_CARRIERS + 1);
    d_ifft_output = fftwf_alloc_real(NUMBER_OF_CARRIERS * 2);
    d_fftw_rev_plan = fftwf_plan_dft_c2r_1d (NUMBER_OF_CARRIERS*2,
                                            d_ifft_input,
                                            d_ifft_output,
                                            FFTW_MEASURE);

    d_fft_input = fftwf_alloc_real(NUMBER_OF_CARRIERS * 2);
    d_fft_output = fftwf_alloc_complex(NUMBER_OF_CARRIERS + 1);
    d_fftw_fwd_plan = fftwf_plan_dft_r2c_1d (NUMBER_OF_CARRIERS*2,
                                            d_fft_input,
                                            d_fft_output,
                                            FFTW_MEASURE);

    d_fft_syncp_input = fftwf_alloc_real(SYNCP_SIZE);
    d_fft_syncp_output = fftwf_alloc_complex(SYNCP_SIZE / 2 + 1);
    d_fftw_syncp_fwd_plan = fftwf_plan_dft_r2c_1d (SYNCP_SIZE,
                                            d_fft_syncp_input,
                                            d_fft_syncp_output,
                                            FFTW_MEASURE);
}

vector_complex phy_service::fft_real_syncp(const vector_float& data) {
    assert (data.size() == SYNCP_SIZE);
    memcpy(d_fft_syncp_input, data.data(), sizeof(float) * SYNCP_SIZE);
    fftwf_execute(d_fftw_syncp_fwd_plan);
    vector_complex output(SYNCP_SIZE / 2 + 1);
    memcpy (output.data(), d_fft_syncp_output, (SYNCP_SIZE / 2 + 1) * sizeof(fftwf_complex));
    return output; 
}

void phy_service::process_ppdu_preamble (vector_float::const_iterator iter, vector_float::const_iterator iter_end) {
    assert (iter_end - iter == PREAMBLE_SIZE);
    DEBUG_VECTOR(SYNCP_FREQ);
    iter += SYNCP_SIZE / 2 + SYNCP_SIZE;
    vector_float syncp(iter, iter + SYNCP_SIZE);
    DEBUG_VECTOR(syncp);
    vector_complex syncp_freq = fft_real_syncp(syncp);
    DEBUG_VECTOR(syncp_freq);
    estimate_channel_phase(syncp_freq.begin(), syncp_freq.end(), SYNCP_FREQ.begin(), SYNCP_CARRIERS_MASK, d_broadcast_channel_response);
    DEBUG_VECTOR(d_broadcast_channel_response.phase);
}

void phy_service::estimate_channel_amplitude (vector_symbol_freq::const_iterator iter, vector_symbol_freq::const_iterator iter_end, vector_symbol_freq::const_iterator ref_iter, channel_response &channel_response) {
    int nsymbols = iter_end - iter;
    channel_response.amplitude = carriers_response(); // zero the amplitude array
    while (iter != iter_end) {
        assert (iter->end() - iter->begin() == (unsigned int)channel_response.amplitude.size());
        int i = 0;
        vector_symbol_freq::value_type::const_iterator carrier_iter = iter->begin();
        vector_symbol_freq::value_type::const_iterator carrier_ref_iter = ref_iter->begin();
        while (carrier_iter != iter->end()) { 
            if (channel_response.mask[i])  // If carrier is ON
                channel_response.amplitude[i] += std::abs(*carrier_iter / *carrier_ref_iter) / nsymbols; // Calculate the average carrier response
            carrier_iter++;
            carrier_ref_iter++;
            i++;
        }
        iter++;
        ref_iter++;
    }
    return;
}

void phy_service::estimate_channel_phase (vector_complex::const_iterator iter, vector_complex::const_iterator iter_end, vector_complex::const_iterator ref_iter, bool const cm[], channel_response &channel_response) {
    int length = iter_end - iter;
    int n = count_non_masked_carriers(cm, cm+length-1);
    vector_float x(n);
    vector_float y(n);
    x.reserve(length);
    y.reserve(length);
    int i=0, j=0;
    while (iter != iter_end) { 
        if (cm[i]) {
            x[j] = i * NUMBER_OF_CARRIERS / length;
            y[j] = std::arg(*iter / *ref_iter);
            j++;
        }
        iter++;
        ref_iter++;
        i++;
    }
    DEBUG_VECTOR(x);
    DEBUG_VECTOR(y);

    std::vector<SplineSet> splineSet = spline(x,y);
    for (unsigned int i = 0, j = 0; i < channel_response.phase.size(); i++) {
        if (channel_response.mask[i]) {
            while (j < splineSet.size()-1 && splineSet[j+1].x <= i) j++;
            float dx = i - splineSet[j].x;
            channel_response.phase[i] = splineSet[j].a + splineSet[j].b * dx + splineSet[j].c * dx * dx +
                                                      splineSet[j].d * dx * dx * dx;
        }
    } 
}

std::array<float, phy_service::NUMBER_OF_CARRIERS*2> phy_service::create_hamming_window() {
    std::array<float, NUMBER_OF_CARRIERS*2> window;
    float a = 0.53836;
    float b = 0.46164;
    for (int n = 0; n<NUMBER_OF_CARRIERS*2; n++)
        window[n] = a - b * cos(((float)2 * M_PI * (float)n) / (NUMBER_OF_CARRIERS * 2 - 1));
    return window;
}

std::vector<phy_service::SplineSet> phy_service::spline(vector_float &x, vector_float &y)
{
    int n = x.size()-1;
    vector_float a;
    a.insert(a.begin(), y.begin(), y.end());
    vector_float b(n);
    vector_float d(n);
    vector_float h;

    for(int i = 0; i < n; ++i)
        h.push_back(x[i+1]-x[i]);

    vector_float alpha;
    for(int i = 0; i < n; ++i)
        alpha.push_back( (float)3*(a[i+1]-a[i])/h[i] - (float)3*(a[i]-a[i-1])/h[i-1]  );

    vector_float c(n+1);
    vector_float l(n+1);
    vector_float mu(n+1);
    vector_float z(n+1);
    l[0] = 1;
    mu[0] = 0;
    z[0] = 0;

    for(int i = 1; i < n; ++i)
    {
        l[i] = 2 *(x[i+1]-x[i-1])-h[i-1]*mu[i-1];
        mu[i] = h[i]/l[i];
        z[i] = (alpha[i]-h[i-1]*z[i-1])/l[i];
    }

    l[n] = 1;
    z[n] = 0;
    c[n] = 0;

    for(int j = n-1; j >= 0; --j)
    {
        c[j] = z [j] - mu[j] * c[j+1];
        b[j] = (a[j+1]-a[j])/h[j]-h[j]*(c[j+1]+(float)2*c[j])/(float)3;
        d[j] = (c[j+1]-c[j])/(float)3/h[j];
    }

    std::vector<SplineSet> output_set(n);
    for(int i = 0; i < n; ++i)
    {
        output_set[i].a = a[i];
        output_set[i].b = b[i];
        output_set[i].c = c[i];
        output_set[i].d = d[i];
        output_set[i].x = x[i];
    }
    return output_set;
}
} /* namespace light_plc */