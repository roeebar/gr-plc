#include "phy_service.h"
#include "debug.h"
#include <iostream>
#include <algorithm>
#include <functional>
#include <assert.h>
#include <cmath>
#include <itpp/comm/modulator.h>
#include <queue>

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

const phy_service::modulation_map_t phy_service::MODULATION_MAP[9] = { 
                         {0,NULL,0},
                         {MAP_BPSK_NBITS, MAP_BPSK, MAP_BPSK_SCALE}, 
                         {MAP_QPSK_NBITS, MAP_QPSK, MAP_QPSK_SCALE}, 
                         {MAP_QAM8_NBITS, MAP_QAM8, MAP_QAM8_SCALE}, 
                         {MAP_QAM16_NBITS, MAP_QAM16, MAP_QAM16_SCALE},
                         {MAP_QAM64_NBITS, MAP_QAM64, MAP_QAM64_SCALE},
                         {MAP_QAM256_NBITS, MAP_QAM256, MAP_QAM256_SCALE},
                         {MAP_QAM1024_NBITS, MAP_QAM1024, MAP_QAM1024_SCALE},
                         {MAP_QAM4096_NBITS, MAP_QAM4096, MAP_QAM4096_SCALE} };

const complex phy_service::ANGLE_NUMBER_TO_VALUE[16] = {complex(1.000000000000000,0.000000000000000),
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
const phy_service::tone_info_t phy_service::BROADCAST_CARRIERS = phy_service::build_broadcast_tone_map();
const int phy_service::N_BROADCAST_CARRIERS = phy_service::count_non_masked_carriers(CARRIERS_BROADCAST_MASK.begin(), CARRIERS_BROADCAST_MASK.end());

phy_service::phy_service (bool debug) : d_debug(debug), d_tone_info(build_broadcast_tone_map()), d_code_rate(RATE_1_2), d_n0(1), PREAMBLE(calc_preamble()), SYNCP_FREQ(calc_syncp_fft(PREAMBLE)), TURBO_INTERLEAVER_SEQUENCE(calc_turbo_interleaver_sequence()) {
    static_assert(BPSK==1 && QPSK==2 && QAM8==3 && QAM16==4 && QAM64==5 && QAM256==6 && QAM1024==7 && QAM4096==8, "Mapping parameters error");
    create_fftw_vars();
    init_turbo_codec();
    d_broadcast_channel_response.mask = CARRIERS_BROADCAST_MASK;
    d_broadcast_channel_response.carriers_gain.fill(1);
    d_noise_psd.fill(0);
}

phy_service::phy_service (const phy_service &obj) : 
        d_debug(obj.d_debug),
        d_tone_info(obj.d_tone_info),
        d_code_rate(obj.d_code_rate),
        d_n0(obj.d_n0),
        PREAMBLE(obj.PREAMBLE), 
        SYNCP_FREQ(obj.SYNCP_FREQ), 
        TURBO_INTERLEAVER_SEQUENCE(obj.TURBO_INTERLEAVER_SEQUENCE),
        d_broadcast_channel_response(obj.d_broadcast_channel_response),
        d_noise_psd(obj.d_noise_psd),
        d_snr(obj.d_snr) {
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
    std::swap(d_noise_psd, tmp.d_noise_psd);
    std::swap(d_snr, tmp.d_snr);
    std::swap(d_code_rate, tmp.d_code_rate);
    std::swap(d_tone_info, tmp.d_tone_info);
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

vector_float phy_service::create_ppdu(const unsigned char *mpdu_fc_bin, size_t mpdu_fc_len, const unsigned char *mpdu_payload_bin, size_t mpdu_payload_len) {
    vector_int mpdu_payload_int(unpack_into_bitvector(mpdu_payload_bin, mpdu_payload_len));
    vector_int mpdu_fc_int(unpack_into_bitvector(mpdu_fc_bin, mpdu_fc_len));
    return create_ppdu(mpdu_fc_int, mpdu_payload_int);
}

vector_float phy_service::create_ppdu(vector_int &mpdu_fc_int, const vector_int &mpdu_payload_int) {
    assert(mpdu_fc_int.size() == FRAME_CONTROL_NBITS);
    ppdu_mode_t ppdu_mode = get_mode(mpdu_fc_int);
    update_frame_control(mpdu_fc_int, ppdu_mode, mpdu_payload_int.size());

    // Encode frame control
    DEBUG_ECHO("Encoding frame control...")
    DEBUG_VECTOR(mpdu_fc_int);
    vector_symbol fc_symbols = create_frame_control_symbol(mpdu_fc_int);

    // Encode payload blocks
    vector_symbol payload_symbols;
    if (ppdu_mode.has_payload) {
        DEBUG_ECHO("Encoding payload blocks...")
        DEBUG_VECTOR(mpdu_payload_int);
        payload_symbols = create_payload_symbols(mpdu_payload_int, ppdu_mode.pb_size, ppdu_mode.robo_mode, d_tone_info, d_code_rate);
    }

    // Create the final symbols array
    DEBUG_ECHO("Creating final data stream...")
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

phy_service::ppdu_mode_t phy_service::get_mode (const vector_int &mpdu_fc_int) {  
    ppdu_mode_t ppdu_mode;
    mpdu_type_t dt = (mpdu_type_t)get_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_DT_IH_OFFSET, IEEE1901_FRAME_CONTROL_DT_IH_WIDTH);
    switch (dt) {
        case MPDU_TYPE_SOF: {
            int tmi = get_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_SOF_TMI_OFFSET, IEEE1901_FRAME_CONTROL_SOF_TMI_WIDTH);
            switch (tmi) {
                case 0: ppdu_mode.robo_mode = STD_ROBO; break;
                case 1: ppdu_mode.robo_mode = HS_ROBO; break;
                case 2: ppdu_mode.robo_mode = MINI_ROBO; break;
                default: ppdu_mode.robo_mode = NO_ROBO; break;
            }
            int pbsz = get_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_SOF_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOF_PBSZ_WIDTH);
            switch (pbsz) {
                case 0: ppdu_mode.pb_size = PB520; break;
                case 1: ppdu_mode.pb_size = PB136; break;
            }
            ppdu_mode.has_payload = true;
            break;
        }
        case MPDU_TYPE_SOUND: {
            int pbsz = get_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_WIDTH);
            switch (pbsz) {
                case 0: ppdu_mode.pb_size = PB520; ppdu_mode.robo_mode = STD_ROBO; break;
                case 1: ppdu_mode.pb_size = PB136; ppdu_mode.robo_mode = MINI_ROBO; break;
            }
            ppdu_mode.has_payload = true;
            break;
        }
        case MPDU_TYPE_SACK:
            ppdu_mode.has_payload = false;
            break;

        default:
            break;
    }
    return ppdu_mode;
}

void phy_service::update_frame_control (vector_int &mpdu_fc_int, ppdu_mode_t ppdu_mode, size_t payload_size) {  
    mpdu_type_t dt = (mpdu_type_t)get_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_DT_IH_OFFSET, IEEE1901_FRAME_CONTROL_DT_IH_WIDTH);
    int fl_width = 0;
    if (ppdu_mode.has_payload) {
        // Calculate frame length
        tone_info_t tone_info = calc_tone_info(ppdu_mode.robo_mode);
        int n_blocks = ppdu_mode.pb_size == PB520 ? payload_size / (520*8) : payload_size / (136*8);
        int interleaved_block_size = calc_interleaved_block_size(ppdu_mode.robo_mode, d_code_rate, ppdu_mode.pb_size);
        int n_bits = n_blocks * interleaved_block_size;
        int n_symbols = (n_bits % tone_info.capacity) ? n_bits / tone_info.capacity + 1 : n_bits / tone_info.capacity;
        fl_width = std::ceil((n_symbols * (NUMBER_OF_CARRIERS*2.0/SAMPLE_RATE) + IEEE1901_RIFS_DEFAULT)/1.28);
    }
    switch (dt) {
        case MPDU_TYPE_SOF:
            set_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_SOF_FL_OFFSET, IEEE1901_FRAME_CONTROL_SOF_FL_WIDTH, fl_width);
            break;
        case MPDU_TYPE_SOUND: 
            set_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_SOUND_FL_OFFSET, IEEE1901_FRAME_CONTROL_SOUND_FL_WIDTH, fl_width);
            break;
        default:
            break;
    }

    // Calculate and set CRC24
    unsigned long crc = crc24(vector_int(mpdu_fc_int.begin(),mpdu_fc_int.end()-24));
    set_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_FCCS_OFFSET + 16, 8, crc & 0xFF); 
    set_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_FCCS_OFFSET + 8, 8, (crc >> 8) & 0xFF); 
    set_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_FCCS_OFFSET, 8, (crc >> 16) & 0xFF); 

    return;
}

void set_field(vector_int &bit_vector, int bit_offset, int bit_width, unsigned long new_value) {
    for (int i=0; i<bit_width; i++) {
        int j = bit_offset + i;
        int calc_offset = 8 * (j/8) + 7 - j % 8;
        bit_vector[calc_offset] = new_value & 0x1;
        new_value >>= 1;
    }
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

vector_symbol_freq phy_service::create_payload_symbols_freq(const vector_int &payload_bits, pb_size_t pb_size, robo_mode_t robo_mode, phy_service::tone_info_t tone_info, code_rate rate) {
    // Determine number of blocks and blocks size
    int block_n_bits = (pb_size == PB520) ? 520*8 : 136*8;
    assert (pb_size == PB520 || pb_size == PB136); // Cannot have payload blocks of 16 octets

    int n_blocks = payload_bits.size() / block_n_bits;
    assert ((payload_bits.size() % block_n_bits) == 0);

    int block_size = calc_interleaved_block_size(robo_mode, rate, pb_size);
    // Encode blocks
    if (robo_mode != NO_ROBO) {
        tone_info = calc_tone_info(robo_mode);
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
    return modulate(encoded_payload_bits, tone_info);
}

vector_symbol phy_service::create_payload_symbols(const vector_int &payload_bits, pb_size_t pb_size, robo_mode_t robo_mode, phy_service::tone_info_t tone_info, code_rate rate) {
    vector_symbol_freq symbols_freq = create_payload_symbols_freq(payload_bits, pb_size, robo_mode, tone_info, rate);

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

vector_int phy_service::tc_encoder(const vector_int &bitstream, pb_size_t pb_size, code_rate rate) {
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

vector_int phy_service::tc_decoder(const vector_float &received_info, const vector_float &received_parity, pb_size_t pb_size, code_rate rate) {
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

vector_int phy_service::consistuent_encoder(const vector_int& in, pb_size_t pb_size) {
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


vector_int phy_service::turbo_interleaver(const vector_int &bitstream, pb_size_t pb_size){

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

vector_int phy_service::channel_interleaver(const vector_int& bitstream, const vector_int& parity_bitstream, pb_size_t pb_size, code_rate rate) {
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

vector_int phy_service::robo_interleaver(const vector_int& bitstream, robo_mode_t robo_mode) {
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

phy_service::tone_info_t phy_service::calc_tone_info (robo_mode_t robo_mode) {
    // Create basic ROBO carriers consists of broadcast mask and QPSK
    tone_info_t tone_info = BROADCAST_CARRIERS;

    int n_copies = 0;

    // Each mode replicates the bits n_copies times
    switch (robo_mode) { 
        case STD_ROBO: n_copies = 4; break;
        case HS_ROBO: n_copies = 2; break;
        case MINI_ROBO: n_copies = 5; break;
        case NO_ROBO: return d_tone_info; break; 
    }
    assert (robo_mode == STD_ROBO || robo_mode == HS_ROBO || robo_mode == MINI_ROBO); // ROBO mode should be only one of these

    unsigned int n_carriers = N_BROADCAST_CARRIERS;
    unsigned int n_carriers_robo = n_copies * (n_carriers / n_copies);

    // Mark the unused carriers as masked
    unsigned int j = 0, i = tone_info.tone_map.size()-1;
    while (j<(n_carriers-n_carriers_robo)) {
        if (tone_info.tone_map[i] != NULLED ) {
            tone_info.tone_map[i] = NULLED;
            j++;
        }
        i--;
    }
    update_tone_info_capacity(tone_info);

    return tone_info; 
}

void phy_service::calc_robo_parameters (robo_mode_t robo_mode, unsigned int n_raw, unsigned int &n_copies, unsigned int &bits_in_last_symbol, unsigned int &bits_in_segment, unsigned int &n_pad) {
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

vector_symbol_freq phy_service::modulate(const vector_int& bits, const phy_service::tone_info_t& tone_info) {
    // Calculate number of symbols needed
    int n_symbols = (bits.size() && (bits.size() % tone_info.capacity)) ? bits.size() / tone_info.capacity + 1 : bits.size() / tone_info.capacity;
    vector_symbol_freq symbols_freq(n_symbols);
    // Perform mapping
    vector_int::const_iterator it = bits.cbegin();
    int pn_state = pn_generator_init();
    for (int j = 0; j < n_symbols; j++) { 
        vector_complex mapped_bits = vector_complex(NUMBER_OF_CARRIERS+1);
        for (int i=0; i<=NUMBER_OF_CARRIERS; i++) {
            if (!TRANSMIT_MASK[i]) // if regulations do not allow the carrier to transmit
                continue;
            else if (tone_info.tone_map[i] != NULLED) {  // If carrier is ON
                modulation_map_t modulation_map = MODULATION_MAP[tone_info.tone_map[i]];
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
                // Calculate the mapped value. Multiplying by the scale for unity average power.
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
    
    // Slice to symbols    
    vector_symbol symbols(d_frame_parameters.n_expected_symbols);
    for (unsigned int i = 0; i < d_frame_parameters.n_expected_symbols; i++) {
        vector_float payload_symbol(NUMBER_OF_CARRIERS*2);
        vector_float::iterator payload_symbol_iter = payload_symbol.begin();
        payload_symbol_iter = std::copy(iter, iter + (NUMBER_OF_CARRIERS * 2 - ROLLOFF_INTERVAL), payload_symbol_iter);
        payload_symbol_iter = std::copy(iter - ROLLOFF_INTERVAL, iter, payload_symbol_iter);

        symbols[i] = payload_symbol;
        iter += NUMBER_OF_CARRIERS * 2 + GUARD_INTERVAL_PAYLOAD;
    }

    // Calc the freq domain symbols
    vector_symbol_freq symbols_freq(symbols.size());
    vector_symbol_freq::iterator symbols_freq_iter = symbols_freq.begin();
    for (vector_symbol::const_iterator symbols_iter = symbols.begin(); symbols_iter != symbols.end(); symbols_iter++, symbols_freq_iter++) {
        // Perform FFT to get the freq domain of the received signal
        *symbols_freq_iter = fft_real(symbols_iter->begin(), symbols_iter->end());
        DEBUG_VECTOR((*symbols_freq_iter));
    }

    switch (d_frame_parameters.type) {
        case (MPDU_TYPE_SOF):
        case (MPDU_TYPE_SOUND): {
            // Determine the decoded blocks size
            int pb_n_bits = (d_frame_parameters.pb_size == PB520) ? 520*8 : 136*8;
            assert (d_frame_parameters.pb_size == PB520 || d_frame_parameters.pb_size == PB136); // Cannot have payload blocks of 16 octets

            vector_float blocks_bits(d_frame_parameters.tone_info.capacity * symbols.size());
            vector_float::iterator blocks_bits_iter = blocks_bits.begin();
            for (vector_symbol_freq::iterator symbol_freq_iter = symbols_freq.begin(); symbol_freq_iter != symbols_freq.end(); symbol_freq_iter++) {
                // Demodulate 
                vector_float new_bits = symbol_demodulate(symbol_freq_iter->begin(), d_frame_parameters.tone_info, d_broadcast_channel_response);
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

            vector_symbol_freq symbols_freq_ref = create_payload_symbols_freq(payload_bits, d_frame_parameters.pb_size, d_frame_parameters.robo_mode, d_frame_parameters.tone_info, d_frame_parameters.rate);
            estimate_channel_gain(symbols_freq.begin(), symbols_freq.end(), symbols_freq_ref.begin(), d_broadcast_channel_response);
            DEBUG_VECTOR(d_broadcast_channel_response.carriers_gain);
            return payload_bits;
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

bool phy_service::process_ppdu_frame_control(vector_float::const_iterator iter, unsigned char* mpdu_fc_bin) {
    vector_int mpdu_fc_int;
    if (process_ppdu_frame_control(iter, mpdu_fc_int) == true) {
        if (mpdu_fc_bin != NULL)
            pack_bitvector(mpdu_fc_int.begin(), mpdu_fc_int.end(), mpdu_fc_bin);
        return true;
    }
    return false;
}

bool phy_service::process_ppdu_frame_control(vector_float::const_iterator iter, vector_int &mpdu_fc_int) {
    // Resolve frame control symbol
    iter += GUARD_INTERVAL_FC;
    vector_float fc_symbol_data(NUMBER_OF_CARRIERS * 2);
    vector_float::iterator fc_symbol_data_iter = fc_symbol_data.begin();
    fc_symbol_data_iter = std::copy(iter, iter + (NUMBER_OF_CARRIERS * 2 - ROLLOFF_INTERVAL), fc_symbol_data_iter);
    fc_symbol_data_iter = std::copy(iter - ROLLOFF_INTERVAL, iter, fc_symbol_data_iter);
    mpdu_fc_int = resolve_frame_control_symbol (fc_symbol_data);

    // Determine parameters
    d_frame_parameters = frame_parameters();
    return parse_frame_control(mpdu_fc_int, d_frame_parameters);
}

void phy_service::process_noise(vector_float::const_iterator iter_begin, vector_float::const_iterator iter_end) {
    const int M = NUMBER_OF_CARRIERS * 2; // window length
    int R = M/2; // 1-R is the overlapping length
    int N = iter_end - iter_begin; // total signal length
    int K = (N - M) / R + 1; // number of overlapping windows fits in signal
    vector_float w(M); // signal segment multiplied by hamming window
    vector_complex fft;
    d_noise_psd.fill(0); // init vector to zero
    for (int k=0; k<K; k++) {
        vector_float::const_iterator iter = iter_begin + k*R;
        for (int i=0; i<M; i++)
            w[i] = *(iter++) + HAMMING_WINDOW[i];
        fft = fft_real(w.begin(), w.end());
        // Calculate the PSD, and average with previous values
        for (int i=0; i<M/2+1; i++)
           d_noise_psd[i] = d_noise_psd[i] + std::norm(fft[i]) / K;
   }
   DEBUG_VECTOR(d_noise_psd);

    // SNR calculation:
    // Payload carrier transmit average power: T = (N/2 * p)^2  (p is the transmitter amplifier)
    // Payload carrier received power is (H*N/2)^2 where H is the carrier (estimated) gain which implicetly includes the transmitted amplifer p
    // Payload carrier SNR = (H*T)^2 / (noise_var)^2 where noise_var is the estimated noise variance for that carrier
    for (int i=0; i<NUMBER_OF_CARRIERS+1; i++)
        if (d_broadcast_channel_response.mask[i])
            d_snr[i] = std::norm(d_broadcast_channel_response.carriers_gain[i] * NUMBER_OF_CARRIERS) / d_noise_psd[i];
        else
            d_snr[i] = 0;
   DEBUG_VECTOR(d_snr);
    
    return;
}

tone_map_t phy_service::calculate_tone_map(float P_t) {
    typedef std::pair<float,int> carrier_ber_t;

    tone_map_t tone_map;
    tone_map.fill(NULLED);
    // Init all carriers bitloading to QAM4096
    std::priority_queue<carrier_ber_t> bitloadings_set;
    modulation_type m = QAM4096;
    int b = MODULATION_MAP[m].n_bits;
    int M = 1 << b;
    float A = 1 - 1 / sqrt(M);
    float P_bar_nom = 0;
    int P_bar_denom = 0;
    for (int i=0; i<NUMBER_OF_CARRIERS+1; i++) {
        if (d_broadcast_channel_response.mask[i]) {
            float erfc_result = std::erfc(sqrt(3 * d_snr[i] / (M - 1) / 2));
            float ser = 2 * A * erfc_result * (1 - A * erfc_result / 2); // calculate symbol error rate (SER) for the carrier
            P_bar_nom += ser;   // sum(P[i]*b[i]) 
            P_bar_denom += b;   // sum(b[i])
            carrier_ber_t carrier_ber;
            carrier_ber = std::make_pair(ser/b, i); // ser/b is the bit error rate
            bitloadings_set.push(carrier_ber);
            tone_map[i] = m;
        }
    }

    // Incremental algorithm
    while (P_bar_nom/P_bar_denom > P_t) {  // test if sum(P[i]*b[i])/sum(b[i]) > P_t)
        carrier_ber_t carrier_ber = bitloadings_set.top(); // get the bitloading of the worst carrier
        float ber = carrier_ber.first;
        int i = carrier_ber.second;
        float new_ser = 0;
        bitloadings_set.pop(); // remove it from the set
        m = tone_map[i];
        b = MODULATION_MAP[m].n_bits;
        P_bar_nom -= b * ber; // substract the removed SER from the sum
        P_bar_denom -= b;
        if (m != BPSK) {
            m = (modulation_type)((int)m - 1); // decrease its bitloading and calculate its new BER
            b = MODULATION_MAP[m].n_bits;
            M = 1 << b;
            switch (m) {
                case BPSK: {
                    float f = std::erfc(sqrt(d_snr[i])); 
                    new_ser = f / 2; 
                    break;
                }
                case QAM8:
                {
                    float f1 = std::erfc(sqrt(d_snr[i]) * MODULATION_MAP[m].scale); 
                    float f2 = std::erfc(sqrt(d_snr[i]) * MODULATION_MAP[m].scale * 1.29); 
                    new_ser = 3 / 4 * f1 * (1 - f2) + f2 / 2;
                    break;
                }
                case QPSK:
                case QAM16:
                case QAM64:
                case QAM256:
                case QAM1024:
                case QAM4096: {
                    float f = std::erfc(sqrt(d_snr[i])*MODULATION_MAP[m].scale); // MODULATION_MAP[m].scale = sqrt(3/(M-1)/2) 
                    A = 1 - 1 / sqrt(M);
                    new_ser = 2 * A * f * (1 - A * f / 2); // calculate symbol error rate (SER) for the carrier
                    break;
                }
                case NULLED:
                    break; // should never arrive here
            }
            carrier_ber_t carrier_ber(new_ser/b, i);
            bitloadings_set.push(carrier_ber); // add the bitloading back to the set
            P_bar_nom += new_ser; // add the new BER to the sum
            P_bar_denom += b;
            tone_map[i] = m;
        } else {
            tone_map[i] = NULLED;
        }
    }
    DEBUG_VECTOR(tone_map);
    return tone_map;
}

void phy_service::set_tone_map(tone_map_t tone_map) {
    d_tone_info.tone_map = tone_map;
    update_tone_info_capacity(d_tone_info);
}

void phy_service::set_code_rate(code_rate rate) {
    d_code_rate = rate;
}

int phy_service::get_mpdu_payload_size() {
    return d_frame_parameters.mpdu_payload_size/8;
}

int phy_service::get_ppdu_payload_length() {
    return d_frame_parameters.ppdu_payload_size;
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
                case 0: frame_parameters.pb_size = PB520; DEBUG_ECHO("pb_size_t = PB520"); break;
                case 1: frame_parameters.pb_size = PB136; DEBUG_ECHO("pb_size_t = PB136"); break;
            }

            int tmi = get_field(fc_bits, IEEE1901_FRAME_CONTROL_SOF_TMI_OFFSET, IEEE1901_FRAME_CONTROL_SOF_TMI_WIDTH);
            switch (tmi) {
                case 0: frame_parameters.robo_mode = STD_ROBO; DEBUG_ECHO("Robo Mode = STD_ROBO"); break;
                case 1: frame_parameters.robo_mode = HS_ROBO; DEBUG_ECHO("Robo Mode = HS_ROBO"); break;
                case 2: frame_parameters.robo_mode = MINI_ROBO; DEBUG_ECHO("Robo Mode = MINI_ROBO"); break;
                default: frame_parameters.robo_mode = NO_ROBO; frame_parameters.tone_info = d_tone_info; break;
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
            frame_parameters.has_payload = false;           
            break;
        }
        
        // Sound frame control
        case 4: {
            frame_parameters.type = MPDU_TYPE_SOUND;
            DEBUG_VAR(frame_parameters.type);

            int pbsz = get_field(fc_bits, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_WIDTH);
            switch (pbsz) {
                case 0: frame_parameters.pb_size = PB520; frame_parameters.robo_mode = STD_ROBO; DEBUG_ECHO("pb_size_t = PB520"); break;
                case 1: frame_parameters.pb_size = PB136; frame_parameters.robo_mode = MINI_ROBO; DEBUG_ECHO("pb_size_t = PB136"); break;
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
            frame_parameters.tone_info = calc_tone_info(frame_parameters.robo_mode);
        frame_parameters.rate = d_code_rate;
        frame_parameters.encoded_block_size = calc_encoded_block_size(frame_parameters.rate, frame_parameters.pb_size);
        frame_parameters.interleaved_block_size = calc_interleaved_block_size(frame_parameters.robo_mode, frame_parameters.rate, frame_parameters.pb_size);
        int number_of_blocks = (frame_parameters.n_expected_symbols * frame_parameters.tone_info.capacity) / frame_parameters.interleaved_block_size;
        frame_parameters.mpdu_payload_size =  number_of_blocks * calc_block_size(frame_parameters.pb_size);
        frame_parameters.ppdu_payload_size = frame_parameters.n_expected_symbols * (NUMBER_OF_CARRIERS * 2 + GUARD_INTERVAL_PAYLOAD);
    } else {
        frame_parameters.ppdu_payload_size = 0;
    }

    return true;
}

vector_int phy_service::resolve_frame_control_symbol (const vector_float& fc_data) {
    vector_float fc_descaled(fc_data);

    vector_complex symbol = fft_real(fc_data.begin(), fc_data.end());

    // Demodulate
    vector_float demodulated = symbol_demodulate(symbol.begin(), BROADCAST_CARRIERS, d_broadcast_channel_response);
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
    for (int i=0; i<bit_width; i++)  {
        int j = bit_offset + i;
        int calc_offset = 8 * (j/8) + 7 - j % 8;
        value |= bit_vector[calc_offset] << i;
    }
    return value;
}

vector_float phy_service::symbol_demodulate (vector_complex::const_iterator iter, const phy_service::tone_info_t& tone_info, const channel_response &channel_response) {
    vector_float soft_bits(tone_info.capacity);
    vector_float::iterator soft_bits_iter = soft_bits.begin();
    for (int i=0; i<=NUMBER_OF_CARRIERS; i++) { 
        if (tone_info.tone_map[i] != NULLED) {  // If carrier is ON
            complex r = *iter / channel_response.carriers[i] / (float)NUMBER_OF_CARRIERS;
            complex p = ANGLE_NUMBER_TO_VALUE[CARRIERS_ANGLE_NUMBER[i]*2]; // Convert the angle number to its value
            complex mapped_value(r.real() * p.real() + r.imag() * p.imag(), 
                                             -r.real() * p.imag() + r.imag() * p.real()); // Rotate channel value by minus angel_number to get the original mapped value
            soft_bits_iter = demodulate(mapped_value, tone_info.tone_map[i], 2 * d_noise_psd[i] / channel_response.carriers_gain[i] / (float)NUMBER_OF_CARRIERS / channel_response.carriers_gain[i] / (float)NUMBER_OF_CARRIERS, soft_bits_iter);
        }
        iter++;
    }
    DEBUG_VECTOR(soft_bits);
    return soft_bits;
}


// Calculate number of bits per symbol
void phy_service::update_tone_info_capacity(tone_info_t& tone_info) {
    tone_info.capacity = 0;
    for (unsigned int i = 0; i< NUMBER_OF_CARRIERS+1; i++)
        tone_info.capacity += MODULATION_MAP[tone_info.tone_map[i]].n_bits;
}

vector_float::iterator phy_service::demodulate(const complex &value, modulation_type modulation, float n0, vector_float::iterator iter) {
    modulation_map_t modulation_map = MODULATION_MAP[modulation];
    switch (modulation) {
        case NULLED: {
            break;
        }
        case BPSK: {
            *iter++ = -4 * std::real(value) * modulation_map.scale / n0;        
            break;
        }
        case QPSK: {
            *iter++ = -4 * std::real(value) * modulation_map.scale / n0;
            *iter++ = -4 * std::imag(value) * modulation_map.scale / n0;
            break;
        }
        case QAM8: {
            iter = demodulate_helper(modulation_map.n_bits-1, std::real(value), modulation_map.scale, n0, iter);
            *iter++ = -4 * std::imag(value) * 1.29 * modulation_map.scale / n0;
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
            // itpp::vec b = qam16.demodulate_soft_bits(s, n0, itpp::APPROX);
            // for (int i = b.size() - 1; i >= 0; i--) {
            //     *iter = b(i);
            //     iter++;
            // }
            // std::cout << "s: " << s << ", b: " << b << std::endl;
            iter = demodulate_helper(modulation_map.n_bits/2, std::real(value), modulation_map.scale, n0, iter);
            iter = demodulate_helper(modulation_map.n_bits/2, std::imag(value), modulation_map.scale, n0, iter);
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

vector_float phy_service::channel_deinterleaver(const vector_float& bitstream, vector_float& parity_bitstream, pb_size_t pb_size, code_rate rate) {
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

vector_float phy_service::robo_deinterleaver(const vector_float& bitstream, int n_raw, robo_mode_t robo_mode)  { 
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

int phy_service::calc_interleaved_block_size(robo_mode_t robo_mode, code_rate rate, pb_size_t pb_size) {
    int encoded_pb_n_bits = calc_encoded_block_size(rate, pb_size);

    // Determine parameters for ROBO mode
    unsigned int n_copies = 1, n_pad = 0, bits_in_last_symbol, bits_in_segment; // Default values when not in ROBO mode
    if (robo_mode != NO_ROBO) {
        calc_robo_parameters (robo_mode, encoded_pb_n_bits, n_copies, bits_in_last_symbol, bits_in_segment, n_pad);
        return (encoded_pb_n_bits + n_pad) * n_copies;
    }
    return encoded_pb_n_bits;
}

inline int phy_service::calc_block_size(pb_size_t pb_size) {
    if (pb_size == PB520) 
        return 520*8;
    else if (pb_size == PB136)
        return 136*8;
    else return 16*8;
}
int phy_service::calc_encoded_block_size(code_rate rate, pb_size_t pb_size) {
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

int phy_service::max_blocks (robo_mode_t robo_mode, code_rate rate, modulation_type modulation) {  
    int encoded_pb_n_bits = calc_interleaved_block_size(robo_mode, rate, PB520);

    float symbol_durarion = NUMBER_OF_CARRIERS * 2.0 / SAMPLE_RATE; // one symbol duration (microseconds)
    float max_frame_duration = (((1 << IEEE1901_FRAME_CONTROL_SOF_FL_WIDTH) - 1) * 1.28 - IEEE1901_RIFS_DEFAULT); // maximum of all symbols duration allowed (microseconds)
    int max_n_symbols = max_frame_duration / symbol_durarion; // maximum number of symbols
    int max_n_bits = max_n_symbols * build_broadcast_tone_map(modulation).capacity; // maximum number of bits
    return max_n_bits / encoded_pb_n_bits; // maximum number of PB520 blocks
}

phy_service::tone_info_t phy_service::build_broadcast_tone_map(modulation_type modulation) {
    tone_info_t broadcast_carriers;
    for (size_t i=0; i<CARRIERS_BROADCAST_MASK.size(); i++)
        broadcast_carriers.tone_map[i] = CARRIERS_BROADCAST_MASK[i] ? modulation : NULLED;
    update_tone_info_capacity(broadcast_carriers);
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

void phy_service::process_ppdu_preamble(vector_float::const_iterator iter, vector_float::const_iterator iter_end) {
    assert (iter_end - iter == PREAMBLE_SIZE);
    DEBUG_VECTOR(SYNCP_FREQ);
    iter += SYNCP_SIZE / 2 + SYNCP_SIZE;
    vector_float syncp(iter, iter + SYNCP_SIZE);
    DEBUG_VECTOR(syncp);
    vector_complex syncp_freq = fft_real_syncp(syncp);
    DEBUG_VECTOR(syncp_freq);
    estimate_channel_phase(syncp_freq.begin(), syncp_freq.end(), SYNCP_FREQ.begin(), SYNCP_CARRIERS_MASK, d_broadcast_channel_response);
    DEBUG_VECTOR(d_broadcast_channel_response.carriers);
}

void phy_service::estimate_channel_gain(vector_symbol_freq::const_iterator iter, vector_symbol_freq::const_iterator iter_end, vector_symbol_freq::const_iterator ref_iter, channel_response &channel_response) {
    int nsymbols = iter_end - iter;
    if (nsymbols) 
        channel_response.carriers_gain.fill(0); // zero the carriers_gain array
    while (iter != iter_end) {
        assert (iter->end() - iter->begin() == (unsigned int)channel_response.carriers_gain.size());
        int i = 0;
        vector_symbol_freq::value_type::const_iterator carrier_iter = iter->begin();
        vector_symbol_freq::value_type::const_iterator carrier_ref_iter = ref_iter->begin();
        while (carrier_iter != iter->end()) { 
            if (channel_response.mask[i])  // If carrier is ON
                channel_response.carriers_gain[i] += std::abs(*carrier_iter / *carrier_ref_iter) / nsymbols; // Calculate the average carrier response
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

    std::vector<spline_set_t> spline_set = spline(x,y);
    for (unsigned int i = 0, j = 0; i < channel_response.carriers.size(); i++) {
        if (channel_response.mask[i]) {
            while (j < spline_set.size()-1 && spline_set[j+1].x <= i) j++;
            float dx = i - spline_set[j].x;
            channel_response.carriers[i] = channel_response.carriers_gain[i] * std::exp(complex(0,spline_set[j].a + spline_set[j].b * dx + spline_set[j].c * dx * dx +
                                                      spline_set[j].d * dx * dx * dx));
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

std::vector<phy_service::spline_set_t> phy_service::spline(vector_float &x, vector_float &y)
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

    std::vector<spline_set_t> output_set(n);
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