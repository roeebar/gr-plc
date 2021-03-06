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
 
#include "phy_service.h"
#include "debug.h"
#include <iostream>
#include <algorithm>
#include <functional>
#include <cmath>
#include <numeric>
#include <itpp/comm/modulator.h>
#include <queue>

namespace light_plc {

#include "mapping.inc"

const int phy_service::CHANNEL_INTERLEAVER_OFFSET[3][3] = {IEEE1901_CHANNEL_INTERLEAVER_OFFSET};
const int phy_service::CHANNEL_INTERLEAVER_STEPSIZE[3][3] = {IEEE1901_CHANNEL_INTERLEAVER_STEPSIZE};
static_assert(RATE_1_2==0 && RATE_16_21==1 && RATE_16_18==2,"Interleaving parameters error");
static_assert(PB16==0 && PB136==1 && PB520==2,"Interleaving parameters error");

const int phy_service::CARRIERS_ANGLE_NUMBER[NUMBER_OF_CARRIERS] = {IEEE1901_CARRIERS_ANGLE_NUMBER};

const phy_service::modulation_map_t phy_service::MODULATION_MAP[9] = {
                         {0,NULL,0},
                         {MAP_BPSK_NBITS, MAP_BPSK, MAP_BPSK_SCALE},
                         {MAP_QPSK_NBITS, MAP_QPSK, MAP_QPSK_SCALE},
                         {MAP_QAM8_NBITS, MAP_QAM8, MAP_QAM8_SCALE},
                         {MAP_QAM16_NBITS, MAP_QAM16, MAP_QAM16_SCALE},
                         {MAP_QAM64_NBITS, MAP_QAM64, MAP_QAM64_SCALE},
                         {MAP_QAM256_NBITS, MAP_QAM256, MAP_QAM256_SCALE},
                         {MAP_QAM1024_NBITS, MAP_QAM1024, MAP_QAM1024_SCALE},
                         {MAP_QAM4096_NBITS, MAP_QAM4096, MAP_QAM4096_SCALE}
                     };

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
                                                        complex(0.923879532511287,-0.382683432365090)
                                                    };

std::mutex phy_service::fftw_mtx;

phy_service::phy_service (tone_mask_t tone_mask, tone_mask_t broadcast_tone_mask, sync_tone_mask_t sync_tone_mask, channel_est_t channel_est, bool debug) {
    static_assert(MT_BPSK==1 && MT_QPSK==2 && MT_QAM8==3 && MT_QAM16==4 && MT_QAM64==5 && MT_QAM256==6 && MT_QAM1024==7 && MT_QAM4096==8, "Mapping parameters error");
    stats = stats_t();
    create_fftw_vars();
    init_turbo_codec();
    d_debug = debug;
    TONE_MASK = tone_mask;
    DEBUG_VECTOR(TONE_MASK);
    BROADCAST_TONE_MASK = broadcast_tone_mask;
    DEBUG_VECTOR(BROADCAST_TONE_MASK);
    assert(*(BROADCAST_TONE_MASK.end()-1) == 0 && (*BROADCAST_TONE_MASK.begin()==0)); // cannot use the first and last carriers
    N_BROADCAST_TONES = count_non_masked_carriers(BROADCAST_TONE_MASK.begin(), BROADCAST_TONE_MASK.end());
    DEBUG_VAR(N_BROADCAST_TONES);
    SYNC_TONE_MASK = sync_tone_mask;
    DEBUG_VECTOR(SYNC_TONE_MASK);
    for (size_t i=0; i<NUMBER_OF_CARRIERS; i++) SYNC_TONE_MASK_EXPANDED[i] = SYNC_TONE_MASK[i / (NUMBER_OF_CARRIERS / N_SYNC_CARRIERS)] && (i % (NUMBER_OF_CARRIERS / N_SYNC_CARRIERS) == 0);
    N_SYNC_ACTIVE_TONES = count_non_masked_carriers(SYNC_TONE_MASK.begin(), SYNC_TONE_MASK.end());
    DEBUG_VAR(N_SYNC_ACTIVE_TONES);
    BROADCAST_QPSK_TONE_INFO = build_broadcast_tone_info();
    DEBUG_VAR(BROADCAST_QPSK_TONE_INFO.capacity);
    TONE_INFO_STD_ROBO = calc_robo_tone_info(TM_STD_ROBO);
    TONE_INFO_MINI_ROBO = calc_robo_tone_info(TM_MINI_ROBO);
    TONE_INFO_HS_ROBO = calc_robo_tone_info(TM_HS_ROBO);
    calc_preamble(PREAMBLE, SYNCP_FREQ);
    TURBO_INTERLEAVER_SEQUENCE = calc_turbo_interleaver_sequence();
    d_channel_est_mode = channel_est;
    DEBUG_VAR(d_channel_est_mode);
    d_custom_tone_info = build_broadcast_tone_info();
    d_channel_response.mask = BROADCAST_TONE_MASK;
    d_channel_response.n_carriers = N_BROADCAST_TONES;
    d_channel_response.carriers.fill(complex(1, 0));
    d_noise_psd.fill(0);
}

phy_service::phy_service (bool debug): phy_service({IEEE1901_DEFAULT_TONE_MASK}, {IEEE1901_DEFAULT_TONE_MASK}, {IEEE1901_SYNCP_TONE_MASK}, CE_SOUND, debug){};

phy_service::phy_service (const phy_service &obj) :
    d_debug(obj.d_debug),
    TONE_MASK(obj.TONE_MASK),
    BROADCAST_TONE_MASK(obj.BROADCAST_TONE_MASK),
    N_BROADCAST_TONES(obj.N_BROADCAST_TONES),
    SYNC_TONE_MASK(obj.SYNC_TONE_MASK),
    N_SYNC_ACTIVE_TONES(obj.N_SYNC_ACTIVE_TONES),
    SYNC_TONE_MASK_EXPANDED(obj.SYNC_TONE_MASK_EXPANDED),
    BROADCAST_QPSK_TONE_INFO(obj.BROADCAST_QPSK_TONE_INFO),
    TONE_INFO_STD_ROBO(obj.TONE_INFO_STD_ROBO),
    TONE_INFO_MINI_ROBO(obj.TONE_INFO_MINI_ROBO),
    TONE_INFO_HS_ROBO(obj.TONE_INFO_HS_ROBO),
    PREAMBLE(obj.PREAMBLE),
    SYNCP_FREQ(obj.SYNCP_FREQ),
    TURBO_INTERLEAVER_SEQUENCE(obj.TURBO_INTERLEAVER_SEQUENCE),
    stats(obj.stats),
    d_channel_est_mode(obj.d_channel_est_mode),
    d_custom_tone_info(obj.d_custom_tone_info),
    d_qpsk_tone_mask(obj.d_qpsk_tone_mask),
    d_channel_response(obj.d_channel_response),
    d_noise_psd(obj.d_noise_psd),
    d_rx_params(obj.d_rx_params),
    d_rx_payload_symbols_freq(obj.d_rx_payload_symbols_freq),
    d_rx_soft_bits(obj.d_rx_soft_bits),
    d_rx_mpdu_payload(obj.d_rx_mpdu_payload)
{
    create_fftw_vars();
    init_turbo_codec();
}

phy_service& phy_service::operator=(const phy_service& rhs) {
    phy_service tmp(rhs);
    std::swap(d_debug, tmp.d_debug);
    std::swap(TONE_MASK, tmp.TONE_MASK);
    std::swap(BROADCAST_TONE_MASK, tmp.BROADCAST_TONE_MASK);
    std::swap(N_BROADCAST_TONES, tmp.N_BROADCAST_TONES);
    std::swap(SYNC_TONE_MASK, tmp.SYNC_TONE_MASK);
    std::swap(N_SYNC_ACTIVE_TONES, tmp.N_SYNC_ACTIVE_TONES);
    std::swap(SYNC_TONE_MASK_EXPANDED, tmp.SYNC_TONE_MASK_EXPANDED);
    std::swap(BROADCAST_QPSK_TONE_INFO, tmp.BROADCAST_QPSK_TONE_INFO);
    std::swap(TONE_INFO_STD_ROBO, tmp.TONE_INFO_STD_ROBO);
    std::swap(TONE_INFO_MINI_ROBO, tmp.TONE_INFO_MINI_ROBO);
    std::swap(TONE_INFO_HS_ROBO, tmp.TONE_INFO_HS_ROBO);
    std::swap(PREAMBLE, tmp.PREAMBLE);
    std::swap(SYNCP_FREQ, tmp.SYNCP_FREQ);
    std::swap(TURBO_INTERLEAVER_SEQUENCE, tmp.TURBO_INTERLEAVER_SEQUENCE);
    std::swap(d_channel_est_mode, tmp.d_channel_est_mode);
    std::swap(d_custom_tone_info, tmp.d_custom_tone_info);
    std::swap(d_qpsk_tone_mask, tmp.d_qpsk_tone_mask);
    std::swap(d_channel_response, tmp.d_channel_response);
    std::swap(d_noise_psd, tmp.d_noise_psd);
    std::swap(stats, tmp.stats);
    std::swap(d_rx_params, tmp.d_rx_params);
    std::swap(d_rx_payload_symbols_freq, tmp.d_rx_payload_symbols_freq);
    std::swap(d_rx_soft_bits, tmp.d_rx_soft_bits);
    std::swap(d_rx_mpdu_payload, tmp.d_rx_mpdu_payload);
    std::swap(d_ifft_input, tmp.d_ifft_input);
    std::swap(d_ifft_output, tmp.d_ifft_output);
    std::swap(d_fftw_rev_plan, tmp.d_fftw_rev_plan);
    std::swap(d_fft_input, tmp.d_fft_input);
    std::swap(d_fft_output, tmp.d_fft_output);
    std::swap(d_fftw_fwd_plan, tmp.d_fftw_fwd_plan);
    std::swap(d_fft_syncp_input, tmp.d_fft_syncp_input);
    std::swap(d_fft_syncp_output, tmp.d_fft_syncp_output);
    std::swap(d_fftw_syncp_fwd_plan, tmp.d_fftw_syncp_fwd_plan);
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

vector_complex phy_service::create_ppdu(const unsigned char *mpdu_fc_bin, size_t mpdu_fc_len, const unsigned char *mpdu_payload_bin, size_t mpdu_payload_len) {
    vector_int mpdu_payload_int(unpack_into_bitvector(mpdu_payload_bin, mpdu_payload_len));
    vector_int mpdu_fc_int(unpack_into_bitvector(mpdu_fc_bin, mpdu_fc_len));
    return create_ppdu(mpdu_fc_int, mpdu_payload_int);
}

vector_complex phy_service::create_ppdu(vector_int &mpdu_fc_int, const vector_int &mpdu_payload_int) {
    assert(mpdu_fc_int.size() == FRAME_CONTROL_NBITS);
    tx_params_t tx_params = get_tx_params(mpdu_fc_int);
    update_frame_control(mpdu_fc_int, tx_params, mpdu_payload_int.size());

    // Encode frame control
    DEBUG_ECHO("Encoding frame control...")
    DEBUG_VECTOR(mpdu_fc_int);
    vector_complex fc_symbols = create_frame_control_symbol(mpdu_fc_int);

    // Encode payload blocks
    vector_complex payload_symbols;
    if (mpdu_payload_int.size()) {
        DEBUG_ECHO("Encoding payload blocks...")
        DEBUG_VECTOR(mpdu_payload_int);
        payload_symbols = create_payload_symbols(mpdu_payload_int, tx_params.pb_size, tx_params.tone_mode);
    }

    DEBUG_ECHO("Creating final data stream...")

    // Calculate final size
    vector_complex datastream(
        PREAMBLE.size() - ROLLOFF_INTERVAL + // Premable size
        NUMBER_OF_CARRIERS + IEEE1901_GUARD_INTERVAL_FC  + // frame control size
        payload_symbols.size() / NUMBER_OF_CARRIERS * (NUMBER_OF_CARRIERS + IEEE1901_GUARD_INTERVAL_PAYLOAD) + ROLLOFF_INTERVAL // payload size
    );

    // Append preamble, frame control and payload to the datastream
    vector_complex::iterator datastream_iter = datastream.begin();
    datastream_iter = append_datastream(PREAMBLE.begin(), PREAMBLE.end(), datastream_iter, 0, IEEE1901_SCALE_FACTOR_PREAMBLE);

    // Frame control and payload scale factor is divided by N to compensate for the ifft which implicitly multiply by N
    datastream_iter = append_datastream(fc_symbols.begin(), fc_symbols.begin() + NUMBER_OF_CARRIERS, datastream_iter - ROLLOFF_INTERVAL, IEEE1901_GUARD_INTERVAL_FC + ROLLOFF_INTERVAL, IEEE1901_SCALE_FACTOR_FC / NUMBER_OF_CARRIERS);
    vector_complex::const_iterator payload_symbols_iter = payload_symbols.begin();
    while (payload_symbols_iter != payload_symbols.end()) {
        datastream_iter = append_datastream(payload_symbols_iter, payload_symbols_iter + NUMBER_OF_CARRIERS, datastream_iter - ROLLOFF_INTERVAL, IEEE1901_GUARD_INTERVAL_PAYLOAD + ROLLOFF_INTERVAL, IEEE1901_SCALE_FACTOR_PAYLOAD / NUMBER_OF_CARRIERS);
        payload_symbols_iter += NUMBER_OF_CARRIERS;
    }

    DEBUG_VECTOR(datastream);
    return datastream;
}

phy_service::tx_params_t phy_service::get_tx_params (const vector_int &mpdu_fc_int) {
    tx_params_t tx_params;
    delimiter_type_t dt = (delimiter_type_t)get_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_DT_IH_OFFSET, IEEE1901_FRAME_CONTROL_DT_IH_WIDTH);
    switch (dt) {
        case DT_SOF: {
            int tmi = get_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_SOF_TMI_OFFSET, IEEE1901_FRAME_CONTROL_SOF_TMI_WIDTH);
            switch (tmi) {
                case 0: tx_params.tone_mode = TM_STD_ROBO; break;
                case 1: tx_params.tone_mode = TM_HS_ROBO; break;
                case 2: tx_params.tone_mode = TM_MINI_ROBO; break;
                default: tx_params.tone_mode = TM_NO_ROBO; break;
            }
            int pbsz = get_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_SOF_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOF_PBSZ_WIDTH);
            switch (pbsz) {
                case 0: tx_params.pb_size = PB520; break;
                case 1: tx_params.pb_size = PB136; break;
            }
            break;
        }
        case DT_SOUND: {
            int pbsz = get_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_WIDTH);
            switch (pbsz) {
                case 0: tx_params.pb_size = PB520; tx_params.tone_mode = TM_STD_ROBO; break;
                case 1: tx_params.pb_size = PB136; tx_params.tone_mode = TM_MINI_ROBO; break;
            }
            break;
        }
        case DT_SACK:
            break;

        default:
            break;
    }
    return tx_params;
}

void phy_service::update_frame_control (vector_int &mpdu_fc_int, tx_params_t tx_params, size_t payload_size) {
    delimiter_type_t dt = (delimiter_type_t)get_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_DT_IH_OFFSET, IEEE1901_FRAME_CONTROL_DT_IH_WIDTH);
    int fl_width = 0;
    if (dt == DT_SOF || dt == DT_SOUND) {
        // Calculate frame length
        tone_info_t tone_info = get_tone_info(tx_params.tone_mode);
        int n_blocks = payload_size / calc_phy_block_size(tx_params.pb_size);
        int fec_block_size = calc_fec_block_size(tx_params.tone_mode, tone_info.rate, tx_params.pb_size);
        int n_bits = n_blocks * fec_block_size;
        int n_symbols = (n_bits % tone_info.capacity) ? n_bits / tone_info.capacity + 1 : n_bits / tone_info.capacity;
        float symbol_durarion = (NUMBER_OF_CARRIERS + (float)IEEE1901_GUARD_INTERVAL_PAYLOAD) / SAMPLE_RATE; // one symbol duration (microseconds)
        fl_width = std::ceil((n_symbols * symbol_durarion + (float)IEEE1901_ROLLOFF_INTERVAL / SAMPLE_RATE + IEEE1901_RIFS_DEFAULT) / 1.28);
    }
    switch (dt) {
        case DT_SOF:
            set_field(mpdu_fc_int, IEEE1901_FRAME_CONTROL_SOF_FL_OFFSET, IEEE1901_FRAME_CONTROL_SOF_FL_WIDTH, fl_width);
            break;
        case DT_SOUND:
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

void phy_service::pack_bitvector(vector_int::const_iterator iter, vector_int::const_iterator end, unsigned char* array) {
    assert ((end-iter) % 8 == 0);
    int i = 0;
    while (iter!=end) {
        unsigned char byte = 0;
        for (int offset=0; offset<8; offset++)
            byte |= *iter++ << offset;
        array[i++] = byte;
    }
}

vector_int phy_service::unpack_into_bitvector (const unsigned char *data, size_t c) {
    vector_int bit_vector(c*8, 0);
    for (unsigned int i=0; i<c; i++)
    {
        unsigned char byte = data[i];
        int offset = 0;
        while (byte)
        {
            bit_vector[i*8+offset++] = byte & 1;
            byte >>= 1;
        }
    }
    return bit_vector;
}

unsigned long phy_service::crc24(const vector_int &bit_vector) {
    //800FE3

    //crc24 table for polynom=800063
    static const unsigned long crc24tab[256] = {
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
        for (int offset=0; offset<8; offset++)
            cp |= (*iter++) << offset;
        crc = ((crc << 8) & 0xffff00) ^ crc24tab[((crc >> 16) & 0xff) ^ cp];
    }

    return (crc ^ 0xffffff);
}

vector_int phy_service::encode_payload(const vector_int &payload_bits, pb_size_t pb_size, code_rate_t rate, tone_mode_t tone_mode) {
    // Determine number of blocks and blocks size
    int block_n_bits = (pb_size == PB520) ? 520*8 : 136*8;
    assert (pb_size == PB520 || pb_size == PB136); // Cannot have payload blocks of 16 octets

    int n_blocks = payload_bits.size() / block_n_bits;
    assert ((payload_bits.size() % block_n_bits) == 0);

    int block_size = calc_fec_block_size(tone_mode, rate, pb_size);
    // Encode blocks
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

        if (tone_mode == TM_STD_ROBO || tone_mode == TM_MINI_ROBO || tone_mode == TM_HS_ROBO) {
            interleaved = robo_interleaver(interleaved, tone_mode);
            DEBUG_VECTOR(interleaved);
        }

        iter_out = std::copy(interleaved.begin(), interleaved.end(), iter_out);
        iter_in += block_n_bits;
    }
    return encoded_payload_bits;
}

vector_complex phy_service::create_payload_symbols(const vector_int &payload_bits, pb_size_t pb_size, tone_mode_t tone_mode) {
    tone_info_t tone_info = get_tone_info(tone_mode);

    // Encode and interleave
    vector_int encoded_payload_bits = encode_payload(payload_bits, pb_size, tone_info.rate, tone_mode);

    // Mapping and split to symbols
    vector_complex symbols_freq = modulate(encoded_payload_bits, tone_info);

    // Perform IFFT to get a time domain symbol
    vector_complex::const_iterator symbols_freq_iter = symbols_freq.begin();
    vector_complex symbols(symbols_freq.size());
    vector_complex::iterator symbols_iter = symbols.begin();
    while (symbols_freq_iter != symbols_freq.end()) {
        symbols_iter = ifft(symbols_freq_iter, symbols_freq_iter + NUMBER_OF_CARRIERS, symbols_iter);
        symbols_freq_iter += NUMBER_OF_CARRIERS;
        DEBUG_VECTOR_RANGE("symbols_freq", symbols_freq_iter - NUMBER_OF_CARRIERS, symbols_freq_iter);
        DEBUG_VECTOR_RANGE("symbols", symbols_iter - NUMBER_OF_CARRIERS , symbols_iter);
    }

    return symbols;
}

vector_complex phy_service::create_frame_control_symbol(const vector_int &frame_control_bits) {
    // Encode frame control
    vector_int encoded_frame_control = encode_frame_control(frame_control_bits);

    // Mapping and split to symbols
    vector_complex frame_control_symbol_freq = modulate(encoded_frame_control, BROADCAST_QPSK_TONE_INFO);
    DEBUG_VECTOR(frame_control_symbol_freq);

    // Perform IFFT to get a time domain symbol
    vector_complex frame_control_symbol(NUMBER_OF_CARRIERS);
    ifft(frame_control_symbol_freq.begin(), frame_control_symbol_freq.end(), frame_control_symbol.begin());
    DEBUG_VECTOR(frame_control_symbol);

    return frame_control_symbol;
}

vector_int phy_service::encode_frame_control(const vector_int &frame_control_bits) {
    // Turbo-convolution encoder
    vector_int parity = tc_encoder(frame_control_bits, PB16, RATE_1_2);
    DEBUG_VECTOR(parity);

    // Channel interleaver
    vector_int interleaved = channel_interleaver(frame_control_bits, parity, PB16, RATE_1_2);
    DEBUG_VECTOR(interleaved);

    vector_int copied = copier(interleaved, N_BROADCAST_TONES, 128 + 12/2); // +12/2 because of non-standard turbo encoder...
    DEBUG_VECTOR(copied);

    return copied;
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
    d_turbo_codec.set_parameters(gen, gen, 4, interleaver_sequence_bvec, puncture_matrix, 4, "LOGMAX", 1.0, true, itpp::LLR_calc_unit());
}

vector_int phy_service::tc_encoder(const vector_int &bitstream, pb_size_t pb_size, code_rate_t rate) {
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
}

vector_int phy_service::tc_decoder(const vector_float &received_info, const vector_float &received_parity, pb_size_t pb_size, code_rate_t rate) {
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

std::array<vector_int, 3>  phy_service::calc_turbo_interleaver_sequence(){
    static const int TCENCODER_SEED_PB16[IEEE1901_TCENCODER_SEED_PB16_N] = {IEEE1901_TCENCODER_SEED_PB16};
    static const int TCENCODER_SEED_PB136[IEEE1901_TCENCODER_SEED_PB136_N] = {IEEE1901_TCENCODER_SEED_PB136};
    static const int TCENCODER_SEED_PB520[IEEE1901_TCENCODER_SEED_PB520_N] = {IEEE1901_TCENCODER_SEED_PB520};
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

vector_int phy_service::channel_interleaver(const vector_int& bitstream, const vector_int& parity_bitstream, pb_size_t pb_size, code_rate_t rate) {
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

vector_int phy_service::robo_interleaver(const vector_int& bitstream, tone_mode_t tone_mode) {
    // Determine number of bits to pad at end of copy
    unsigned int n_raw = bitstream.size();
    unsigned int n_copies, bits_in_last_symbol, bits_in_segment, n_pad;
    calc_robo_parameters (tone_mode, n_raw, n_copies, bits_in_last_symbol, bits_in_segment, n_pad);

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

phy_service::tone_info_t phy_service::get_tone_info (tone_mode_t tone_mode) {
    switch (tone_mode) {
        case TM_STD_ROBO: return TONE_INFO_STD_ROBO; break;
        case TM_HS_ROBO: return TONE_INFO_HS_ROBO; break;
        case TM_MINI_ROBO: return TONE_INFO_MINI_ROBO; break;
        default: return d_custom_tone_info; break;
    }
}

phy_service::tone_info_t phy_service::calc_robo_tone_info (tone_mode_t tone_mode) {
    assert (tone_mode == TM_STD_ROBO || tone_mode == TM_MINI_ROBO || tone_mode == TM_HS_ROBO);

    // Create basic ROBO carriers consists of broadcast mask and QPSK
    tone_info_t tone_info = BROADCAST_QPSK_TONE_INFO;

    int n_copies = 0;

    // Each mode replicates the bits n_copies times
    switch (tone_mode) {
        case TM_STD_ROBO: n_copies = 4; break;
        case TM_HS_ROBO: n_copies = 2; break;
        case TM_MINI_ROBO: n_copies = 5; break;
        case TM_NO_ROBO: break;
    }

    unsigned int n_carriers = N_BROADCAST_TONES;
    unsigned int n_carriers_robo = n_copies * (n_carriers / n_copies);
    // Mark the unused carriers as masked
    unsigned int j = 0, i = tone_info.tone_map.size()-1;
    while (j<(n_carriers-n_carriers_robo)) {
        if (tone_info.tone_map[i] != MT_NULLED ) {
            tone_info.tone_map[i] = MT_NULLED;
            j++;
        }
        i--;
    }
    update_tone_info_capacity(tone_info);
    return tone_info;
}

void phy_service::calc_robo_parameters (tone_mode_t tone_mode, unsigned int n_raw, unsigned int &n_copies, unsigned int &bits_in_last_symbol, unsigned int &bits_in_segment, unsigned int &n_pad) {
    // Each mode replicates the bits n_copies times
    switch (tone_mode) {
        case TM_STD_ROBO: n_copies = 4; break;
        case TM_HS_ROBO: n_copies = 2; break;
        case TM_MINI_ROBO: n_copies = 5; break;
        case TM_NO_ROBO: break;
    }
    assert (tone_mode == TM_STD_ROBO || tone_mode == TM_HS_ROBO || tone_mode == TM_MINI_ROBO); // ROBO mode should be only one of these

    unsigned int n_carriers = N_BROADCAST_TONES;
    unsigned int n_carriers_robo = n_copies * (n_carriers / n_copies);

    // Determine number of bits to pad at end of copy
    unsigned int n_carriers_in_segment = n_carriers_robo / n_copies;
    int bits_per_symbol = MODULATION_MAP[MT_QPSK].n_bits * n_carriers_robo;
    bits_in_segment = MODULATION_MAP[MT_QPSK].n_bits * n_carriers_in_segment;
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

vector_complex phy_service::modulate(const vector_int& bits, const phy_service::tone_info_t& tone_info) {
    // Calculate number of symbols needed
    int n_symbols = (bits.size() && (bits.size() % tone_info.capacity)) ? bits.size() / tone_info.capacity + 1 : bits.size() / tone_info.capacity;
    vector_complex symbols_freq(n_symbols * NUMBER_OF_CARRIERS);
    vector_complex::iterator symbols_freq_iter = symbols_freq.begin();
    // Perform mapping
    vector_int::const_iterator it = bits.cbegin();
    int pn_state = pn_generator_init();
    for (int j = 0; j < n_symbols; j++) {
        for (int i=0; i<NUMBER_OF_CARRIERS; i++, symbols_freq_iter++) {
            if (!TONE_MASK[i]) // if regulations do not allow the carrier to transmit
                continue;
            else if (tone_info.tone_map[i] != MT_NULLED) {  // If carrier is ON
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
                // Multiplying by N so in time domain this will produce cos() with unit amplitude
                complex m = modulation_map.map[decimal] * modulation_map.scale * (float)NUMBER_OF_CARRIERS;
                 // Rotate the mapped value using the angle number and add it to the mapped values vector
                *symbols_freq_iter = complex(m.real() * p.real() - m.imag() * p.imag(),
                                             m.real() * p.imag() + m.imag() * p.real());
            } else {  // If carrier is OFF use random bit with BPSK modulation
                *symbols_freq_iter = MODULATION_MAP[MT_BPSK].map[pn_generator(1, pn_state)] * MODULATION_MAP[MT_BPSK].scale * (float)NUMBER_OF_CARRIERS;
            }
        } // Repeat until entire symbol is filled
    } // Keep add symbols until all bits are mapped

    return symbols_freq;
}

vector_complex::iterator phy_service::ifft(vector_complex::const_iterator iter_begin, vector_complex::const_iterator iter_end, vector_complex::iterator iter_out) {
    assert (iter_end - iter_begin == NUMBER_OF_CARRIERS);
    // Wrap the carrier by N/2, so N/2 carrier becomes first and so on...
    std::copy(iter_begin + NUMBER_OF_CARRIERS/2, iter_end, (complex*)d_ifft_input);
    std::copy(iter_begin, iter_begin + NUMBER_OF_CARRIERS/2, (complex*)d_ifft_input + NUMBER_OF_CARRIERS/2);
    fftwf_execute(d_fftw_rev_plan);
    iter_out = std::copy ((complex*)d_ifft_output, (complex*)d_ifft_output + NUMBER_OF_CARRIERS, iter_out);
    return iter_out;
}

vector_complex::iterator phy_service::fft(vector_complex::const_iterator iter_begin, vector_complex::const_iterator iter_end, vector_complex::iterator iter_out) {
    assert (iter_end - iter_begin ==  NUMBER_OF_CARRIERS);
    std::copy(iter_begin, iter_end, (complex*)d_fft_input);
    fftwf_execute(d_fftw_fwd_plan);
    // Unwrap the carriers by N/2: 1st carrier becomes N/2 and so on...
    iter_out = std::copy ((complex*)d_fft_output + NUMBER_OF_CARRIERS / 2, (complex*)d_fft_output + NUMBER_OF_CARRIERS, iter_out);
    iter_out = std::copy ((complex*)d_fft_output, (complex*)d_fft_output + NUMBER_OF_CARRIERS / 2, iter_out);
    return iter_out;
}

vector_complex::iterator phy_service::append_datastream(vector_complex::const_iterator symbol_iter_begin, vector_complex::const_iterator symbol_iter_end, vector_complex::iterator iter_out, size_t cp_length, float gain) {
    static const float ROLLOFF_WINDOW_RISE[ROLLOFF_INTERVAL] = {IEEE1901_ROLLOFF_WINDOW_RISE};
    static const float ROLLOFF_WINDOW_FALL[ROLLOFF_INTERVAL] = {IEEE1901_ROLLOFF_WINDOW_FALL};

    vector_complex::const_iterator symbol_iter = symbol_iter_begin;

    if (cp_length > 0) {
        symbol_iter = symbol_iter_end - cp_length; // start with the cyclic prefix part
        assert(cp_length > ROLLOFF_INTERVAL);
    }
    else
        symbol_iter = symbol_iter_begin; // or with the symbol, if no cyclic prefix is required

    // Multiply the cyclic prefix/symbol by a rising window and add to previous symbol
    for (size_t i=0; i<ROLLOFF_INTERVAL; i++, iter_out++)
        *iter_out = *symbol_iter++ * ROLLOFF_WINDOW_RISE[i] * gain + *iter_out;

    // Copy the rest of the cyclic prefix if exists
    if (cp_length > 0) {
        while (symbol_iter != symbol_iter_end)
            *iter_out++ = *symbol_iter++ * gain;
        symbol_iter = symbol_iter_begin; // go to the symbol begining
    }

    // Copy the symbol until the roll off
    while (symbol_iter != symbol_iter_end - ROLLOFF_INTERVAL)
        *iter_out++ = *symbol_iter++ * gain;

    // Copy the rest of the symbol multiplied by falling window
    for (unsigned int i=0; i<ROLLOFF_INTERVAL; i++)
        *iter_out++ = *symbol_iter++ * ROLLOFF_WINDOW_FALL[i] * gain;

    // Return the pointer to current position
    return iter_out;

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

void phy_service::calc_preamble(vector_complex &preamble, vector_complex &syncp_freq) {
    static const int SYNCP_CARRIERS_ANGLE_NUMBER [SYNCP_SIZE] = {IEEE1901_SYNCP_CARRIERS_ANGLE_NUMBER};

    // Calculate SYNCP
    syncp_freq = vector_complex(SYNCP_SIZE);
    for (unsigned int i = 0; i<syncp_freq.size(); i++) {
        if (SYNC_TONE_MASK[i])
            syncp_freq[i] = ANGLE_NUMBER_TO_VALUE[SYNCP_CARRIERS_ANGLE_NUMBER[i]];
    }

    // Calculate IFFT(syncp_freq)
    vector_complex syncp(SYNCP_SIZE);
    ifft_syncp(syncp_freq.begin(), syncp_freq.end(), syncp.begin()); // implicitly multiply by N
    std::for_each(syncp_freq.begin(), syncp_freq.end(), [](complex &n){ n=n*(float)SYNCP_SIZE; }); // multiply by N to match the true SYNCP

    // Calculate SYNCM
    vector_complex syncm = vector_complex(syncp);
    std::transform(syncm.begin(), syncm.end(), syncm.begin(), std::negate<vector_complex::value_type>());

    // Build preamble: [P/2 P P P P P M M M/2]
    preamble = vector_complex();
    preamble.reserve(syncp.size() * 10);
    preamble.insert(preamble.end(),syncp.begin()+syncp.size()/2,syncp.end());
    preamble.insert(preamble.end(),syncp.begin(),syncp.end());
    preamble.insert(preamble.end(),syncp.begin(),syncp.end());
    preamble.insert(preamble.end(),syncp.begin(),syncp.end());
    preamble.insert(preamble.end(),syncp.begin(),syncp.end());
    preamble.insert(preamble.end(),syncp.begin(),syncp.end());
    preamble.insert(preamble.end(),syncp.begin(),syncp.end());
    preamble.insert(preamble.end(),syncp.begin(),syncp.end());
    preamble.insert(preamble.end(),syncm.begin(),syncm.end());
    preamble.insert(preamble.end(),syncm.begin(),syncm.end());
    preamble.insert(preamble.end(),syncm.begin(),std::next(syncm.begin(),syncm.size()/2));

    DEBUG_VECTOR(preamble);
}

unsigned int phy_service::count_non_masked_carriers(tone_mask_t::const_iterator begin, tone_mask_t::const_iterator end) {
    return std::count(begin, end, true);
}

void phy_service::process_ppdu_payload(vector_complex::const_iterator iter, unsigned char *mpdu_payload_bin) {
    vector_int payload_bits(process_ppdu_payload(iter));
    pack_bitvector(payload_bits.begin(), payload_bits.end(), mpdu_payload_bin);
    return;
}

vector_int phy_service::process_ppdu_payload(vector_complex::const_iterator iter) {
    size_t n_symbols = d_rx_params.n_symbols;
    size_t fec_block_size = d_rx_params.fec_block_size;
    size_t n_blocks = d_rx_params.n_blocks;
    pb_size_t pb_size = d_rx_params.pb_size;

    if (!n_symbols){
        d_rx_payload_symbols_freq = vector_complex();
        d_rx_soft_bits = vector_float();
        d_rx_mpdu_payload = vector_int();
        return vector_int(0);
    }

    // Slice to symbols
    iter += IEEE1901_GUARD_INTERVAL_PAYLOAD;
    vector_complex symbols(n_symbols * NUMBER_OF_CARRIERS);
    vector_complex::iterator symbols_iter = symbols.begin();
    for (unsigned int i = 0; i < n_symbols; i++) {
        symbols_iter = std::copy(iter, iter + (NUMBER_OF_CARRIERS - ROLLOFF_INTERVAL), symbols_iter);
        symbols_iter = std::copy(iter - ROLLOFF_INTERVAL, iter, symbols_iter);
        iter += NUMBER_OF_CARRIERS + IEEE1901_GUARD_INTERVAL_PAYLOAD;
    }

    // Calc the freq domain symbols
    d_rx_payload_symbols_freq = vector_complex(n_symbols * NUMBER_OF_CARRIERS);
    vector_complex::iterator symbols_freq_iter = d_rx_payload_symbols_freq.begin();
    for (symbols_iter = symbols.begin(); symbols_iter != symbols.end(); symbols_iter+=NUMBER_OF_CARRIERS) {
        // Perform FFT to get the freq domain of the received signal
        symbols_freq_iter = fft(symbols_iter, symbols_iter + NUMBER_OF_CARRIERS, symbols_freq_iter);
        DEBUG_VECTOR_RANGE("symbols_freq", symbols_freq_iter - NUMBER_OF_CARRIERS, symbols_freq_iter)
    }

    tone_info_t tone_info = get_tone_info(d_rx_params.tone_mode);

    // Perform channel estimation based on payload QPSK carriers or preamble
    if (d_rx_params.tone_mode == TM_NO_ROBO) {
        if (d_channel_est_mode == CE_PAYLOAD)
            estimate_channel_gain_payload(d_rx_payload_symbols_freq.begin(), d_rx_payload_symbols_freq.end(), d_qpsk_tone_mask, d_channel_response);
        else if (d_channel_est_mode == CE_PREAMBLE)
            estimate_channel_gain_preamble(d_channel_response);
    }
    stats.channel = d_channel_response.carriers;
    DEBUG_VECTOR(d_channel_response.carriers);

    // Demodulate
    d_rx_soft_bits = vector_float(tone_info.capacity * n_symbols);
    vector_float::iterator rx_soft_bits_iter = d_rx_soft_bits.begin();
    rx_soft_bits_iter = demodulate_symbols(d_rx_payload_symbols_freq.begin(), d_rx_payload_symbols_freq.end(), rx_soft_bits_iter, tone_info.tone_map, d_channel_response);

    // Trim the dummy bits in the last symbol
    d_rx_soft_bits.erase(d_rx_soft_bits.begin() + n_blocks * fec_block_size, d_rx_soft_bits.end());
    DEBUG_VECTOR(d_rx_soft_bits);

    // Deinterleave and decode blocks
    rx_soft_bits_iter = d_rx_soft_bits.begin();
    int scrambler_state = scrambler_init(); // init the scrambler state
    d_rx_mpdu_payload = vector_int(n_blocks * calc_phy_block_size(pb_size));
    vector_int::iterator payload_bits_iter = d_rx_mpdu_payload.begin();
    for (size_t i = 0; i< n_blocks; i++) {
        vector_float block_bits = vector_float(rx_soft_bits_iter, rx_soft_bits_iter + fec_block_size);
        vector_float received_info;
        vector_float received_parity;
        vector_int decoded_info;

        if (d_rx_params.tone_mode != TM_NO_ROBO) {
            vector_float robo_deinterleaved = robo_deinterleaver(block_bits, calc_encoded_block_size(tone_info.rate, pb_size), d_rx_params.tone_mode);
            DEBUG_VECTOR(robo_deinterleaved);
            received_info = channel_deinterleaver(robo_deinterleaved, received_parity, pb_size, tone_info.rate);
        } else {
            received_info = channel_deinterleaver(block_bits, received_parity, pb_size, tone_info.rate);
        }

        decoded_info = tc_decoder(received_info, received_parity, pb_size, tone_info.rate);

        DEBUG_VECTORINT_PACK(decoded_info);

        vector_int descrambled = scrambler(decoded_info, scrambler_state);
        DEBUG_VECTOR(descrambled);

        payload_bits_iter = std::copy(descrambled.begin(),descrambled.end(), payload_bits_iter);
        rx_soft_bits_iter += fec_block_size;
    }
    DEBUG_VECTOR(d_rx_mpdu_payload);

    return d_rx_mpdu_payload;
}

void phy_service::post_process_ppdu() {
    if (d_rx_params.n_symbols) { // If bits received, use them to calculate BER and channel estimation
        assert(d_rx_soft_bits.size());
        int n_blocks =  d_rx_soft_bits.size() / d_rx_params.fec_block_size;
        tone_info_t tone_info = get_tone_info(d_rx_params.tone_mode);

        // Find the received hard bits (convert from soft to hard)
        vector_int hard_demodulated_bits(d_rx_params.fec_block_size * n_blocks);
        std::transform(d_rx_soft_bits.begin(), d_rx_soft_bits.begin() + hard_demodulated_bits.size(), hard_demodulated_bits.begin(), [](float v){return v<0;});
        DEBUG_VECTOR(hard_demodulated_bits);

        // Find the expected hard bits
        vector_int mpdu_payload_ref;
        if (d_rx_params.type == DT_SOUND) // if sound, the expected bits are zeros
            mpdu_payload_ref = vector_int(d_rx_mpdu_payload.size());
        else
            mpdu_payload_ref = d_rx_mpdu_payload;
        assert(mpdu_payload_ref.size());
        vector_int hard_demodulated_bits_ref = encode_payload(mpdu_payload_ref, d_rx_params.pb_size, tone_info.rate, d_rx_params.tone_mode);
        DEBUG_VECTOR(hard_demodulated_bits_ref);

        // Update stats with BER and number of bits
        int diff = 0;
        for (size_t i=0; i<hard_demodulated_bits.size(); i++)
            diff += (hard_demodulated_bits[i] != hard_demodulated_bits_ref[i]);
        stats.ber = (float)diff/hard_demodulated_bits.size();
        stats.n_bits = hard_demodulated_bits.size();

        // Perform channel estimation if SOUND frame
        if (d_rx_params.type == DT_SOUND) {
            // Find the expected modulated symbols for channel estimation
            assert(d_rx_payload_symbols_freq.size());
            vector_complex symbols_freq_ref = modulate(hard_demodulated_bits_ref, tone_info);
            estimate_channel_gain_sound(d_rx_payload_symbols_freq.begin(), d_rx_payload_symbols_freq.end(), symbols_freq_ref.begin(), d_channel_response);
            stats.channel = d_channel_response.carriers;
            DEBUG_VECTOR(d_channel_response.carriers);
        }
    } else {
        stats.ber = 0; // if no payload, BER=0
        stats.n_bits = 0;
    }
}

bool phy_service::process_ppdu_frame_control(vector_complex::const_iterator iter, unsigned char* mpdu_fc_bin) {
    vector_int mpdu_fc_int;
    if (process_ppdu_frame_control(iter, mpdu_fc_int) == true) {
        if (mpdu_fc_bin != NULL)
            pack_bitvector(mpdu_fc_int.begin(), mpdu_fc_int.end(), mpdu_fc_bin);
        return true;
    }
    return false;
}

bool phy_service::process_ppdu_frame_control(vector_complex::const_iterator iter, vector_int &mpdu_fc_int) {
    // Resolve frame control symbol
    iter += IEEE1901_GUARD_INTERVAL_FC;
    vector_complex fc_symbol_data(NUMBER_OF_CARRIERS);
    vector_complex::iterator fc_symbol_data_iter = fc_symbol_data.begin();
    fc_symbol_data_iter = std::copy(iter, iter + NUMBER_OF_CARRIERS - ROLLOFF_INTERVAL, fc_symbol_data_iter);
    fc_symbol_data_iter = std::copy(iter - ROLLOFF_INTERVAL, iter, fc_symbol_data_iter);
    DEBUG_VECTOR(fc_symbol_data);

    vector_complex fc_symbol_freq(NUMBER_OF_CARRIERS);
    fft(fc_symbol_data.begin(), fc_symbol_data.end(), fc_symbol_freq.begin());
    DEBUG_VECTOR(fc_symbol_freq);

    // Demodulate
    vector_float fc_soft_bits = vector_float(BROADCAST_QPSK_TONE_INFO.capacity);
    vector_float::iterator fc_soft_bits_iter = fc_soft_bits.begin();
    demodulate_symbols(fc_symbol_freq.begin(), fc_symbol_freq.end(), fc_soft_bits_iter, BROADCAST_QPSK_TONE_INFO.tone_map, d_channel_response);
    DEBUG_VECTOR(fc_soft_bits);

    // Undo the diversity copy
    vector_float decopied = combine_copies(fc_soft_bits, FRAME_CONTROL_NBITS + 12/2, FRAME_CONTROL_NBITS*2 + 12); // +12/2 because of non standard turbo encoder
    DEBUG_VECTOR(decopied);

    // Undo channel interleaver
    vector_float received_parity;
    vector_float received_info = channel_deinterleaver(decopied, received_parity, PB16, RATE_1_2);

    // Decode
    mpdu_fc_int = tc_decoder(received_info, received_parity, PB16, RATE_1_2);
    DEBUG_VECTOR(mpdu_fc_int);

    // Determine parameters
    d_rx_params = rx_params_t();
    bool result = get_rx_params(mpdu_fc_int, d_rx_params);

    // Update channel estimation params
    if (result)
        d_channel_response.frame_control_carriers = sum_carriers_gain(fc_symbol_freq.begin(), fc_symbol_freq.end(), BROADCAST_TONE_MASK);

    stats.tone_mode = d_rx_params.tone_mode;
    return result;
}

void phy_service::process_noise(vector_complex::const_iterator iter_begin, vector_complex::const_iterator iter_end) {
    static const int N = NUMBER_OF_CARRIERS; // window length
    int M = iter_end - iter_begin; // total signal length
    int K = M / N; // number of windows fits in signal
    vector_complex w_fft(N);
    d_noise_psd.fill(0); // init vector to zero
    for (int k=0; k<K; k++) {
        fft(iter_begin + k*N, iter_begin + k*N + N, w_fft.begin());
        // Calculate the PSD, and average with previous values
        for (int i=0; i<N; i++)
           d_noise_psd[i] += std::norm(w_fft[i]) / K;
    }
    stats.noise_psd = d_noise_psd;
    DEBUG_VECTOR(d_noise_psd);
    return;
}

 tone_map_t phy_service::calculate_tone_map(float P_t, tone_mask_t qpsk_force_mask) {
    // Calculating the SNR. The average received signal is NUMBER_OF_CARRIERS*H[k]
    tones_float_t snr;
    for (size_t i=0; i<NUMBER_OF_CARRIERS; i++)
        if (d_channel_response.mask[i])
            snr[i] = std::norm(d_channel_response.carriers[i] * (float)NUMBER_OF_CARRIERS) / d_noise_psd[i];
        else
            snr[i] = 0;

    stats.snr = snr; // update stats

    typedef std::pair<float,int> carrier_ber_t;
    tone_map_t tone_map;
    tone_map.fill(MT_NULLED);
    // Init all carriers bitloading to QAM4096
    std::priority_queue<carrier_ber_t> ber_set;
    float P_bar_nom = 0;
    int P_bar_denom = 0;
    for (int i=0; i<NUMBER_OF_CARRIERS; i++) {
        if (d_channel_response.mask[i]) {
            modulation_type_t m = qpsk_force_mask[i] ? MT_QPSK : MT_QAM4096; // determine modulation (forced/QAM4096)
            tone_map[i] = m;
            float ser = calc_ser(m, snr[i]);
            int b = MODULATION_MAP[m].n_bits;
            P_bar_nom += ser;   // sum(P[i]*b[i])
            P_bar_denom += b;   // sum(b[i])
            if (!qpsk_force_mask[i]) {// push to set only if not a forced map
                carrier_ber_t carrier_ber = std::make_pair(ser/b, i); // ser/b is the bit error rate
                ber_set.push(carrier_ber);
            }
        }
    }

    // Incremental algorithm
    while (P_bar_nom/P_bar_denom > P_t && !ber_set.empty()) {  // test if sum(P[i]*b[i])/sum(b[i]) > P_t)
        carrier_ber_t carrier_ber = ber_set.top(); // get the bitloading of the worst carrier
        float ber = carrier_ber.first;
        int i = carrier_ber.second;
        float new_ser = 0;
        ber_set.pop(); // remove it from the set
        modulation_type_t m = tone_map[i];
        int b = MODULATION_MAP[m].n_bits;
        P_bar_nom -= b * ber; // substract the removed SER from the sum
        P_bar_denom -= b;
        if (m != MT_BPSK) {
            m = (modulation_type_t)((int)m - 1); // decrease its bitloading and calculate its new BER
            new_ser = calc_ser(m, snr[i]);
            b = MODULATION_MAP[m].n_bits;
            carrier_ber_t carrier_ber(new_ser/b, i);
            ber_set.push(carrier_ber); // add the bitloading back to the set
            P_bar_nom += new_ser; // add the new BER to the sum
            P_bar_denom += b;
            tone_map[i] = m;
        } else {
            tone_map[i] = MT_NULLED;
        }
    }
    DEBUG_VECTOR(tone_map);
    return tone_map;
}

inline float phy_service::calc_ser(modulation_type_t m, float snr) {
    float ser = 0;
    switch (m) {
        case MT_BPSK: {
            float f = std::erfc(sqrt(snr));
            ser = f / 2;
            break;
        }
        case MT_QAM8:
        {
            float f1 = std::erfc(MODULATION_MAP[m].scale * sqrt(snr));
            float f2 = std::erfc(MODULATION_MAP[m].scale * 1.29 * sqrt(snr));
            ser = 3 / 4 * f1 * - 3 / 8 * f1 * f2 + f2 / 2;
            break;
        }
        case MT_QPSK:
        case MT_QAM16:
        case MT_QAM64:
        case MT_QAM256:
        case MT_QAM1024:
        case MT_QAM4096: {
            float M = 1 << MODULATION_MAP[m].n_bits;
            float f = std::erfc(MODULATION_MAP[m].scale * sqrt(snr)); // MODULATION_MAP[m].scale = sqrt(3/(M-1)/2)
            float A = 1 - 1 / sqrt(M);
            ser = 2 * A * f * (1 - A * f / 2); // calculate symbol error rate (SER) for the carrier
            break;
        }
        case MT_NULLED:
            break; // should never arrive here
    }
    return ser;
}

void phy_service::set_tone_map(tone_map_t tone_map) {
    d_custom_tone_info.tone_map = tone_map;
    d_custom_tone_info.rate = RATE_1_2;
    update_tone_info_capacity(d_custom_tone_info);
    DEBUG_VAR(d_custom_tone_info.capacity);
    for (size_t i=0; i<tone_map.size(); i++)
        d_qpsk_tone_mask[i] = (tone_map[i] == MT_QPSK);
}

int phy_service::get_mpdu_payload_size() {
    return d_rx_params.n_blocks * calc_phy_block_size(d_rx_params.pb_size) / 8;
}

int phy_service::get_ppdu_payload_length() {
    return d_rx_params.n_symbols * (NUMBER_OF_CARRIERS + IEEE1901_GUARD_INTERVAL_PAYLOAD);
}

bool phy_service::get_rx_params (const vector_int &fc_bits, rx_params_t &rx_params) {
    if (!crc24_check(fc_bits))
        return false;
    int fl_width = 0;
    rx_params.type = (delimiter_type_t) get_field(fc_bits, IEEE1901_FRAME_CONTROL_DT_IH_OFFSET, IEEE1901_FRAME_CONTROL_DT_IH_WIDTH);
    DEBUG_VAR(rx_params.type);
    switch (rx_params.type) {
        // SOF frame control
        case DT_SOF: {

            int pbsz = get_field(fc_bits, IEEE1901_FRAME_CONTROL_SOF_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOF_PBSZ_WIDTH);
            switch (pbsz) {
                case 0: rx_params.pb_size = PB520; DEBUG_ECHO("pb_size_t = PB520"); break;
                case 1: rx_params.pb_size = PB136; DEBUG_ECHO("pb_size_t = PB136"); break;
            }

            int tmi = get_field(fc_bits, IEEE1901_FRAME_CONTROL_SOF_TMI_OFFSET, IEEE1901_FRAME_CONTROL_SOF_TMI_WIDTH);
            switch (tmi) {
                case 0: rx_params.tone_mode = TM_STD_ROBO; DEBUG_ECHO("Tone Mode = TM_STD_ROBO"); break;
                case 1: rx_params.tone_mode = TM_HS_ROBO; DEBUG_ECHO("Tone Mode = TM_HS_ROBO"); break;
                case 2: rx_params.tone_mode = TM_MINI_ROBO; DEBUG_ECHO("Tone Mode = TM_MINI_ROBO"); break;
                default: rx_params.tone_mode = TM_NO_ROBO; DEBUG_ECHO("Tone Mode = TM_NO_ROBO"); break;
            }

            fl_width = get_field(fc_bits, IEEE1901_FRAME_CONTROL_SOF_FL_OFFSET, IEEE1901_FRAME_CONTROL_SOF_FL_WIDTH);
            rx_params.n_symbols = (fl_width * 1.28 - IEEE1901_RIFS_DEFAULT - (float)IEEE1901_ROLLOFF_INTERVAL / SAMPLE_RATE) / ((NUMBER_OF_CARRIERS + (float)IEEE1901_GUARD_INTERVAL_PAYLOAD) / SAMPLE_RATE) + 0.5; // add 0.5 due numerical instability
            break;
        }

        // SACK frame control
        case DT_SACK: {
            rx_params.n_symbols = 0;
            break;
        }

        // Sound frame control
        case DT_SOUND: {
            int pbsz = get_field(fc_bits, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_WIDTH);
            switch (pbsz) {
                case 0: rx_params.pb_size = PB520; rx_params.tone_mode = TM_STD_ROBO; DEBUG_ECHO("pb_size_t = PB520"); break;
                case 1: rx_params.pb_size = PB136; rx_params.tone_mode = TM_MINI_ROBO; DEBUG_ECHO("pb_size_t = PB136"); break;
            }

            fl_width = get_field(fc_bits, IEEE1901_FRAME_CONTROL_SOUND_FL_OFFSET, IEEE1901_FRAME_CONTROL_SOUND_FL_WIDTH);
            rx_params.n_symbols = (fl_width * 1.28 - IEEE1901_RIFS_DEFAULT - (float)IEEE1901_ROLLOFF_INTERVAL / SAMPLE_RATE) / ((NUMBER_OF_CARRIERS + (float)IEEE1901_GUARD_INTERVAL_PAYLOAD) / SAMPLE_RATE) + 0.5; // add 0.5 due numerical instability
            break;
        }

        // Any other frame type is not supported
        default:
            return false;
            break;
    }
    // Calc number of expected symbols
    DEBUG_VAR(rx_params.n_symbols);

    // If this frame contains a payload, calculate these parameters
    if (rx_params.n_symbols > 0) {
        tone_info_t tone_info = get_tone_info(rx_params.tone_mode);
        rx_params.fec_block_size = calc_fec_block_size(rx_params.tone_mode, tone_info.rate, rx_params.pb_size);
        rx_params.n_blocks = (rx_params.n_symbols - 1) * tone_info.capacity / rx_params.fec_block_size + 1;
    }

    return true;
}

bool phy_service::crc24_check(const vector_int &bit_vector) {
    return (crc24(bit_vector) == 0x7FF01C); // The one's complement of 0x800FE3
}

vector_float::iterator phy_service::demodulate_symbols (vector_complex::const_iterator iter, vector_complex::const_iterator iter_end, vector_float::iterator soft_bits_iter, const tone_map_t& tone_map, const channel_response_t &channel_response) {
    int i = 0;
    while (iter != iter_end) {
        if (tone_map[i] != MT_NULLED) {  // If carrier is ON
            complex r = *iter / channel_response.carriers[i] / (float)NUMBER_OF_CARRIERS;
            complex p = ANGLE_NUMBER_TO_VALUE[CARRIERS_ANGLE_NUMBER[i]*2]; // Convert the angle number to its value
            complex mapped_value(r.real() * p.real() + r.imag() * p.imag(),
                                             -r.real() * p.imag() + r.imag() * p.real()); // Rotate channel value by minus angel_number to get the original mapped value
            soft_bits_iter = demodulate_soft_bits(mapped_value, tone_map[i], d_noise_psd[i] / std::norm(channel_response.carriers[i] * (float)NUMBER_OF_CARRIERS), soft_bits_iter);
        }
        i = (i + 1) % NUMBER_OF_CARRIERS;
        iter++;
    }
    return soft_bits_iter;
}

// Calculate number of bits per symbol
void phy_service::update_tone_info_capacity(tone_info_t& tone_info) {
    tone_info.capacity = 0;
    for (unsigned int i = 0; i<NUMBER_OF_CARRIERS; i++)
        tone_info.capacity += MODULATION_MAP[tone_info.tone_map[i]].n_bits;
}

vector_float::iterator phy_service::demodulate_soft_bits(const complex &value, modulation_type_t modulation, float n0, vector_float::iterator iter) {
    modulation_map_t modulation_map = MODULATION_MAP[modulation];
    switch (modulation) {
        case MT_NULLED: {
            break;
        }
        case MT_BPSK: {
            *iter++ = -4 * std::real(value) * modulation_map.scale / n0;
            break;
        }
        case MT_QPSK: {
            *iter++ = -4 * std::real(value) * modulation_map.scale / n0;
            *iter++ = -4 * std::imag(value) * modulation_map.scale / n0;
            break;
        }
        case MT_QAM8: {
            iter = demodulate_soft_bits_helper(modulation_map.n_bits-1, std::real(value), modulation_map.scale, n0, iter);
            *iter++ = -4 * std::imag(value) * 1.29 * modulation_map.scale / n0;
            break;
        }
        case MT_QAM16:
        case MT_QAM64:
        case MT_QAM256:
        case MT_QAM1024:
        case MT_QAM4096: {
            iter = demodulate_soft_bits_helper(modulation_map.n_bits/2, std::real(value), modulation_map.scale, n0, iter);
            iter = demodulate_soft_bits_helper(modulation_map.n_bits/2, std::imag(value), modulation_map.scale, n0, iter);
            break;
        }
    }

    return iter;
}
vector_float::iterator phy_service::demodulate_soft_bits_helper(int n_bits, float r, float scale, float n0, vector_float::iterator iter) {
    r = r / scale;
    int l = 1<<n_bits;
    int k = std::round((r+1)/2)*2-1; // find the closest constellation point
    k = std::max(1-l, std::min(k, l-1)); // clip k if overflow
    for (int b = 0; b < n_bits; b++) {
        float d[2];
        for (int z=0; z<=1; z++) {
            int i_right = r + 2 * l;
            int i_left = r - 2 * l;
            // scan to the right and look for bit z (0 or 1)
            for (int i=k; i<=l-1; i+=2) {
                int dec = qam_demodulate(i,l);
                // if found the bit, calculate the distance and break
                if (((dec >> b) & 0x1) == z) {
                    i_right = i;
                    break;
                }
            }

            // scan to the left and look for bit z (0 or 1)
            for (int i=k-2; i>=1-l; i-=2) {
                int dec = qam_demodulate(i,l);// l-(i+1)/2) % l;
                // if found the bit, calculate the distance and break
                if (((dec >> b) & 0x1) == z) {
                    i_left = i;
                    break;
                }
            }
            d[z] = std::min((r-i_right)*(r-i_right), (r-i_left)*(r-i_left)); // take the minimum
        }
        *iter++ = (d[1] - d[0]) * scale * scale / n0;
    }
    return iter;
}

inline int phy_service::qam_demodulate(int v, int l) {
    // The following takes the point v from (...-7, -5, -3, -1, 1, 3, 5, 7 ...)
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

vector_float phy_service::channel_deinterleaver(const vector_float& bitstream, vector_float& parity_bitstream, pb_size_t pb_size, code_rate_t rate) {
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

vector_float phy_service::robo_deinterleaver(const vector_float& bitstream, int n_raw, tone_mode_t tone_mode)  {
    unsigned int n_copies, bits_in_last_symbol, bits_in_segment, n_pad;
    calc_robo_parameters (tone_mode, n_raw, n_copies, bits_in_last_symbol, bits_in_segment, n_pad);
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

int phy_service::calc_fec_block_size(tone_mode_t tone_mode, code_rate_t rate, pb_size_t pb_size) {
    int encoded_pb_n_bits = calc_encoded_block_size(rate, pb_size);

    // Determine parameters for ROBO mode
    unsigned int n_copies = 1, n_pad = 0, bits_in_last_symbol, bits_in_segment; // default values when not in ROBO mode
    if (tone_mode != TM_NO_ROBO) {
        calc_robo_parameters (tone_mode, encoded_pb_n_bits, n_copies, bits_in_last_symbol, bits_in_segment, n_pad);
        return (encoded_pb_n_bits + n_pad) * n_copies;
    }
    return encoded_pb_n_bits;
}

inline int phy_service::calc_phy_block_size(pb_size_t pb_size) {
    if (pb_size == PB520)
        return 520*8;
    else if (pb_size == PB136)
        return 136*8;
    else return 16*8;
}
int phy_service::calc_encoded_block_size(code_rate_t rate, pb_size_t pb_size) {
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

int phy_service::max_blocks (tone_mode_t tone_mode) {
    static const float SYMBOL_DURARION = (NUMBER_OF_CARRIERS + (float)IEEE1901_GUARD_INTERVAL_PAYLOAD + (float)IEEE1901_ROLLOFF_INTERVAL) / SAMPLE_RATE; // one symbol duration (microseconds)
    static const float MAX_FRAME_DURATION = ((1 << IEEE1901_FRAME_CONTROL_SOF_FL_WIDTH) - 1) * 1.28 - IEEE1901_RIFS_DEFAULT; // maximum of all symbols duration allowed (microseconds)
    static const int MAX_N_SYMBOLS = MAX_FRAME_DURATION / SYMBOL_DURARION; // maximum number of symbols
    tone_info_t tone_info = get_tone_info(tone_mode);
    int encoded_pb_n_bits = calc_fec_block_size(tone_mode, tone_info.rate, PB520);
    int max_n_bits = MAX_N_SYMBOLS * tone_info.capacity; // maximum number of bits
    return max_n_bits / encoded_pb_n_bits; // maximum number of PB520 blocks
}

phy_service::tone_info_t phy_service::build_broadcast_tone_info(modulation_type_t modulation) {
    tone_info_t broadcast_carriers;
    for (size_t i=0; i<BROADCAST_TONE_MASK.size(); i++)
        broadcast_carriers.tone_map[i] = BROADCAST_TONE_MASK[i] ? modulation : MT_NULLED;
    broadcast_carriers.rate = RATE_1_2;
    update_tone_info_capacity(broadcast_carriers);
    return broadcast_carriers;
}

void phy_service::create_fftw_vars () {
    std::lock_guard<std::mutex> lck (fftw_mtx); // lock this part since fftw plan creation is not thread safe

    // Create fftw3 plan (used by fft and ifft routine)
    d_ifft_input = fftwf_alloc_complex(NUMBER_OF_CARRIERS);
    d_ifft_output = fftwf_alloc_complex(NUMBER_OF_CARRIERS);
    d_fftw_rev_plan = fftwf_plan_dft_1d (NUMBER_OF_CARRIERS,
                                            d_ifft_input,
                                            d_ifft_output,
                                            FFTW_BACKWARD,
                                            FFTW_MEASURE);

    d_fft_input = fftwf_alloc_complex(NUMBER_OF_CARRIERS);
    d_fft_output = fftwf_alloc_complex(NUMBER_OF_CARRIERS);
    d_fftw_fwd_plan = fftwf_plan_dft_1d (NUMBER_OF_CARRIERS,
                                            d_fft_input,
                                            d_fft_output,
                                            FFTW_FORWARD,
                                            FFTW_MEASURE);

    d_ifft_syncp_input = fftwf_alloc_complex(SYNCP_SIZE);
    d_ifft_syncp_output = fftwf_alloc_complex(SYNCP_SIZE);
    d_fftw_syncp_rev_plan = fftwf_plan_dft_1d (SYNCP_SIZE,
                                            d_ifft_syncp_input,
                                            d_ifft_syncp_output,
                                            FFTW_BACKWARD,
                                            FFTW_MEASURE);

    d_fft_syncp_input = fftwf_alloc_complex(SYNCP_SIZE);
    d_fft_syncp_output = fftwf_alloc_complex(SYNCP_SIZE);
    d_fftw_syncp_fwd_plan = fftwf_plan_dft_1d (SYNCP_SIZE,
                                            d_fft_syncp_input,
                                            d_fft_syncp_output,
                                            FFTW_FORWARD,
                                            FFTW_MEASURE);
}

vector_complex::iterator phy_service::fft_syncp(vector_complex::const_iterator iter_begin, vector_complex::const_iterator iter_end, vector_complex::iterator iter_out) {
    assert (iter_end - iter_begin ==  SYNCP_SIZE);
    std::copy(iter_begin, iter_end, (complex*)d_fft_syncp_input);
    fftwf_execute(d_fftw_syncp_fwd_plan);
    // Unwrap the carriers by N/2, 1st carrier becomes N/2  and so on...
    iter_out = std::copy((complex*)d_fft_syncp_output + SYNCP_SIZE / 2, (complex*)d_fft_syncp_output + SYNCP_SIZE, iter_out);
    iter_out = std::copy((complex*)d_fft_syncp_output, (complex*)d_fft_syncp_output + SYNCP_SIZE / 2, iter_out);
    return iter_out;
}

vector_complex::iterator phy_service::ifft_syncp(vector_complex::const_iterator iter_begin, vector_complex::const_iterator iter_end, vector_complex::iterator iter_out) {
    assert (iter_end - iter_begin == SYNCP_SIZE);
    // Wrap the carriers by N/2, so N/2 carrier becomes first and so on...
    std::copy(iter_begin + SYNCP_SIZE / 2, iter_end, (complex*)d_ifft_syncp_input);
    std::copy(iter_begin, iter_begin + SYNCP_SIZE / 2, (complex*)d_ifft_syncp_input + SYNCP_SIZE / 2);
    fftwf_execute(d_fftw_syncp_rev_plan);
    iter_out = std::copy ((complex*)d_ifft_syncp_output, (complex*)d_ifft_syncp_output + SYNCP_SIZE, iter_out);
    return iter_out;
}

void phy_service::process_ppdu_preamble(vector_complex::const_iterator iter, vector_complex::const_iterator iter_end) {
    assert (iter_end - iter == PREAMBLE_SIZE);

    auto iter_1 = iter + SYNCP_SIZE / 2 + 3 * SYNCP_SIZE; // SYNCP between [3.5-4.5]
    auto iter_2 = iter + SYNCP_SIZE / 2 + 4 * SYNCP_SIZE; // SYNCP between [4.5-5.5]
    auto iter_3 = iter + SYNCP_SIZE / 2 + 5 * SYNCP_SIZE; // SYNCP between [5.5-6.5]
    vector_complex syncp_avg(SYNCP_SIZE);
    for (auto avg_iter = syncp_avg.begin(); avg_iter!=syncp_avg.end(); avg_iter++) // average all 3 SYNCPs
        *avg_iter = (*iter_1++ + *iter_2++ + *iter_3++) / (float)3;
    DEBUG_VECTOR(syncp_avg);

    vector_complex syncp_freq(SYNCP_SIZE);
    fft_syncp(syncp_avg.begin(), syncp_avg.end(), syncp_freq.begin());
    DEBUG_VECTOR(syncp_freq);

    d_channel_response.n_syncp_symbols = 3; // set number of syncp symbols in channel response calculation
    estimate_channel_syncp(syncp_freq.begin(), syncp_freq.end(), SYNCP_FREQ.begin(), d_channel_response);
    stats.channel = d_channel_response.carriers;
    DEBUG_VECTOR(d_channel_response.carriers);
    return;
}

vector_float phy_service::phase_unwrap(const vector_float &y) {
    vector_float y_unwrapped(y.size());
    y_unwrapped[0] = y[0];
    for (size_t i = 1; i < y.size(); i++) {
        float d = y[i] - y[i-1];
        d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
        y_unwrapped[i] = y_unwrapped[i-1] + d;
    }
    return y_unwrapped;
}

tones_float_t phy_service::sum_carriers_gain(vector_complex::const_iterator iter, vector_complex::const_iterator iter_end, const tone_mask_t &mask) {
    assert((iter_end - iter) % NUMBER_OF_CARRIERS == 0);
    tones_float_t carriers;
    for (auto i = 0; i < NUMBER_OF_CARRIERS; i++) {
        if (mask[i]) {
            unsigned int j = i;
            float a = 0;
            while (iter + j < iter_end) {
                a += std::norm(*(iter + j));
                j += NUMBER_OF_CARRIERS;
            }
            carriers[i] = a;
        } else {
            carriers[i] = 0;
        }
    }
    return carriers;
}

void phy_service::estimate_channel_gain_sound(vector_complex::const_iterator iter, vector_complex::const_iterator iter_end, vector_complex::const_iterator iter_ref, channel_response_t &channel_response) {
    assert((iter_end - iter) % NUMBER_OF_CARRIERS == 0);
    int n_symbols = (iter_end - iter) / NUMBER_OF_CARRIERS;
    for (auto i = 0; i < NUMBER_OF_CARRIERS; i++) {
        if (channel_response.mask[i]) {
            unsigned int j = i;
            complex a = 0;
            while (iter + j < iter_end) {
                a += *(iter + j) / *(iter_ref + j);
                j += NUMBER_OF_CARRIERS;
            }
            channel_response.carriers[i] =  std::abs(a) / n_symbols * channel_response.carriers[i] / abs(channel_response.carriers[i]);
        }
    }
}

void phy_service::estimate_channel_gain_payload(vector_complex::const_iterator iter, vector_complex::const_iterator iter_end, const tone_mask_t &qpsk_tone_mask, channel_response_t &channel_response) {
    int n_payload_symbols = (iter_end - iter) / NUMBER_OF_CARRIERS;
    tones_float_t payload_carriers = sum_carriers_gain(iter, iter_end, qpsk_tone_mask);
    vector_float x;
    vector_float y;
    x.reserve(NUMBER_OF_CARRIERS);
    y.reserve(NUMBER_OF_CARRIERS);
    static const float p_payload = NUMBER_OF_CARRIERS * NUMBER_OF_CARRIERS;
    static const float p_frame_control = (NUMBER_OF_CARRIERS * IEEE1901_SCALE_FACTOR_FC / IEEE1901_SCALE_FACTOR_PAYLOAD) * (NUMBER_OF_CARRIERS * IEEE1901_SCALE_FACTOR_FC / IEEE1901_SCALE_FACTOR_PAYLOAD);
    static const float p_sync = (IEEE1901_SCALE_FACTOR_PREAMBLE / IEEE1901_SCALE_FACTOR_PAYLOAD) * (IEEE1901_SCALE_FACTOR_PREAMBLE / IEEE1901_SCALE_FACTOR_PAYLOAD);
    float sync_variance = channel_response.n_syncp_symbols * channel_response.n_syncp_symbols * N_SYNC_CARRIERS * N_SYNC_CARRIERS;
    for (size_t i = 0; i < NUMBER_OF_CARRIERS; i++) {
        if (qpsk_tone_mask[i]) {
            float nominator =
                qpsk_tone_mask[i] * payload_carriers[i] +
                BROADCAST_TONE_MASK[i] * channel_response.frame_control_carriers[i] +
                SYNC_TONE_MASK_EXPANDED[i] * channel_response.sync_carriers[i] * sync_variance -
                d_noise_psd[i] * (qpsk_tone_mask[i] * n_payload_symbols + BROADCAST_TONE_MASK[i] + SYNC_TONE_MASK_EXPANDED[i]);
            if (nominator > 0) { // add it only if greater than 0. lower than zero does not make sense so the point is discarded
                float denominator =
                    qpsk_tone_mask[i] * n_payload_symbols * p_payload +
                    BROADCAST_TONE_MASK[i] * p_frame_control +
                    SYNC_TONE_MASK_EXPANDED[i] * p_sync * sync_variance;
                y.push_back(std::sqrt(nominator / denominator));
                x.push_back(i);
            }
        }
    }

    assert(x.size() > 2); // require at least two carriers for interpolation...

    std::vector<linear_set_t> interp_set = linear(x,y);
    for (unsigned int i = 0, j = 0; i < NUMBER_OF_CARRIERS; i++) {
        if (channel_response.mask[i]) { // interpolate carriers only if necessary
            while (j < interp_set.size()-1 && interp_set[j+1].x <= i) j++;
            channel_response.carriers[i] =  linear_interpolate(interp_set[j] ,(float)i) * channel_response.carriers[i] / abs(channel_response.carriers[i]);
        }
    }

    return;
}

void phy_service::estimate_channel_gain_preamble(channel_response_t &channel_response) {
    vector_float x;
    vector_float y;
    x.reserve(NUMBER_OF_CARRIERS);
    y.reserve(NUMBER_OF_CARRIERS);
    static const float p_frame_control = (NUMBER_OF_CARRIERS * IEEE1901_SCALE_FACTOR_FC / IEEE1901_SCALE_FACTOR_PAYLOAD) * (NUMBER_OF_CARRIERS * IEEE1901_SCALE_FACTOR_FC / IEEE1901_SCALE_FACTOR_PAYLOAD);
    static const float p_sync = (IEEE1901_SCALE_FACTOR_PREAMBLE / IEEE1901_SCALE_FACTOR_PAYLOAD) * (IEEE1901_SCALE_FACTOR_PREAMBLE / IEEE1901_SCALE_FACTOR_PAYLOAD);
    float sync_variance = channel_response.n_syncp_symbols * channel_response.n_syncp_symbols * N_SYNC_CARRIERS * N_SYNC_CARRIERS;
    for (size_t i = 0; i < NUMBER_OF_CARRIERS; i++) {
        if (SYNC_TONE_MASK_EXPANDED[i]) {
            float nominator =
                BROADCAST_TONE_MASK[i] * channel_response.frame_control_carriers[i] +
                SYNC_TONE_MASK_EXPANDED[i] * channel_response.sync_carriers[i] * sync_variance -
                d_noise_psd[i] * (BROADCAST_TONE_MASK[i] + SYNC_TONE_MASK_EXPANDED[i]);
            if (nominator > 0) { // add it only if greater than 0. lower than zero does not make sense so the point is discarded
                float denominator =
                    BROADCAST_TONE_MASK[i] * p_frame_control +
                    SYNC_TONE_MASK_EXPANDED[i] * p_sync * sync_variance;
                y.push_back(std::sqrt(nominator / denominator));
                x.push_back(i);
            }
        }
    }

    if (x.size() < 2) return; // require at least two carriers for interpolation...

    std::vector<linear_set_t> interp_set = linear(x,y);
    for (unsigned int i = 0, j = 0; i < NUMBER_OF_CARRIERS; i++) {
        if (channel_response.mask[i]) { // interpolate carriers only if necessary
            while (j < interp_set.size()-1 && interp_set[j+1].x <= i) j++;
            channel_response.carriers[i] =  linear_interpolate(interp_set[j] ,(float)i) * channel_response.carriers[i] / abs(channel_response.carriers[i]);
        }
    }

    return;
}

void phy_service::estimate_channel_syncp (vector_complex::const_iterator iter, vector_complex::const_iterator iter_end, vector_complex::const_iterator ref_iter, channel_response_t &channel_response) {
    vector_float x(N_SYNC_ACTIVE_TONES);
    vector_float y(N_SYNC_ACTIVE_TONES);
    int i=0, j=0;
    while (iter != iter_end) {
        if (SYNC_TONE_MASK[i]) {
            int k = i * NUMBER_OF_CARRIERS / N_SYNC_CARRIERS;
            x[j] = k;
            y[j] = std::arg(*iter / *ref_iter);
            channel_response.sync_carriers[k] = std::norm(*iter / *ref_iter);
            j++;
        }
        iter++;
        ref_iter++;
        i++;
    }

    vector_float y_unwrapped = phase_unwrap(y);
    std::vector<phy_service::spline_set_t> interp_set = spline(x, y_unwrapped);
    // std::vector<phy_service::linear_set_t> interp_set = linear(x, y_unwrapped);
    for (unsigned int i = 0, j = 0; i < NUMBER_OF_CARRIERS; i++) {
        if (channel_response.mask[i]) { // interpolate carriers only if necessary
            while (j < interp_set.size()-1 && interp_set[j+1].x <= i) j++;
            channel_response.carriers[i] =  abs(channel_response.carriers[i]) *
                                            std::exp(complex(0, spline_interpolate(interp_set[j] ,(float)i)));
                                            // std::exp(complex(0, linear_interpolate(interp_set[j] ,(float)i)));
        }
    }
    return;
}

inline float phy_service::spline_interpolate(spline_set_t &spline_set, float x) {
    float dx = x - spline_set.x;
    return spline_set.a +
           spline_set.b * dx +
           spline_set.c * dx * dx +
           spline_set.d * dx * dx * dx;
}

inline float phy_service::linear_interpolate(linear_set_t &linear_set, float x){
    float dx = x - linear_set.x;
    return linear_set.b +
           linear_set.a * dx;
}

std::vector<phy_service::linear_set_t> phy_service::linear(const vector_float &x, const vector_float &y) {
    int n = x.size()-1;
    std::vector<linear_set_t> output_set(n);
    for (int j=0; j<n; j++){
        output_set[j].a = (y[j+1] - y[j]) / (x[j+1] - x[j]);
        output_set[j].b = y[j];
        output_set[j].x = x[j];
    }
    return output_set;
}

std::vector<phy_service::spline_set_t> phy_service::spline(const vector_float &x, const vector_float &y) {
    int n = x.size()-1;
    assert(n > 0);
    vector_float a(y);
    vector_float b(n);
    vector_float d(n);
    vector_float h(n);

    for(int i = 0; i < n; ++i)
        h[i] = x[i+1]-x[i];

    vector_float alpha(n);
    for(int i = 0; i < n; ++i)
        alpha[i] = ((float)3*(a[i+1] - a[i]) / h[i] - (float)3 * (a[i] - a[i-1]) / h[i-1]);

    vector_float c(n+1);
    vector_float l(n+1);
    vector_float mu(n+1);
    vector_float z(n+1);
    l[0] = 1;
    mu[0] = 0;
    z[0] = 0;

    for(int i = 1; i < n; ++i)
    {
        l[i] = 2 * (x[i+1] - x[i-1]) - h[i-1] * mu[i-1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i-1] * z[i-1]) / l[i];
    }

    l[n] = 1;
    z[n] = 0;
    c[n] = 0;

    for(int j = n-1; j >= 0; --j)
    {
        c[j] = z [j] - mu[j] * c[j+1];
        b[j] = (a[j+1] - a[j]) / h[j] - h[j] * (c[j+1] + (float)2 * c[j]) / (float)3;
        d[j] = (c[j+1] - c[j]) / (float)3 / h[j];
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
