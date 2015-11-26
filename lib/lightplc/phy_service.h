#ifndef _PLC_PLCP_H_
#define _PLC_PLCP_H_

#include <cstring>
#include <vector>
#include <array>
#include <complex>
#include <fftw3.h>
#include <itpp/itcomm.h>
#include "ieee1901.inc"

namespace light_plc {

typedef std::vector<int> vector_int;
typedef std::vector<float> vector_float;
typedef std::complex<float> complex;
typedef std::vector<complex> vector_complex;
typedef std::vector<vector_float> vector_symbol;
typedef std::vector<vector_complex> vector_symbol_freq;

enum code_rate {
    RATE_1_2 = 0,
    RATE_16_21 = 1,
    RATE_16_18 = 2,
};

enum pb_size_t {
    PB16 = 0,
    PB136 = 1,
    PB520 = 2,
};

enum robo_mode_t {
    NO_ROBO = 0,
    STD_ROBO = 1,
    HS_ROBO = 2,
    MINI_ROBO = 3
};

enum modulation_type {
    NULLED = 0,
    BPSK  = 1,
    QPSK  = 2,
    QAM8  = 3,
    QAM16 = 4,
    QAM64 = 5,
    QAM256 = 6,
    QAM1024 = 7,
    QAM4096 = 8
};

enum mpdu_type_t {
    MPDU_TYPE_BEACON= 0,
    MPDU_TYPE_SOF = 1,
    MPDU_TYPE_SACK = 2,
    MPDU_TYPE_RTS_CTS = 3,
    MPDU_TYPE_SOUND = 4,
    MPDU_TYPE_RSOF = 5
};

typedef std::array<modulation_type, IEEE1901_NUMBER_OF_CARRIERS+1> tone_map_t;

class phy_service
{

private:
    struct spline_set_t{
        float a;
        float b;
        float c;
        float d;
        double x;
    };

    struct modulation_map_t {
        const unsigned int n_bits;
        const complex *map;
        const float scale;
    };

    typedef std::array<bool, IEEE1901_NUMBER_OF_CARRIERS+1> carriers_mask;
    
    typedef struct channel_response {
        carriers_mask mask;
        std::array<complex, IEEE1901_NUMBER_OF_CARRIERS+1> carriers;
        std::array<float, IEEE1901_NUMBER_OF_CARRIERS+1> carriers_gain;
    } channel_response;

    typedef struct tone_info_t {
        tone_map_t tone_map;
        unsigned int capacity;
    } tone_info_t;

    typedef std::array<float, IEEE1901_NUMBER_OF_CARRIERS+1> tones_float;

    typedef struct frame_parameters {
        mpdu_type_t type;
        bool has_payload;
        unsigned int n_expected_symbols;
        pb_size_t pb_size;
        robo_mode_t robo_mode;
        code_rate rate;
        tone_info_t tone_info;
        int encoded_block_size;
        int interleaved_block_size;
        int mpdu_payload_size;
        int ppdu_payload_size;
        int inter_frame_space;
        vector_int sackd;
    } frame_parameters;

    static const int SAMPLE_RATE = IEEE1901_SAMPLE_RATE; 
    static const int FRAME_CONTROL_NBITS = IEEE1901_FRAME_CONTROL_NBITS;
    static const int TCENCODER_SEED_PB16[IEEE1901_TCENCODER_SEED_PB16_N];
    static const int TCENCODER_SEED_PB136[IEEE1901_TCENCODER_SEED_PB136_N];
    static const int TCENCODER_SEED_PB520[IEEE1901_TCENCODER_SEED_PB520_N];
    static const int CHANNEL_INTERLEAVER_OFFSET[3][3];
    static const int CHANNEL_INTERLEAVER_STEPSIZE[3][3];
    static const int NUMBER_OF_CARRIERS = IEEE1901_NUMBER_OF_CARRIERS;
    static const std::array<bool, NUMBER_OF_CARRIERS+1> TRANSMIT_MASK;
    static const std::array<bool, NUMBER_OF_CARRIERS+1> CARRIERS_BROADCAST_MASK;
    static const int CARRIERS_ANGLE_NUMBER[NUMBER_OF_CARRIERS+1];
    static const unsigned int ROLLOFF_INTERVAL = IEEE1901_ROLLOFF_INTERVAL;
    static const float ROLLOFF_WINDOW_RISE[ROLLOFF_INTERVAL];
    static const float ROLLOFF_WINDOW_FALL[ROLLOFF_INTERVAL];
    static const float SCALE_FACTOR_PAYLOAD;
    static const float SCALE_FACTOR_FC;
    static const int GUARD_INTERVAL_PAYLOAD = IEEE1901_GUARD_INTERVAL_PAYLOAD;
    static const int GUARD_INTERVAL_FC = IEEE1901_GUARD_INTERVAL_FC;
    static const int SYNCP_CARRIERS_ANGLE_NUMBER[IEEE1901_SYNCP_SIZE / 2 + 1];
    static const bool SYNCP_CARRIERS_MASK [IEEE1901_SYNCP_SIZE / 2 + 1];
    static const modulation_map_t MODULATION_MAP[9];
    static const complex ANGLE_NUMBER_TO_VALUE[16];
    static const int N_BROADCAST_CARRIERS;
    static const tone_info_t BROADCAST_CARRIERS;
    static const std::array<float, NUMBER_OF_CARRIERS*2> HAMMING_WINDOW;
    
public:
    static const int SYNCP_SIZE = IEEE1901_SYNCP_SIZE;
    static const int PREAMBLE_SIZE = SYNCP_SIZE * 10;
    static const int FRAME_CONTROL_SIZE = NUMBER_OF_CARRIERS * 2 + GUARD_INTERVAL_FC;

    phy_service (bool debug = false);
    phy_service (const phy_service &instance);
    ~phy_service (void);
    phy_service& operator=(const phy_service& rhs);

    vector_float create_sof_ppdu(const unsigned char *mpdu_payload_bin, size_t len);
    vector_float create_sof_ppdu(const vector_int &mpdu_payload);
    vector_float create_sack_ppdu(const unsigned char *sackd_bin, size_t len);
    vector_float create_sack_ppdu(const vector_int sackd);
    vector_float create_sound_ppdu(robo_mode_t robo_mode);
    void process_ppdu_preamble(vector_float::const_iterator iter, vector_float::const_iterator iter_end);
    bool process_ppdu_frame_control(vector_float::const_iterator iter);
    void process_ppdu_payload(vector_float::const_iterator iter, unsigned char *mpdu_payload_bin);
    vector_int process_ppdu_payload(vector_float::const_iterator iter);
    void process_noise(vector_float::const_iterator iter, vector_float::const_iterator iter_end);
    tone_map_t calculate_tone_map(float P_t);
    void set_tone_map(tone_map_t tone_map);
    void set_code_rate(code_rate rate);
    void set_robo_mode(robo_mode_t robo_mode);
    void set_noise_psd(float n0);
    int get_mpdu_payload_size();
    int get_ppdu_payload_length();
    mpdu_type_t get_frame_type();
    vector_float get_snr();
    vector_int get_sackd();
    void get_sackd(unsigned char *sackd_bin);
    int get_sackd_size();
    int get_inter_frame_space();
    vector_float::const_iterator preamble();
    static int max_blocks (robo_mode_t robo_mode, code_rate rate, modulation_type modulation = QPSK);
    void debug(bool debug) {d_debug = debug; return;};

private:
    vector_symbol create_payload_symbols(const vector_int &payload_bits, pb_size_t pb_size, robo_mode_t robo_mode, tone_info_t tone_info = tone_info_t(), code_rate rate = RATE_1_2);
    vector_symbol_freq create_payload_symbols_freq (const vector_int &bitstream, pb_size_t pb_size, robo_mode_t robo_mode, tone_info_t tone_info = tone_info_t(), code_rate rate = RATE_1_2);
    vector_symbol create_frame_control_symbol(const vector_int &bitstream);
    static vector_int create_sof_frame_control (unsigned int n_symbols, robo_mode_t robo_mode, pb_size_t pb_size);
    static vector_int create_sack_frame_control (const vector_int sackd);
    vector_int create_sound_frame_control (unsigned int n_symbols, pb_size_t pb_size);
    static void set_field(vector_int &bit_vector, int bit_offset, int bit_width, unsigned long new_value);
    static void pack_bitvector(vector_int::const_iterator begin, vector_int::const_iterator end, unsigned char* array);
    static vector_int unpack_into_bitvector (const unsigned char *data, size_t c);    
    static unsigned long crc24(const vector_int &bit_vector);
    static unsigned long crc32(unsigned char *icp, int icnt);
    static vector_int scrambler(const vector_int& bitstream, int &state);
    static int scrambler_init(void);
    void init_turbo_codec();
    vector_int tc_encoder(const vector_int &bitstream, pb_size_t pb_size, code_rate rate);
    vector_int tc_decoder(const vector_float &received_info, const vector_float &received_parity, pb_size_t pb_size, code_rate rate);
    static vector_int channel_interleaver(const vector_int& bitstream, const vector_int& parity, pb_size_t pb_size, code_rate rate);
    static vector_int robo_interleaver(const vector_int& bitstream, robo_mode_t robo_mode);
    static tone_info_t calc_robo_carriers (robo_mode_t robo_mode);
    static void calc_robo_parameters (robo_mode_t robo_mode, unsigned int n_raw, unsigned int &n_copies, unsigned int &bits_in_last_symbol, unsigned int &bits_in_segment, unsigned int &n_pad);
    static vector_int copier(const vector_int& bitstream, int n_carriers, int offset, int start = 0);
    vector_symbol_freq modulate(const vector_int& bits, const tone_info_t& tone_info);
    static void cyclic_prefix_and_window(vector_symbol& symbols, int gi_length, float gain);
    static itpp::bvec to_bvec (const vector_int in);
    static itpp::ivec to_ivec (const vector_int in);
    static vector_int to_vector_int (const itpp::bvec in);
    static vector_int consistuent_encoder(const vector_int& in, pb_size_t pb_size);
    static std::array<vector_int, 3> calc_turbo_interleaver_sequence();
    static vector_int turbo_interleaver(const vector_int &bitstream, pb_size_t pb_size);
    static void puncture(vector_int &bitstream, code_rate rate);
    static int pn_generator(int n_bits, int &pn_state);
    static int pn_generator_init(void);
    static bool channel_interleaver_row(const vector_int& bitstream, vector_int::iterator &iter, int step_size, int& row_no, int& rows_done, int& nibble_no, bool wrap = false);
    vector_complex fft_real(vector_float::const_iterator iter_begin, vector_float::const_iterator iter_end);
    vector_float ifft_real(vector_complex::const_iterator iter_begin, vector_complex::const_iterator iter_end);
    vector_float calc_preamble();
    static vector_complex calc_syncp_fft(const vector_float &preamble);
    static vector_float symbols_to_datastream (const vector_symbol &symbols, int ifs);
    static unsigned int count_non_masked_carriers(carriers_mask::const_iterator begin, carriers_mask::const_iterator end);
    static unsigned int count_non_masked_carriers(bool *begin, bool *end);
    static void update_tone_info_capacity(tone_info_t& tone_info);
    vector_int resolve_frame_control_symbol (const vector_float& fc_data);
    bool parse_frame_control (const vector_int &fc_bits, frame_parameters &frame_control);
    static bool crc24_check(const vector_int &bit_vector);
    static unsigned long get_field(const vector_int &bit_vector, int bit_offset, int bit_width);
    vector_float symbol_demodulate (vector_complex::const_iterator iter, const phy_service::tone_info_t& tone_info, const channel_response &channel_response);
    vector_float::iterator demodulate_helper(int n_bits, float r, float scale, float n0, vector_float::iterator iter);
    vector_float::iterator demodulate(const complex &value, modulation_type modulation, float n0, vector_float::iterator iter);
    int qam_demodulate(int v, int l);
    static vector_float combine_copies(vector_float& bitstream, int offset, int n_bits);
    static vector_float channel_deinterleaver(const vector_float& bitstream, vector_float& parity_bitstream, pb_size_t pb_size, code_rate rate);
    static bool channel_deinterleaver_row(vector_float::const_iterator& iter, vector_float& out, int step_size, int& row_no, int& rows_done, int& nibble_no, bool wrap = false);
    static vector_float robo_deinterleaver(const vector_float& bitstream, int n_raw, robo_mode_t robo_mode);
    inline int calc_block_size(pb_size_t pb_size);
    static int calc_interleaved_block_size(robo_mode_t robo_mode, code_rate rate, pb_size_t pb_size);
    static int calc_encoded_block_size(code_rate rate, pb_size_t pb_size);
    static tone_info_t build_broadcast_tone_map(modulation_type modulation = QPSK);
    void create_fftw_vars ();
    vector_complex fft_real_syncp(const vector_float& data);
    void estimate_channel_gain (vector_symbol_freq::const_iterator iter, vector_symbol_freq::const_iterator iter_end, vector_symbol_freq::const_iterator ref_iter, channel_response &channel_response);
    void estimate_channel_phase (vector_complex::const_iterator iter, vector_complex::const_iterator iter_end, vector_complex::const_iterator ref_iter, const bool cm[], channel_response &channel_response);
    static std::array<float, NUMBER_OF_CARRIERS*2> create_hamming_window();
    std::vector<spline_set_t> spline(vector_float &x, vector_float &y);
    bool d_debug;
    tone_info_t d_tone_info;
    code_rate d_code_rate;
    robo_mode_t d_robo_mode;
    float d_n0;
    fftwf_complex *d_ifft_input, *d_fft_output, *d_fft_syncp_output;
    float *d_ifft_output, *d_fft_input, *d_fft_syncp_input;
    fftwf_plan d_fftw_rev_plan, d_fftw_fwd_plan, d_fftw_syncp_fwd_plan;
    frame_parameters d_frame_parameters;
    itpp::Punctured_Turbo_Codec d_turbo_codec;
    const vector_float PREAMBLE;
    const vector_complex SYNCP_FREQ;
    const std::array<vector_int, 3> TURBO_INTERLEAVER_SEQUENCE;
    vector_symbol SOUND_PB136, SOUND_PB520;
    vector_symbol_freq SOUND_PB136_FREQ, SOUND_PB520_FREQ;
    channel_response d_broadcast_channel_response;
    tones_float d_noise_psd;
    tones_float d_snr;

};

}; /* namespace light_plc */

#endif /* _PLC_PLCP_H_ */