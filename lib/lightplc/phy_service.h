#ifndef _LIGHT_PLC_PHY_SERVICE
#define _LIGHT_PLC_PHY_SERVICE

#include <cstring>
#include <fftw3.h>
#include <itpp/itcomm.h>
#include <mutex> 
#include "defs.h"

namespace light_plc {

class phy_service
{

private:
    enum delimiter_type_t {
        DT_BEACON= 0,
        DT_SOF = 1,
        DT_SACK = 2,
        DT_RTS_CTS = 3,
        DT_SOUND = 4,
        DT_RSOF = 5
    };

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

    typedef std::array<float, IEEE1901_NUMBER_OF_CARRIERS+1> tones_float;
    
    typedef struct channel_response {
        tone_mask_t mask;
        std::array<complex, IEEE1901_NUMBER_OF_CARRIERS+1> carriers;
        tones_float carriers_gain;
    } channel_response;

    typedef struct tone_info_t {
        tone_map_t tone_map;
        unsigned int capacity;
        code_rate_t rate;
    } tone_info_t;

    typedef struct tx_params_t {
        tone_mode_t tone_mode;
        pb_size_t pb_size;
    } tx_params_t;

    typedef struct rx_params_t {
        delimiter_type_t type;
        size_t n_symbols;
        size_t n_blocks;
        pb_size_t pb_size;
        tone_mode_t tone_mode;
        int fec_block_size;
        int inter_frame_space;
    } rx_params_t;

    static const int SAMPLE_RATE = IEEE1901_SAMPLE_RATE; 
    static const int FRAME_CONTROL_NBITS = IEEE1901_FRAME_CONTROL_NBITS;
    static const int CHANNEL_INTERLEAVER_OFFSET[3][3];
    static const int CHANNEL_INTERLEAVER_STEPSIZE[3][3];
    static const int NUMBER_OF_CARRIERS = IEEE1901_NUMBER_OF_CARRIERS;
    static const int CARRIERS_ANGLE_NUMBER[NUMBER_OF_CARRIERS+1];
    static const int N_SYNC_CARRIERS = IEEE1901_SYNCP_SIZE / 2;
    static const modulation_map_t MODULATION_MAP[9];
    static const complex ANGLE_NUMBER_TO_VALUE[16];
    bool d_debug;
    tone_mask_t TONE_MASK;
    tone_mask_t BROADCAST_TONE_MASK;
    int N_BROADCAST_TONES;
    sync_tone_mask_t SYNC_TONE_MASK;
    int N_SYNC_ACTIVE_TONES;
    tone_info_t BROADCAST_QPSK_TONE_INFO;
    tone_info_t TONE_INFO_STD_ROBO;
    tone_info_t TONE_INFO_MINI_ROBO;
    tone_info_t TONE_INFO_HS_ROBO;
    vector_float PREAMBLE;
    std::array<vector_int, 3> TURBO_INTERLEAVER_SEQUENCE;

public:
    static const int SYNCP_SIZE = IEEE1901_SYNCP_SIZE;
    static const int PREAMBLE_SIZE = SYNCP_SIZE * 10;
    static const int FRAME_CONTROL_SIZE = NUMBER_OF_CARRIERS * 2 + IEEE1901_GUARD_INTERVAL_FC;
    static const int ROLLOFF_INTERVAL = IEEE1901_ROLLOFF_INTERVAL;

    phy_service (bool debug = false);
    phy_service (tone_mask_t tone_mask, tone_mask_t broadcast_tone_mask, sync_tone_mask_t sync_tone_mask, bool debug = false);
    phy_service (const phy_service &instance);
    ~phy_service (void);
    phy_service& operator=(const phy_service& rhs);

    vector_float create_ppdu(const unsigned char *mpdu_fc_bin, size_t mpdu_fc_len, const unsigned char *mpdu_payload_bin = NULL, size_t mpdu_payload_len = 0);
    vector_float create_ppdu(vector_int &mpdu_fc_int, const vector_int &mpdu_payload_int = vector_int());
    void process_ppdu_preamble(vector_float::const_iterator iter, vector_float::const_iterator iter_end);
    bool process_ppdu_frame_control(vector_float::const_iterator iter, vector_int &mpdu_fc_int);
    bool process_ppdu_frame_control(vector_float::const_iterator iter, unsigned char* mpdu_fc_bin = NULL);
    void process_ppdu_payload(vector_float::const_iterator iter, unsigned char *mpdu_payload_bin);
    vector_int process_ppdu_payload(vector_float::const_iterator iter);
    void process_noise(vector_float::const_iterator iter, vector_float::const_iterator iter_end);
    void utilize_payload();
    tone_map_t calculate_tone_map(float P_t);
    void set_tone_map(tone_map_t tone_map);
    int get_mpdu_payload_size();
    int get_ppdu_payload_length();
    int get_inter_frame_space();
    stats_t get_stats();
    vector_float::const_iterator preamble();
    int max_blocks (tone_mode_t tone_mode);
    void debug(bool debug) {d_debug = debug; return;};

private:
    tx_params_t get_tx_params (const vector_int &mpdu_fc_int);
    void update_frame_control (vector_int &mpdu_fc_int, tx_params_t tx_params, size_t payload_size);
    vector_float create_payload_symbols(const vector_int &payload_bits, pb_size_t pb_size, tone_mode_t tone_mode);
    vector_int encode_payload(const vector_int &payload_bits, pb_size_t pb_size, code_rate_t rate, tone_mode_t tone_mode);
    vector_float create_frame_control_symbol(const vector_int &bitstream);
    static void pack_bitvector(vector_int::const_iterator begin, vector_int::const_iterator end, unsigned char* array);
    static vector_int unpack_into_bitvector (const unsigned char *data, size_t c);    
    static unsigned long crc24(const vector_int &bit_vector);
    static vector_int scrambler(const vector_int& bitstream, int &state);
    static int scrambler_init(void);
    void init_turbo_codec();
    vector_int tc_encoder(const vector_int &bitstream, pb_size_t pb_size, code_rate_t rate);
    vector_int tc_decoder(const vector_float &received_info, const vector_float &received_parity, pb_size_t pb_size, code_rate_t rate);
    static vector_int channel_interleaver(const vector_int& bitstream, const vector_int& parity, pb_size_t pb_size, code_rate_t rate);
    vector_int robo_interleaver(const vector_int& bitstream, tone_mode_t tone_mode);
    tone_info_t calc_robo_tone_info (tone_mode_t tone_mode);
    tone_info_t get_tone_info (tone_mode_t tone_mode);
    void calc_robo_parameters (tone_mode_t tone_mode, unsigned int n_raw, unsigned int &n_copies, unsigned int &bits_in_last_symbol, unsigned int &bits_in_segment, unsigned int &n_pad);
    static vector_int copier(const vector_int& bitstream, int n_carriers, int offset, int start = 0);
    vector_complex modulate(const vector_int& bits, const tone_info_t& tone_info);
    static itpp::bvec to_bvec (const vector_int in);
    static itpp::ivec to_ivec (const vector_int in);
    static vector_int to_vector_int (const itpp::bvec in);
    static vector_int consistuent_encoder(const vector_int& in, pb_size_t pb_size);
    static std::array<vector_int, 3> calc_turbo_interleaver_sequence();
    static vector_int turbo_interleaver(const vector_int &bitstream, pb_size_t pb_size);
    static void puncture(vector_int &bitstream, code_rate_t rate);
    static int pn_generator(int n_bits, int &pn_state);
    static int pn_generator_init(void);
    static bool channel_interleaver_row(const vector_int& bitstream, vector_int::iterator &iter, int step_size, int& row_no, int& rows_done, int& nibble_no, bool wrap = false);
    vector_complex::iterator fft_real(vector_float::const_iterator iter_begin, vector_float::const_iterator iter_end, vector_complex::iterator iter_out);
    vector_float::iterator ifft_real(vector_complex::const_iterator iter_begin, vector_complex::const_iterator iter_end, vector_float::iterator iter_out);
    vector_float calc_preamble();
    static vector_complex calc_syncp_fft(const vector_float &preamble);
    vector_float::iterator append_datastream(vector_float::const_iterator symbol_iter_begin, vector_float::const_iterator symbol_iter_end, vector_float::iterator iter_out, size_t cp_length, float gain=1);
    static unsigned int count_non_masked_carriers(tone_mask_t::const_iterator begin, tone_mask_t::const_iterator end);
    static void update_tone_info_capacity(tone_info_t& tone_info);
    bool get_rx_params (const vector_int &fc_bits, rx_params_t &rx_params);
    static bool crc24_check(const vector_int &bit_vector);
    vector_float::iterator demodulate_symbols (vector_complex::const_iterator iter, vector_complex::const_iterator iter_end, vector_float::iterator soft_bits_iter, const tone_map_t& tone_map, const channel_response &channel_response);
    vector_float::iterator demodulate_soft_bits_helper(int n_bits, float r, float scale, float n0, vector_float::iterator iter);
    vector_float::iterator demodulate_soft_bits(const complex &value, modulation_type_t modulation, float n0, vector_float::iterator iter);
    int qam_demodulate(int v, int l);
    static vector_float combine_copies(vector_float& bitstream, int offset, int n_bits);
    static vector_float channel_deinterleaver(const vector_float& bitstream, vector_float& parity_bitstream, pb_size_t pb_size, code_rate_t rate);
    static bool channel_deinterleaver_row(vector_float::const_iterator& iter, vector_float& out, int step_size, int& row_no, int& rows_done, int& nibble_no, bool wrap = false);
    vector_float robo_deinterleaver(const vector_float& bitstream, int n_raw, tone_mode_t tone_mode);
    inline int calc_phy_block_size(pb_size_t pb_size);
    int calc_fec_block_size(tone_mode_t tone_mode, code_rate_t rate, pb_size_t pb_size);
    static int calc_encoded_block_size(code_rate_t rate, pb_size_t pb_size);
    tone_info_t build_broadcast_tone_info(modulation_type_t modulation = MT_QPSK);
    void create_fftw_vars ();
    vector_complex fft_real_syncp(const vector_float& data);
    void estimate_channel_gain(vector_complex::const_iterator iter, vector_complex::const_iterator iter_end, vector_complex::const_iterator ref_iter, channel_response &channel_response);
    void estimate_channel_phase (vector_complex::const_iterator iter, vector_complex::const_iterator iter_end, vector_complex::const_iterator ref_iter, channel_response &channel_response);
    static std::array<float, NUMBER_OF_CARRIERS*2> create_hamming_window();
    std::vector<spline_set_t> spline(vector_float &x, vector_float &y);
    tone_info_t d_custom_tone_info;
    channel_response d_broadcast_channel_response;
    tones_float d_noise_psd;
    stats_t d_stats;
    rx_params_t d_rx_params;
    vector_float d_rx_soft_bits;
    vector_int d_rx_mpdu_payload;
    vector_complex d_rx_symbols_freq;
    static std::mutex fftw_mtx;
    fftwf_complex *d_ifft_input, *d_fft_output, *d_fft_syncp_output;
    float *d_ifft_output, *d_fft_input, *d_fft_syncp_input;
    fftwf_plan d_fftw_rev_plan, d_fftw_fwd_plan, d_fftw_syncp_fwd_plan;
    itpp::Punctured_Turbo_Codec d_turbo_codec;
};

}; /* namespace light_plc */

#endif /* _LIGHT_PLC_PHY_SERVICE */