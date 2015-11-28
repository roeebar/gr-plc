#include <time.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include "phy_service_qa.h"

using namespace light_plc;

phy_service_qa::phy_service_qa (bool d_debug, unsigned int seed) {
    encoder = phy_service();
    encoder.debug(d_debug);
    if (seed == 0)
        seed = time(NULL);
    std::cout << "Seed = " << seed << std::endl;
    srand (seed);
    //srand(1);
}

int binary_random() { return ((rand() % 2) == 0); }
int phy_service_qa::integer_random(int max) { return (rand() % (max+1)); }

bool phy_service_qa::random_test(int number_of_tests, bool encode_only) {
    // First send sounding
    std::cout << "Test 1 (Sound):" << std::endl;
    
    test_sound(TM_STD_ROBO);

    // Send the rest of the random tests
    for (int i=2; i<=number_of_tests; i++) {

        int test_type = integer_random(100);

        if (test_type < 10) { // sound test
            std::cout << "Test " << i << " (Sound): " << std::endl;           
            tone_mode_t tone_mode;
            if (integer_random(1) == 0)
                tone_mode = TM_STD_ROBO;
            else
                tone_mode = TM_MINI_ROBO;
            if (!test_sound(tone_mode))
                return false;
        } else if (test_type < 30) {  // sack test
            std::cout << "Test " << i << " (SACK): " << std::endl;
            if (!test_sack(encode_only))
                return false;
        } else if (test_type < 100) { // sof test
            //core_rate_t rate = (core_rate_t) integer_random(2);
            core_rate_t rate = RATE_1_2;
            tone_mode_t tone_mode;
            int mode = integer_random(3);
            switch (mode){
                case 0: tone_mode = TM_STD_ROBO; break;
                case 1: tone_mode = TM_MINI_ROBO; break;
                case 2: tone_mode = TM_HS_ROBO; break;
                case 3: tone_mode = TM_NO_ROBO; break;
            }
            int number_of_blocks = integer_random(encoder.max_blocks(tone_mode));
            std::cout << "Test " << i << " (SOF): " << std::endl;
            
            if (!test_sof(rate, tone_mode, number_of_blocks))
                return false;
        }
    }
    if (!encode_only) 
        std::cout << "All tests passed!" << std::endl;
    return true;
}

bool phy_service_qa::test_sof(core_rate_t rate, tone_mode_t tone_mode, int number_of_blocks, float SNRdb, bool encode_only) {

    vector_int payload(520*8*number_of_blocks);

    std::cout << "Encoding rate = " << rate << std::endl;
    std::cout << "ROBO mode = " << tone_mode << std::endl;
    std::cout << "number of blocks = " << number_of_blocks << std::endl;
    std::generate(payload.begin(), payload.end(), binary_random);
    encoder.set_tone_map(d_tone_map);
    pb_size_t pb_size;
    (payload.size() > 136*8) ? pb_size = PB520 : pb_size = PB136;
    vector_int fc = create_sof_frame_control(tone_mode, pb_size);
    vector_float datastream = encoder.create_ppdu(fc, payload);

    float var = add_noise(datastream.begin(), datastream.end(), SNRdb);
    encoder.set_noise_psd(var*2);

    std::cout << "Datastream length = " << datastream.size() << std::endl;
    if (!encode_only) {
        vector_float::const_iterator iter = datastream.begin();
        encoder.process_ppdu_preamble(iter, iter + phy_service::PREAMBLE_SIZE);
        if (encoder.process_ppdu_frame_control(iter += phy_service::PREAMBLE_SIZE) == false)
        {
            std::cout << "Failed!" << std::endl;
            return false;
        }
        vector_int return_payload = encoder.process_ppdu_payload(iter += phy_service::FRAME_CONTROL_SIZE);
        encoder.process_noise(iter, iter + encoder.get_inter_frame_space());
        if (std::equal(payload.begin(), payload.end(), return_payload.begin())) {
            std::cout << "Passed." << std::endl << std::endl;
            return true;
        } else {
            std::cout << "Failed!" << std::endl;
            return false;
        }
    } else {
        return true;
    }
}

bool phy_service_qa::test_sack(float SNRdb, bool encode_only) {
    vector_int sackd(80);
    std::generate(sackd.begin(), sackd.end(), binary_random);
    vector_int fc = create_sack_frame_control(sackd);
    vector_float datastream = encoder.create_ppdu(fc);
    std::cout << "Datastream length = " << datastream.size() << std::endl;

    float var = add_noise(datastream.begin(), datastream.end(), SNRdb);
    encoder.set_noise_psd(var*2);

    if (!encode_only) {
        vector_float::const_iterator iter = datastream.begin();
        encoder.process_ppdu_preamble(iter, iter + phy_service::PREAMBLE_SIZE);
        vector_int frame_control;
        if (encoder.process_ppdu_frame_control(iter += phy_service::PREAMBLE_SIZE, frame_control) == false)
        {
            std::cout << "Failed!" << std::endl;
            return false;
        }
        vector_int return_sackd(IEEE1901_FRAME_CONTROL_SACK_SACKD_WIDTH);
        for (int i=0; i<IEEE1901_FRAME_CONTROL_SACK_SACKD_WIDTH; i++)
            return_sackd[i] = frame_control[IEEE1901_FRAME_CONTROL_SACK_SACKD_OFFSET + i];
        if (std::equal(sackd.begin(), sackd.end(), return_sackd.begin())) {
            std::cout << "Passed." << std::endl << std::endl;
            return true;
        } else {
            std::cout << "Failed sackd!" << std::endl;
            return false;
        }
    } else {
        return true;
    }
}

bool phy_service_qa::test_sound(tone_mode_t tone_mode, float SNRdb, bool encode_only) {
    std::cout << "ROBO mode = " << tone_mode << std::endl;
    vector_int mpdu_payload;
    if (tone_mode == TM_STD_ROBO)
        mpdu_payload = vector_int(520*8);
    else
        mpdu_payload = vector_int(136*8);
    vector_int fc = create_sound_frame_control(tone_mode);
    vector_float datastream = encoder.create_ppdu(fc, mpdu_payload);
    std::cout << "Datastream length = " << datastream.size() << std::endl;

    add_noise(datastream.begin(), datastream.end(), SNRdb);

    if (!encode_only) {
        vector_float::const_iterator iter = datastream.begin();
        encoder.process_ppdu_preamble(iter, iter + phy_service::PREAMBLE_SIZE);
        if (encoder.process_ppdu_frame_control(iter += phy_service::PREAMBLE_SIZE) == false)
        {
            std::cout << "Failed!" << std::endl;
            return false;
        }
        vector_int return_payload = encoder.process_ppdu_payload(iter += phy_service::FRAME_CONTROL_SIZE);
        iter += encoder.get_ppdu_payload_length();
        encoder.process_noise(iter, iter + encoder.get_inter_frame_space());
        d_tone_map = encoder.calculate_tone_map(0.001);
        std::cout << "Passed." << std::endl << std::endl;
        return true;
    } else {
        return true;
    }
}

bool phy_service_qa::encode_to_file (core_rate_t rate, tone_mode_t tone_mode, int number_of_blocks, std::string input_filename, std::string output_filename) {
    vector_int payload(520*8*number_of_blocks);

    std::cout << "Testing parameters: " << std::endl;
    std::cout << "Encoding rate = " << rate << std::endl;
    std::cout << "ROBO mode = " << tone_mode << std::endl;
    std::cout << "number of blocks = " << number_of_blocks << std::endl;
    std::generate(payload.begin(), payload.end(), binary_random);
    pb_size_t pb_size;
    (payload.size() > 136*8) ? pb_size = PB520 : pb_size = PB136;
    vector_int fc = create_sof_frame_control(tone_mode, pb_size);
    vector_float datastream = encoder.create_ppdu(fc, payload);    
    std::cout << "Datastream length = " << datastream.size() << std::endl;

    std::ofstream in(input_filename,std::ios_base::binary);
    vector_int::value_type v;
    if(!in.good())
        return false;

    for (vector_int::iterator iter = payload.begin(); iter != payload.end(); iter++) {
        v = *iter;
        in.write((char *)&v, sizeof(vector_float::value_type));
    }
    in.close();

    std::ofstream out(output_filename,std::ios_base::binary);
    vector_float::value_type f;
    if(!out.good())
        return false;
    for (vector_float::iterator iter = datastream.begin(); iter != datastream.end(); iter++) {
        f = *iter;
        out.write((char *)&f, sizeof(vector_float::value_type));
    }
    out.close();
    return true;
}

float phy_service_qa::add_noise(vector_float::iterator iter_begin, vector_float::iterator iter_end, float SNRdb) {
    // Calculating signal variance    
    float datastream_var = 0;
    int size = iter_end - iter_begin;
    for (vector_float::const_iterator iter = iter_begin; iter != iter_end; iter++)
        datastream_var += (*iter)*(*iter)/size;

    // Adding the desired noise based on SNR requirements
    float SNR = std::pow(10,SNRdb/10);
    float var = datastream_var/SNR;
    std::normal_distribution<float> n(0,std::sqrt(var));
    std::default_random_engine g;
    for (vector_float::iterator iter = iter_begin; iter != iter_end; iter++) {
        float y = n(g);
        *iter = *iter + y;
    }
    std::cout << "SNR = " << SNRdb << std::endl;
    return var;
}

vector_int phy_service_qa::create_sof_frame_control (tone_mode_t tone_mode, pb_size_t pb_size)  {
    vector_int frame_control(IEEE1901_FRAME_CONTROL_NBITS,0);

    // Set the pbsz bit
    if (pb_size == PB520)
        set_field(frame_control, IEEE1901_FRAME_CONTROL_SOF_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOF_PBSZ_WIDTH, 0);
    else
        set_field(frame_control, IEEE1901_FRAME_CONTROL_SOF_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOF_PBSZ_WIDTH, 1);

    // Set delimiter type to SOF    
    set_field(frame_control, IEEE1901_FRAME_CONTROL_DT_IH_OFFSET, IEEE1901_FRAME_CONTROL_DT_IH_WIDTH, 1);
    
    // Set tone map index
    int tmi = 0;
    switch (tone_mode) {
        case TM_STD_ROBO: tmi=0; break;
        case TM_HS_ROBO: tmi=1; break;
        case TM_MINI_ROBO: tmi=2; break;
        case TM_NO_ROBO: tmi=3; break;
    }
    set_field(frame_control, IEEE1901_FRAME_CONTROL_SOF_TMI_OFFSET, IEEE1901_FRAME_CONTROL_SOF_TMI_WIDTH, tmi);
    
    return frame_control;
}

vector_int phy_service_qa::create_sound_frame_control (tone_mode_t tone_mode)  {
    vector_int frame_control(IEEE1901_FRAME_CONTROL_NBITS,0);

    // Set the pbsz bit
    if (tone_mode == TM_STD_ROBO)
        set_field(frame_control, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_WIDTH, 0);
    else
        set_field(frame_control, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_OFFSET, IEEE1901_FRAME_CONTROL_SOUND_PBSZ_WIDTH, 1);

    // Set delimiter type to Sound    
    set_field(frame_control, IEEE1901_FRAME_CONTROL_DT_IH_OFFSET, IEEE1901_FRAME_CONTROL_DT_IH_WIDTH, 4);

    return frame_control;
}

vector_int phy_service_qa::create_sack_frame_control (const vector_int &sackd)  {
    vector_int frame_control(IEEE1901_FRAME_CONTROL_NBITS,0);

    // Set delimiter type to SACK
    set_field(frame_control, IEEE1901_FRAME_CONTROL_DT_IH_OFFSET, IEEE1901_FRAME_CONTROL_DT_IH_WIDTH, 2);

    // SACK version number
    set_field(frame_control, IEEE1901_FRAME_CONTROL_SACK_SVN_OFFSET, IEEE1901_FRAME_CONTROL_SACK_SVN_WIDTH, 0);

    // Assign the SACK data bits
    int j = 0;
    for (vector_int::const_iterator iter = sackd.begin(); iter != sackd.end(); iter++)
        frame_control[IEEE1901_FRAME_CONTROL_SACK_SACKD_OFFSET + j++] = *iter;

    return frame_control;
}
