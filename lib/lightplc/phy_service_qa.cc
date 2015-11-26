#include <time.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include "phy_service_qa.h"

using namespace light_plc;

phy_service_qa::phy_service_qa (bool d_debug, unsigned int seed) {
    encoder = phy_service(d_debug);
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
    
    test_sound(STD_ROBO);

    // Send the rest of the random tests
    for (int i=2; i<=number_of_tests; i++) {

        int test_type = integer_random(100);

        if (test_type < 10) { // sound test
            std::cout << "Test " << i << " (Sound): " << std::endl;           
            robo_mode_t robo_mode;
            if (integer_random(1) == 0)
                robo_mode = STD_ROBO;
            else
                robo_mode = MINI_ROBO;
            if (!test_sound(robo_mode))
                return false;
        } else if (test_type < 30) {  // sack test
            std::cout << "Test " << i << " (SACK): " << std::endl;
            if (!test_sack(encode_only))
                return false;
        } else if (test_type < 100) { // sof test
            //code_rate rate = (code_rate) integer_random(2);
            code_rate rate = RATE_1_2;
            robo_mode_t robo_mode = (robo_mode_t) integer_random(3);
            int number_of_blocks = integer_random(encoder.max_blocks(robo_mode, rate));
            std::cout << "Test " << i << " (SOF): " << std::endl;
            
            if (!test_sof(rate, robo_mode, number_of_blocks))
                return false;
        }
    }
    if (!encode_only) 
        std::cout << "All tests passed!" << std::endl;
    return true;
}

bool phy_service_qa::test_sof(code_rate rate, robo_mode_t robo_mode, int number_of_blocks, float SNRdb, bool encode_only) {

    vector_int payload(520*8*number_of_blocks);

    std::cout << "Encoding rate = " << rate << std::endl;
    std::cout << "ROBO mode = " << robo_mode << std::endl;
    std::cout << "number of blocks = " << number_of_blocks << std::endl;
    std::generate(payload.begin(), payload.end(), binary_random);
    encoder.set_code_rate(rate);
    encoder.set_robo_mode(robo_mode);
    encoder.set_tone_map(d_tone_map);
    vector_float datastream = encoder.create_sof_ppdu(payload);

    float var = add_noise(datastream.begin(), datastream.end(), SNRdb);
    encoder.set_noise_psd(var*2);

    std::cout << "Datastream length = " << datastream.size() << std::endl;
    if (!encode_only) {
        vector_float::const_iterator iter = datastream.begin();
        encoder.process_ppdu_preamble(iter, iter + phy_service::PREAMBLE_SIZE);
        if (encoder.process_ppdu_frame_control(iter += phy_service::PREAMBLE_SIZE) == -1)
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
    vector_float datastream = encoder.create_sack_ppdu(sackd);
    std::cout << "Datastream length = " << datastream.size() << std::endl;

    float var = add_noise(datastream.begin(), datastream.end(), SNRdb);
    encoder.set_noise_psd(var*2);

    if (!encode_only) {
        vector_float::const_iterator iter = datastream.begin();
        encoder.process_ppdu_preamble(iter, iter + phy_service::PREAMBLE_SIZE);
        if (encoder.process_ppdu_frame_control(iter += phy_service::PREAMBLE_SIZE) == -1)
        {
            std::cout << "Failed!" << std::endl;
            return false;
        }
        vector_int return_sackd = encoder.get_sackd();
        if (std::equal(sackd.begin(), sackd.end(), return_sackd.begin())) {
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

bool phy_service_qa::test_sound(robo_mode_t robo_mode, float SNRdb, bool encode_only) {
    std::cout << "ROBO mode = " << robo_mode << std::endl;
   
    vector_float datastream = encoder.create_sound_ppdu(robo_mode);
    std::cout << "Datastream length = " << datastream.size() << std::endl;

    add_noise(datastream.begin(), datastream.end(), SNRdb);

    if (!encode_only) {
        vector_float::const_iterator iter = datastream.begin();
        encoder.process_ppdu_preamble(iter, iter + phy_service::PREAMBLE_SIZE);
        if (encoder.process_ppdu_frame_control(iter += phy_service::PREAMBLE_SIZE) == -1)
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

bool phy_service_qa::encode_to_file (code_rate rate, robo_mode_t robo_mode, int number_of_blocks, std::string input_filename, std::string output_filename) {
    vector_int payload(520*8*number_of_blocks);

    std::cout << "Testing parameters: " << std::endl;
    std::cout << "Encoding rate = " << rate << std::endl;
    std::cout << "ROBO mode = " << robo_mode << std::endl;
    std::cout << "number of blocks = " << number_of_blocks << std::endl;
    std::generate(payload.begin(), payload.end(), binary_random);
    
    encoder.set_code_rate(rate);
    encoder.set_robo_mode(robo_mode);
    vector_float datastream = encoder.create_sof_ppdu(payload);
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
