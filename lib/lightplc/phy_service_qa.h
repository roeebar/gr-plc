#include "phy_service.h"

using namespace light_plc;

class phy_service_qa {

public:
	phy_service_qa (bool debug_ = false, unsigned int seed = 0);
    bool random_test(int number_of_tests, bool encode_only = false);
    bool test_sof(code_rate rate, RoboMode robo_mode, modulation_type modulation, int number_of_blocks, bool encode_only = false);
	bool test_sack(bool encode_only = false);
	bool test_sound(RoboMode robo_mode, bool encode_only);
	float add_noise(vector_float::iterator iter_begin, vector_float::iterator iter_end);
    bool encode_to_file(code_rate rate, RoboMode robo_mode, modulation_type modulation, int number_of_blocks, std::string input_filename, std::string output_filename);
    phy_service encoder;

private: 
    int integer_random(int max);
    bool debug_;
};