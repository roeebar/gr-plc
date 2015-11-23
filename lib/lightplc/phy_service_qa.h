#include "phy_service.h"

using namespace light_plc;

class phy_service_qa {

public:
	phy_service_qa (bool debug_ = false, unsigned int seed = 0);
    bool random_test(int number_of_tests, bool encode_only = false);
    bool test_sof(code_rate rate, robo_mode_t robo_mode, modulation_type modulation, int number_of_blocks, float SNRdb = 30, bool encode_only = false);
	bool test_sack(float SNRdb = 30, bool encode_only = false);
	bool test_sound(robo_mode_t robo_mode, float SNRdb = 30, bool encode_only = false);
	float add_noise(vector_float::iterator iter_begin, vector_float::iterator iter_end, float SNRdb);
    bool encode_to_file(code_rate rate, robo_mode_t robo_mode, modulation_type modulation, int number_of_blocks, std::string input_filename, std::string output_filename);
    phy_service encoder;

private: 
    int integer_random(int max);
    bool debug_;
};