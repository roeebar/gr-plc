#include "phy_service.h"
#include "debug.h"
#include "phy_service_qa.h"
#include <algorithm>

char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

int main(int argc, char * argv[]) {
    unsigned int seed = 1444438709;
    bool debug_ = false;
    light_plc::robo_mode_t robo_mode = light_plc::NO_ROBO;
    light_plc::modulation_type modulation = light_plc::QPSK;
    int nblocks = 1;
    float snr = 30;
    bool encode_only = false;
    if(cmdOptionExists(argv, argv+argc, "-help")) {
        std::cout << "Options:\n" 
        << "  -mode MODE          Can be SOF, SOUND, SACK, SOFFILE, RANDOM.\n"
        << "                      Default to RANDOM (100 random tests)\n\n"
        << "  -modulation NUMBER  Set modulation in SOF or SOFFILE modes\n"
        << "                      Default = 1 (QPSK)\n\n"
        << "  -robo-mode NUMBER   Set ROBO mode in SOF, SOUND or SOFFILE modes\n" 
        << "                      Default to 0 (NO_ROBO)\n\n"
        << "  -nblocks NUMBER     Set number of blocks to encode in SOF or SOFFILE modes\n"
        << "                      Default = 1\n\n"
        << "  -snr NUMBER         Set noise according to SNR number in db\n"
        << "                      Default = 30db\n\n"
        << "  -encode_only        Do not try to process the recevied stream\n\n"
        << "  -in_filename NAME   Input file name is SOFFILE mode\n\n" 
        << "  -out_filename NAME  Output file name is SOFFILE mode\n\n"
        << "  -d                  Print debug output\n\n" 
        << "  -seed NUMBER        Use seed number for random values\n" << std::endl;
        return 0;
    }

    if(cmdOptionExists(argv, argv+argc, "-d"))
    {
        debug_ = true;
        std::cout << "Debug = true" << std::endl;
    } else {
        std::cout << "Debug = false" << std::endl;
    }

    char* seed_str = getCmdOption(argv, argv + argc, "-seed");
    if (seed_str != NULL)
        seed = (light_plc::robo_mode_t)atoi(seed_str);

    phy_service_qa tester(debug_, seed);

    char* robo_mode_str = getCmdOption(argv, argv + argc, "-robo-mode");
    if (robo_mode_str != NULL)
        robo_mode = (light_plc::robo_mode_t)atoi(robo_mode_str);

    char* modulation_str = getCmdOption(argv, argv + argc, "-modulation");
    if (modulation_str != NULL)
        modulation = (light_plc::modulation_type)atoi(modulation_str);

    char* nblocks_str = getCmdOption(argv, argv + argc, "-nblocks");
    if (nblocks_str != NULL)
        nblocks = atoi(nblocks_str);

    char* snr_str = getCmdOption(argv, argv + argc, "-snr");
    if (snr_str != NULL)
        snr = atoi(snr_str);

    if(cmdOptionExists(argv, argv+argc, "-encode_only"))
        encode_only = true;

    char* in_filename = getCmdOption(argv, argv + argc, "-in_filename");
    char* out_filename = getCmdOption(argv, argv + argc, "-out_filename");

    char* mode_str = getCmdOption(argv, argv + argc, "-mode");
    if (mode_str == NULL)
        tester.random_test(100, encode_only);
    else if (std::string(mode_str) == "SOF")
        tester.test_sof(RATE_1_2, robo_mode, modulation, nblocks, snr, encode_only);
    else if (std::string(mode_str) == "SOUND")
        tester.test_sound(robo_mode, snr, encode_only);
    else if (std::string(mode_str) == "SOFFILE")
        tester.encode_to_file(RATE_1_2, robo_mode, modulation, 1, in_filename, out_filename);
    else if (std::string(mode_str) == "SACK")
        tester.test_sack();


    //tester.encode_to_file(RATE_1_2, NO_ROBO, QAM1024, 1, "input.bin", "output.bin");
    //tester.test_sound(STD_ROBO, false);
    //tester.test_sof(RATE_1_2, NO_ROBO, QAM1024, 72, false);
    //tester.test_sof(RATE_1_2, STD_ROBO, QPSK, 1, false);
    //tester.test_sack();

    //tester.random_test(100, false);

    // std::ofstream out("output.bin",std::ios_base::binary);
    // float f;
    // if(out.good())
    // {
    //     for (vector_float::iterator iter = datastream.begin(); iter != datastream.end(); iter++) {
    //     	f = *iter;
    // 		out.write((char *)&f,sizeof(float));
    // 	}
    // out.close();
    // }

}
