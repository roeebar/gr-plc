#include "phy_service.h"
#include "debug.h"
#include "qa_phy_service.h"
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
    light_plc::tone_mode_t tone_mode = light_plc::TM_NO_ROBO;
    int nblocks = 1;
    float snr = 30;
    bool encode_only = false;
    if(cmdOptionExists(argv, argv+argc, "-help")) {
        std::cout << "Options:\n"
        << "  -mode MODE          Can be SOF, SOUND, SACK, SOFFILE, RANDOM.\n"
        << "                      Default to RANDOM (100 random tests)\n"
        << "  -robo-mode NUMBER   Set ROBO mode in SOF, SOUND or SOFFILE modes\n"
        << "                      Default to 3 (TM_NO_ROBO)\n"
        << "  -nblocks NUMBER     Set number of blocks to encode in SOF or SOFFILE modes\n"
        << "                      Default = 1\n"
        << "  -snr NUMBER         Set noise according to SNR number in db\n"
        << "                      Default = 30db\n"
        << "  -encode_only        Do not try to process the recevied stream\n"
        << "  -in_filename NAME   Input file name is SOFFILE mode\n"
        << "  -out_filename NAME  Output file name is SOFFILE mode\n"
        << "  -d                  Print debug output\n"
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
        seed = (light_plc::tone_mode_t)atoi(seed_str);

    qa_phy_service tester(debug_, seed);

    char* tone_mode_str = getCmdOption(argv, argv + argc, "-robo-mode");
    if (tone_mode_str != NULL)
        tone_mode = (light_plc::tone_mode_t)atoi(tone_mode_str);

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
    else if (std::string(mode_str) == "SOF") {
        tester.test_sound(TM_STD_ROBO, snr, encode_only);
        tester.test_sof(tone_mode, nblocks, snr, encode_only);
    }
    else if (std::string(mode_str) == "SOUND")
        tester.test_sound(tone_mode, snr, encode_only);
    else if (std::string(mode_str) == "SOFFILE")
        tester.encode_to_file(tone_mode, 1, in_filename, out_filename);
    else if (std::string(mode_str) == "SACK")
        tester.test_sack();


    //tester.encode_to_file(RATE_1_2, TM_NO_ROBO, QAM1024, 1, "input.bin", "output.bin");
    //tester.test_sound(TM_STD_ROBO, false);
    //tester.test_sof(RATE_1_2, TM_NO_ROBO, QAM1024, 72, false);
    //tester.test_sof(RATE_1_2, TM_STD_ROBO, QPSK, 1, false);
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
