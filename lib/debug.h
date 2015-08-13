#ifndef DEBUG_H
#define DEBUG_H


#define dout d_debug && std::cout
#define PRINT_VECTOR(x) std::cout << #x << " (" << x.size() << "):"; for( unsigned int debug_i = 0; debug_i<x.size(); ++debug_i) std::cout << x.at(debug_i)<<' '; std::cout << std::endl;


#endif /* DEBUG_H */