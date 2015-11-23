#ifndef DEBUG_H
#define DEBUG_H
#include  <iomanip>

//#define NDEBUG

#define PRINT_VAR(x)            std::cout << #x << ":" << x << std::endl;
#define PRINT_VECTOR(x)         std::cout << #x << " (" << x.size() << "):";                                            \
                                for( unsigned int debug_i = 0; debug_i<x.size(); ++debug_i)                             \
                                    std::cout << x.at(debug_i)<<' ';                                                    \
                                std::cout << std::endl;                                                                 

#define PRINT_VECTORINT_PACK(x) std::ios state(NULL);                                                                   \
                                state.copyfmt(std::cout);                                                               \
                                std::cout << #x << " (" << x.size() << "):";                                            \
                                vector_int::const_iterator iter=x.begin();                                               \
                                while (iter!=x.end()) {                                                                 \
                                    unsigned char byte=0;                                                               \
                                    for (int offset=7; offset>=0; offset --) {                                          \
                                        byte |= (*iter << offset);                                                      \
                                        iter++;                                                                         \
                                    }                                                                                   \
                                    std::cout << std::setfill('0') << std::setw(2) << std::hex << (unsigned int)byte;   \
                                }                                                                                       \
                                std::cout << std::endl;                                                                 \
                                std::cout.copyfmt(state);

#define ECHO(x) (std::cout << x << std::endl);

#ifndef NDEBUG
#  define DEBUG_VAR(x) if (d_debug) {PRINT_VAR(x)}
#  define DEBUG_VECTOR(x) if (d_debug) {PRINT_VECTOR(x)}
#  define DEBUG_ECHO(x) if (d_debug) {(std::cout << x << std::endl);}
#  define DEBUG_VECTORINT_PACK(x) if (d_debug) {PRINT_VECTORINT_PACK(x)}
#else
#  define DEBUG_VAR(x) do {} while (0);
#  define DEBUG_VECTOR(x) do {} while (0);
#  define DEBUG_ECHO(x) do {} while (0);
#  define DEBUG_VECTORINT_PACK(x) do {} while (0);
#endif

#define dout d_debug && std::cout

#endif /* DEBUG_H */