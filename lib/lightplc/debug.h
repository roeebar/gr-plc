/*
 * Gr-plc - IEEE 1901 module for GNU Radio
 * Copyright (C) 2016 Roee Bar <roeeb@ece.ubc.ca>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#ifndef DEBUG_H
#define DEBUG_H
#include  <iomanip>

//#define NDEBUG

#define PRINT_VAR(x)            std::cout << #x << ":" << x << std::endl;
#define PRINT_VECTOR(x)         std::cout << #x << " (" << x.size() << "):";                                            \
                                for( unsigned int debug_i = 0; debug_i<x.size(); ++debug_i)                             \
                                    std::cout << x.at(debug_i)<<' ';                                                    \
                                std::cout << std::endl;

#define PRINT_VECTOR_RANGE(name,x,y)   std::cout << name << " (" << (y)-(x) << "):";                                    \
                                { unsigned int debug_i=0;                                                               \
                                while (x+debug_i != y)                                                                  \
                                    std::cout << *(x+debug_i++) <<' ';                                                  \
                                std::cout << std::endl; }                                                               \


#define PRINT_VECTORINT_PACK(x) std::ios state(NULL);                                                                   \
                                state.copyfmt(std::cout);                                                               \
                                std::cout << #x << " (" << x.size() << "):";                                            \
                                vector_int::const_iterator iter=x.begin();                                              \
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
#  define DEBUG_VECTOR_RANGE(name,x,y) if (d_debug) {PRINT_VECTOR_RANGE(name,x,y)}
#else
#  define DEBUG_VAR(x) do {} while (0);
#  define DEBUG_VECTOR(x) do {} while (0);
#  define DEBUG_ECHO(x) do {} while (0);
#  define DEBUG_VECTORINT_PACK(x) do {} while (0);
#  define DEBUG_VECTOR_RANGE(name,x,y) do {} while (0);
#endif

#define dout d_debug && std::cout

#endif /* DEBUG_H */
