#ifndef DEBUG_H
#define DEBUG_H


#define dout d_debug_level && std::cout
#define PRINT_VECTOR(x) std::cout << #x << " (" << x.size() << "):"; for( unsigned int debug_i = 0; debug_i<x.size(); ++debug_i) std::cout << x.at(debug_i)<<' '; std::cout << std::endl;

#define PRINT_INFO_VECTOR(x,name)   if (d_info) {                                           \
                                    std::cout << "'" << d_name << "'; " << name  << " = ["; \
                                    for (auto iter=x.begin(); iter != x.end(); iter++)      \
                                        std::cout << *iter << ",";                          \
                                    std::cout << "];" << std::endl; }

#define PRINT_INFO_VAR(x,name)      if (d_info) {std::cout << "'" << d_name << "'; " << (name) << " = " << (x) << ";" << std::endl;}


#endif /* DEBUG_H */
