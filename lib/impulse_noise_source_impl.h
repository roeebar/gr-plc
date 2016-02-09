/* -*- c++ -*- */

#ifndef INCLUDED_PLC_IMPULSE_NOISE_SOURCE_IMPL_H
#define INCLUDED_PLC_IMPULSE_NOISE_SOURCE_IMPL_H

#include <plc/impulse_noise_source.h>
#include <vector>

namespace gr {
  namespace plc {

    class impulse_noise_source_impl : public impulse_noise_source
    {
     private:   	
     	typedef struct impulse_t {
     		std::vector<gr_complex> signal;
     		int pos;
     	} impulse_t;

     	float d_samp_rate;
     	std::vector<impulse_t> d_impulses;
		impulse_t create_impulse(float iat, float A, float l, float f, float offset);

     public:
      impulse_noise_source_impl(float samp_rate);
      ~impulse_noise_source_impl();
	  void add_noise(float iat, float A, float l, float f, float offset);
      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_IMPULSE_NOISE_SOURCE_IMPL_H */

