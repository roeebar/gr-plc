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

#ifndef INCLUDED_PLC_impulse_source_IMPL_H
#define INCLUDED_PLC_impulse_source_IMPL_H

#include <plc/impulse_source.h>
#include <vector>

namespace gr {
  namespace plc {

    class impulse_source_impl : public impulse_source
    {
     private:
     	typedef struct impulse_t {
     		//std::vector<gr_complex> signal;
        gr_complex* signal;
        int length;
     		int pos;
     	} impulse_t;

     	float d_samp_rate;
     	std::vector<impulse_t> d_impulses;
		impulse_t create_impulse(float iat, float A, float l, float f, float offset);

     public:
      impulse_source_impl(float samp_rate);
      ~impulse_source_impl();
	  void add_noise(float iat, float A, float l, float f, float offset);
      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_impulse_source_IMPL_H */
