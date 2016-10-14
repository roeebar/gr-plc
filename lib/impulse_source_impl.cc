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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "impulse_source_impl.h"
#include <volk/volk.h>

namespace gr {
  namespace plc {

    impulse_source::sptr
    impulse_source::make(float samp_rate)
    {
      return gnuradio::get_initial_sptr
        (new impulse_source_impl(samp_rate));
    }

    /*
     * The private constructor
     */
    impulse_source_impl::impulse_source_impl(float samp_rate)
      : gr::sync_block("impulse_source",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_samp_rate(samp_rate)
    {
      const int alignment_multiple = volk_get_alignment() / sizeof(gr_complex);
      set_alignment(std::max(1, alignment_multiple));
    }

    /*
     * Our virtual destructor.
     */
    impulse_source_impl::~impulse_source_impl()
    {
      for(auto iter = d_impulses.begin(); iter != d_impulses.end(); iter++)
        volk_free(iter->signal);
    }

    int
    impulse_source_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      gr_complex *out = (gr_complex *) output_items[0];
      memset(out, 0, noutput_items * sizeof(gr_complex)); // zero output

      for (size_t j=0; j<d_impulses.size(); j++) {
        int i = 0; // output
        int len = d_impulses[j].length; // impulse length
        int k = d_impulses[j].pos; // impulse last position

        while (i < noutput_items) { // keep filling with noise till no more room
          int l = std::min(len - k, noutput_items - i);
          volk_32f_x2_add_32f((float*)(out + i), (float*)(out + i), (float*)(d_impulses[j].signal + k), l * 2);
          k = (k + l) % len;
          i+=l;
        }

        d_impulses[j].pos = k; // update position
      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    void impulse_source_impl::add_noise(float iat, float A, float l, float f, float offset) {
      std::cout <<  "I" << d_impulses.size() << ": IAT=" << iat <<  " A=" << A <<" l=" << l  << " f=" << f << " offset=" << offset << std::endl;
      d_impulses.push_back(create_impulse(iat, A, l, f, offset));
    }

    impulse_source_impl::impulse_t impulse_source_impl::create_impulse(float iat, float A, float l, float f, float offset) {
      impulse_t impulse;

      size_t N = (size_t)(iat * d_samp_rate);
      size_t samples_offset = floor(offset * N);
      std::vector<float> t(N);
      for (size_t i=0; i<N; i++)
        t[i] = i/d_samp_rate;

      impulse.signal = (gr_complex*)volk_malloc(sizeof(gr_complex)*N, volk_get_alignment());
      int j = 0;

      for (size_t i=N-samples_offset; i<N; i++) {
        impulse.signal[j++] = A * std::exp(gr_complex(-l * t[i], 2 * M_PI * f * t[i]));
      }
      for (size_t i=0; i<N-samples_offset; i++) {
        impulse.signal[j++] = A * std::exp(gr_complex(-l * t[i], 2 * M_PI * f * t[i]));
      }
      impulse.length = N;
      impulse.pos = 0;
      return impulse;
    }

  } /* namespace plc */
} /* namespace gr */
