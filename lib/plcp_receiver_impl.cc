/* -*- c++ -*- */
/* 
 * Copyright 2015 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "plcp_receiver_impl.h"
#include "debug.h"

namespace gr {
  namespace plc {

    plcp_receiver::sptr
    plcp_receiver::make(float threshold, unsigned int min_plateau, bool debug)
    {
      return gnuradio::get_initial_sptr
        (new plcp_receiver_impl(threshold, min_plateau, debug));
    }

    /*
     * The private constructor
     */
    plcp_receiver_impl::plcp_receiver_impl(float threshold, unsigned int min_plateau, bool debug)
      : gr::block("plcp_receiver",
              gr::io_signature::make(2, 2, sizeof(float)),
              gr::io_signature::make(1, 1, sizeof(float))),
        d_debug(debug),
        d_state(SEARCH),
        d_plateau(0),
        d_offset(0),
        THRESHOLD(threshold),
        MIN_PLATEAU(min_plateau),
        SYNC_LENGTH(3.5*512) 
    {
      //light_plc::Plcp::debug(debug);
      d_plcp = light_plc::Plcp();
      std::vector<float> syncp (d_plcp.syncp()); 
      d_fir = new gr::filter::kernel::fir_filter_fff(1, syncp);
      d_correlation = gr::fft::malloc_float(3.5*512);
    }

    /*
     * Our virtual destructor.
     */
    plcp_receiver_impl::~plcp_receiver_impl()
    {
      delete d_fir;
      gr::fft::free(d_correlation);
    }

    void
    plcp_receiver_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
        /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    int
    plcp_receiver_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
        const float *in1 = (const float *) input_items[0];
        const float *in2 = (const float *) input_items[1];
        int ninput = std::min(ninput_items[0], ninput_items[1]);

        int i = 0;
        switch(d_state) {

        case SEARCH: 
          while (i < ninput) {
            if(in1[i] > THRESHOLD) {
              if(d_plateau < MIN_PLATEAU) {
                d_plateau++;
              } else {
                d_state = SYNC;
                d_plateau = 0;
                //insert_tag(nitems_written(0));
                dout << "Found frame!" << std::endl;
                break;
              }   
            } else {
              d_plateau = 0;
            }
            i++;
          }
          break;

        case SYNC:
          d_fir->filterN(d_correlation, in2, std::min(SYNC_LENGTH, std::max(ninput - 511, 0)));

          while(i + 511 < ninput) {
            d_cor.push_back(pair<double, int>(d_correlation[i], d_offset));
            i++;
            d_offset++;

            if(d_offset == SYNC_LENGTH) {
              d_cor.sort();
              unsigned int sync_pos = std::max(d_cor[0].second, d_cor[1].second);
              d_frame_start = 1.5*512 - (SYNC_LENGTH - sync_pos);
              if ((d_frame_start < 0) || (std::min(d_cor[0].first, d_cor[1].first) < -THRESHOLD))
              {
                d_state = RESET;
              }

              d_offset = 0;
              d_state = CHANNEL_ESTIMATE;
              break;
            }
          }
          break;
        case CHANNEL_ESTIMATE:
          while (i<ninput) {
            rel = d_offset - d_frame_start;
            if (rel >= 0 && rel < IEEE1901_FRAME_CONTROL_SIZE) {
              frame_control[rel] = in2[i];
            } else if (rel = IEEE1901_FRAME_CONTROL_SIZE) {
              d_plcp.estimateChannel(preamble);
              d_length = d_plcp.parseFrameControl(frame_control);
              state = COPY;
            } else {
              i++;
              d_offset++;
            }
        case RESET:
          //d_copy_left = MAX_SAMPLES;
          break;
        case COPY: break;
        }


        float *out = (float *) output_items[0];

        // Do <+signal processing+>
        // Tell runtime system how many input items we consumed on
        // each input stream.
        consume_each (i);

        // Tell runtime system how many output items we produced.
        return noutput_items;
    }


    void search_frame_start() {

    // sort list (highest correlation first)
    assert(d_cor.size() == SYNC_LENGTH);
    d_cor.sort();

    // copy list in vector for nicer access
    vector<pair<double, int> > vec(d_cor.begin(), d_cor.end());
    d_cor.clear();

    // in case we don't find anything use SYNC_LENGTH
    d_frame_start = SYNC_LENGTH;

    for(int i = 0; i < 3; i++) {
      for(int k = i + 1; k < 4; k++) {
              int diff = abs(get<1>(vec[i]) - get<1>(vec[k]));
              if(diff == 64) {
                      d_frame_start =  max(get<1>(vec[i]), get<1>(vec[k])) + 64;
                      // nice match found, return immediately
                      return;

              // TODO: check if these offsets make sense
              } else if(diff == 63) {
                      d_frame_start = max(get<1>(vec[i]), get<1>(vec[k])) + 63;
              } else if(diff == 65) {
                      d_frame_start = max(get<1>(vec[i]), get<1>(vec[k])) + 64;
              }
      }
    }
}


  } /* namespace plc */
} /* namespace gr */

