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

#include "plcp_receiver_impl.h"
#include "debug.h"
#include <gnuradio/io_signature.h>
#include <gnuradio/fft/fft.h>

namespace gr {
  namespace plc {

    const int plcp_receiver_impl::SYNCP_SIZE = light_plc::Plcp::SYNCP_SIZE;
    const int plcp_receiver_impl::SYNC_LENGTH = 2 * plcp_receiver_impl::SYNCP_SIZE;
    const int plcp_receiver_impl::PREAMBLE_SIZE = light_plc::Plcp::PREAMBLE_SIZE;
    const int plcp_receiver_impl::FRAME_CONTROL_SIZE = light_plc::Plcp::FRAME_CONTROL_SIZE;
    const float plcp_receiver_impl::THRESHOLD = 0.9;
    const int plcp_receiver_impl::MIN_PLATEAU = 5.5 * plcp_receiver_impl::SYNCP_SIZE; 


    plcp_receiver::sptr
    plcp_receiver::make(bool debug)
    {
      return gnuradio::get_initial_sptr
        (new plcp_receiver_impl(debug));
    }

    /*
     * The private constructor
     */
    plcp_receiver_impl::plcp_receiver_impl(bool debug)
      : gr::block("plcp_receiver",
              gr::io_signature::make(2, 2, sizeof(float)),
              gr::io_signature::make(0, 0, 0)),
        d_debug(debug),
        d_state(SEARCH),
        d_plateau(0),
        d_payload_size(0),
        d_payload_offset(0),
        d_sync_offset(0),
        d_frame_control_offset(0),
        d_preamble_offset(0),
        d_frame_start(0),
        d_output_datastream_offset(0),
        d_output_datastream_len(0)
    {
      message_port_register_out(pmt::mp("out"));

      //light_plc::Plcp::debug(debug);
      d_plcp = light_plc::Plcp();
      
      // Set the correlation filter
      light_plc::VectorFloat syncp (d_plcp.preamble() + SYNCP_SIZE * 7.5, d_plcp.preamble() + SYNCP_SIZE * 8.5);
      std::reverse(syncp.begin(), syncp.end());
      d_fir = new gr::filter::kernel::fir_filter_fff(1, syncp);    
      d_correlation = gr::fft::malloc_float(SYNC_LENGTH); 

      d_preamble = light_plc::VectorFloat(PREAMBLE_SIZE); 
      d_frame_control = light_plc::VectorFloat(FRAME_CONTROL_SIZE);
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
      if (d_state == SYNC) {
        ninput_items_required[0] = SYNC_LENGTH + SYNCP_SIZE - 1;
        ninput_items_required[1] = SYNC_LENGTH + SYNCP_SIZE - 1;
      } else {
        ninput_items_required[0] = noutput_items;
        ninput_items_required[1] = noutput_items;
      }
    }

    int
    plcp_receiver_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
        const float *in0 = (const float *) input_items[0];
        const float *in1 = (const float *) input_items[1];
        int ninput = std::min(ninput_items[0], ninput_items[1]);

        int i = 0;
        switch(d_state) {

        case SEARCH:
          dout << "PLCP Receiver: state = SEARCH, d_plateau = " << d_plateau << std::endl;
          while (i < ninput) {
            if(in0[i] > THRESHOLD) {
              if(d_plateau < MIN_PLATEAU) {
                d_plateau++;
              } else {
                d_state = SYNC;
                d_plateau = 0;
                dout << "PLCP Receiver: Found frame! i = " << i << std::endl;
                break;
              }   
            } else {
              d_plateau = 0;
            }
            d_preamble[d_preamble_offset++] = in1[i];
            d_preamble_offset = d_preamble_offset % PREAMBLE_SIZE;
            i++;
          }
          break;

        case SYNC:
        {
          dout << "PLCP Receiver: state = SYNC, d_sync_offset = " << d_sync_offset << std::endl;
          d_fir->filterN(d_correlation, in1, SYNC_LENGTH);
          int max1_index = 0, max2_index = 0;
          float max1_value = d_correlation[0];
          float max2_value = d_correlation[0];
          for (; i < SYNC_LENGTH; i++) {
            d_preamble[d_preamble_offset++] = in1[i];
            d_preamble_offset = d_preamble_offset % PREAMBLE_SIZE;
            if (d_correlation[i] > max2_value) {
              if (d_correlation[i] > max1_value) {
                max2_value = max1_value;
                max2_index = max1_index;
                max1_value = d_correlation[i];
                max1_index = i;
              } else {
                max2_value = d_correlation[i];
                max2_index = i;
              }
            }
          }
          unsigned int sync_pos = std::max(max1_index, max2_index);
          dout << "PLCP Receiver: state = SYNC, max1 = " << max1_value << ", " << max1_index 
                                            << " max2 = " << max2_value << ", " << max2_index 
                                            << " sync_pos = " << sync_pos << std::endl;           
          d_frame_start = 1.5 * SYNCP_SIZE + sync_pos - i;  // frame begins 1.5 syncp after last min
          if (d_frame_start < 0)
            d_state = RESET; // If sync_pos does not make sense
          else 
            d_state = CHANNEL_ESTIMATE;
          break;
        }

        case CHANNEL_ESTIMATE:
          dout << "PLCP Receiver: state = CHANNEL_ESTIMATE, d_frame_control_offset = " << d_frame_start << std::endl;
          while (i < ninput && d_sync_offset < d_frame_start) {
            d_preamble[d_preamble_offset++] = in1[i];
            d_preamble_offset = d_preamble_offset % PREAMBLE_SIZE;
            d_sync_offset++;
            i++;
          }
          if (d_sync_offset == d_frame_start) {
            light_plc::VectorFloat preamble_aligned (d_preamble.size());
            light_plc::VectorFloat::iterator preamble_aligned_iter (preamble_aligned.begin());
            preamble_aligned_iter = std::copy(d_preamble.begin() + d_preamble_offset, d_preamble.end(), preamble_aligned_iter);
            std::copy(d_preamble.begin(), d_preamble.begin() + d_preamble_offset, preamble_aligned_iter);
            d_plcp.estimateChannel(preamble_aligned.begin(), preamble_aligned.end());
            d_state = COPY_FRAME_CONTROL;
          }
          break;

        case COPY_FRAME_CONTROL:
          dout << "PLCP Receiver: state = COPY_FRAME_CONTROL" << std::endl;
          while (i < ninput) {
            if (d_frame_control_offset < FRAME_CONTROL_SIZE) {
              d_frame_control[d_frame_control_offset] = in1[i];
            } else  {
              d_payload_size = d_plcp.setReceivedFrameControl(d_frame_control.begin());
              if (d_payload_size == -1) {
                d_state = RESET;
              } else {
                d_state = COPY_PAYLOAD;
                d_payload = light_plc::VectorFloat(d_payload_size);
              }
              break;
            }
            i++;
            d_frame_control_offset++;
          }
          break;

        case COPY_PAYLOAD: {
          dout << "PLCP Receiver: state = COPY_PAYLOAD, d_payload_size = " << d_payload_size << " d_payload_offset = " << d_payload_offset << " ninput = " << ninput << std::endl;
          int k = std::min(d_payload_size - d_payload_offset, ninput);
          std::copy(in1, in1 + k, d_payload.begin() + d_payload_offset);
          d_payload_offset += k;
          i += k;
          if (d_payload_offset == d_payload_size) {
            light_plc::VectorInt mpdu_payload = d_plcp.resolveReceivedPayload(d_payload.begin(), light_plc::RATE_1_2);
            std::vector<char> mpdu_payload_byte(mpdu_payload.begin(), mpdu_payload.end());

            dout << "PLCP Receiver: payload resolved. Payload size = " << mpdu_payload.size() << std::endl;
            // dict
            pmt::pmt_t dict = pmt::make_dict();

            // blob
            pmt::pmt_t mpdu_payload_blob = pmt::make_blob(mpdu_payload_byte.data(), mpdu_payload_byte.size());

            // pdu
            message_port_pub(pmt::mp("out"), pmt::cons(dict, mpdu_payload_blob));

            d_state = RESET;
          }
          break;
        }

        case RESET:
          dout << "PLCP Receiver: state = RESET" << std::endl;
          d_frame_control_offset = 0;
          d_sync_offset = 0;
          d_preamble_offset = 0;
          d_frame_control_offset = 0;
          d_payload_size = 0;
          d_payload_offset = 0;
          d_cor.clear();
          d_output_datastream_offset = 0;
          d_output_datastream_len = 0;
          d_state = SEARCH;
          break;
        }

        // Do <+signal processing+>
        // Tell runtime system how many input items we consumed on
        // each input stream.
        consume_each (i);

        dout << "PLCP Receiver: consumed: " << i << std::endl;

        // Tell runtime system how many output items we produced.
        return 0;
    }


  } /* namespace plc */
} /* namespace gr */

