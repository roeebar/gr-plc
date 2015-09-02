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

#ifndef INCLUDED_PLC_PLCP_RECEIVER_IMPL_H
#define INCLUDED_PLC_PLCP_RECEIVER_IMPL_H

#include <plc/plcp_receiver.h>
#include <lightplc/plcp.h>
#include <list>
#include <gnuradio/filter/fir_filter.h>

namespace gr {
  namespace plc {

    class plcp_receiver_impl : public plcp_receiver
    {

     static const int SYNCP_SIZE;
     static const int SYNC_LENGTH;
     static const int PREAMBLE_SIZE;
     static const int FRAME_CONTROL_SIZE;
     static const float THRESHOLD;
     static const int MIN_PLATEAU; 

    private:
      light_plc::Plcp d_plcp;
      gr::filter::kernel::fir_filter_fff *d_fir;
      const bool d_debug;
      enum {SEARCH, SYNC, CHANNEL_ESTIMATE, COPY_FRAME_CONTROL, COPY_PAYLOAD, RESET} d_state;
      int d_plateau;
      int d_payload_size;
      int d_payload_offset;
      float *d_correlation;
      light_plc::VectorFloat d_preamble;
      light_plc::VectorFloat d_frame_control;
      light_plc::VectorFloat d_payload;
      int d_sync_offset;
      int d_frame_control_offset;
      int d_preamble_offset;
      int d_frame_start;
      light_plc::VectorInt d_output_datastream;
      int d_output_datastream_offset;
      int d_output_datastream_len;

      std::list<std::pair<double, int>> d_cor; 
      std::list<std::pair<double, int>>::iterator d_cor_iter;

     public:
      plcp_receiver_impl(bool debug);
      ~plcp_receiver_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
		       gr_vector_int &ninput_items,
		       gr_vector_const_void_star &input_items,
		       gr_vector_void_star &output_items);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_PLCP_RECEIVER_IMPL_H */

