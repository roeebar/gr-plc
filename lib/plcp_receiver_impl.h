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
#include <plcp.h>
#include <gnuradio/filter/fir_filter.h>


namespace gr {
  namespace plc {

    class plcp_receiver_impl : public plcp_receiver
    {
     private:
      light_plc::Plcp d_plcp;
      gr::filter::kernel::fir_filter_fff *d_fir;
      const bool d_debug;
      enum {SEARCH, SYNC, COPY, RESET} d_state;
      int d_copy_left;
      unsigned int d_plateau;
      float *d_correlation;
      unsigned int d_offset;
      const double THRESHOLD;
      const unsigned int MIN_PLATEAU;


     public:
      plcp_receiver_impl(float threshold, unsigned int min_plateau, bool debug);
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

