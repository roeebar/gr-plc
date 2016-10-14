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

#ifndef INCLUDED_PLC_APP_OUT_IMPL_H
#define INCLUDED_PLC_APP_OUT_IMPL_H

#include <plc/app_out.h>

namespace gr {
  namespace plc {

    class app_out_impl : public app_out
    {

     static const int PAYLOAD_SIZE = 1516;

     private:
      unsigned char d_mac_payload[PAYLOAD_SIZE];
      pmt::pmt_t d_mac_payload_pmt;
      int d_mac_payload_offset;
      bool d_payload_sent;
      long int d_time_begin;
      long int d_total_bytes;
      int d_log_level;

     public:
      app_out_impl(int debug_level);
      ~app_out_impl();

      void send_payload();
      bool start();
      bool stop();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
		       gr_vector_int &ninput_items,
		       gr_vector_const_void_star &input_items,
		       gr_vector_void_star &output_items);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_APP_OUT_IMPL_H */
