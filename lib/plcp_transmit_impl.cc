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
#include "plcp_transmit_impl.h"
#include "debug.h"

namespace gr {
  namespace plc {

    plcp_transmit::sptr
    plcp_transmit::make(bool debug)
    {
      return gnuradio::get_initial_sptr
        (new plcp_transmit_impl(debug));
    }

    /*
     * The private constructor
     */
    plcp_transmit_impl::plcp_transmit_impl(bool debug)
      : gr::block("plcp_transmit",
             gr::io_signature::make(0, 0, 0),
             gr::io_signature::make(1, 1, sizeof(float))),
             d_debug (debug),
             d_datastream_offset(0),
             d_datastream_len(0)
    {
      message_port_register_in(pmt::mp("in"));
      //light_plc::Plcp::debug(debug);
      d_plcp = light_plc::Plcp();
    }

    /*
     * Our virtual destructor.
     */
    plcp_transmit_impl::~plcp_transmit_impl()
    {
    }

    void
    plcp_transmit_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
        /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    int
    plcp_transmit_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      float *out = (float *) output_items[0];

      dout << "PLCP Transmitter: called offset: " << d_datastream_offset <<
              "   length: " << d_datastream_len << std::endl;

      while(!d_datastream_offset) {
        pmt::pmt_t msg(delete_head_blocking(pmt::intern("in")));

        if(pmt::is_eof_object(msg)) {
                dout << "PLCP Transmitter: exiting" << std::endl;
                return -1;
        }

        if(pmt::is_pair(msg)) {
          dout << "PLCP Transmitter: received new message" << std::endl;
          gr::thread::scoped_lock lock(d_mutex);

          size_t mpdu_payload_length = pmt::blob_length(pmt::cdr(msg));
          const char* mpdu_payload = reinterpret_cast<const char *>(pmt::blob_data(pmt::cdr(msg)));
          //const uint8_t *mpdu = pmt::u8vector_elements(pmt::cdr(msg), mpdu_payload_length);

          light_plc::VectorInt mpdu_bits = light_plc::VectorInt(mpdu_payload, mpdu_payload + mpdu_payload_length);
          d_datastream = d_plcp.createDatastream(mpdu_bits, light_plc::STD_ROBO, light_plc::RATE_1_2);

          d_datastream_len = d_datastream.size();

          // add tags
          pmt::pmt_t key = pmt::string_to_symbol("packet_len");
          pmt::pmt_t value = pmt::from_long(d_datastream_len);
          pmt::pmt_t srcid = pmt::string_to_symbol(alias());
          add_item_tag(0, nitems_written(0), key, value, srcid);

          add_item_tag(0, nitems_written(0), pmt::mp("mpdu_len"),
                          pmt::from_long(mpdu_payload_length), srcid);

          break;
        }
      }

      int i = std::min(noutput_items, d_datastream_len - d_datastream_offset);
      std::memcpy(out, &d_datastream[d_datastream_offset], sizeof(light_plc::VectorFloat::value_type)*i);

      d_datastream_offset += i;
      dout << "PLCP Transmitter: produced: " << i << std::endl;

      if(d_datastream_offset == d_datastream_len) {
              d_datastream_offset = 0;
              d_datastream_len = 0;
              dout << "PLCP Transmitter: packet sent!" << std::endl;
      }

      // Tell runtime system how many output items we produced.
      return i;
    }
  } /* namespace plc */
} /* namespace gr */

