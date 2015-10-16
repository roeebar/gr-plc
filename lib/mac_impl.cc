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
#include "mac_impl.h"

namespace gr {
  namespace plc {

    mac::sptr
    mac::make(float period_ms, int count)
    {
      return gnuradio::get_initial_sptr
        (new mac_impl(period_ms, count));
    }

    /*
     * The private constructor
     */
    mac_impl::mac_impl(float period_ms, int count)
      : gr::block("mac",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(0, 0, 0)),
              d_finished(false),
              d_period_ms(period_ms),
              d_count(count)
    {
      message_port_register_out(pmt::mp("mac out"));
    }

    /*
     * Our virtual destructor.
     */
    mac_impl::~mac_impl()
    {
    }

    void
    mac_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
        /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    bool mac_impl::start()
    {
      // NOTE: d_finished should be something explicitely thread safe. But since
      // nothing breaks on concurrent access, I'll just leave it as bool.
      d_finished = false;
      d_thread = boost::shared_ptr<gr::thread::thread>
        (new gr::thread::thread(boost::bind(&mac_impl::run, this)));

      return block::start();
    }

    bool mac_impl::stop()
    {
      // Shut down the thread
      d_finished = true;
      d_thread->interrupt();
      d_thread->join();

      return block::stop();
    }

    void mac_impl::run()
    {
      while(!d_finished && d_count) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(d_period_ms));
        if(d_finished) {
          return;
        }
        send_data();
      }
      message_port_pub(pmt::mp("mac out"), pmt::PMT_EOF);
    }

    int binaryRandom() { return ((rand() % 2) == 0); }

    void mac_impl::send_data (void)
    {
      // make MAC frame      
      //char *mpdu_payload;
      int number_of_blocks = 2;
      int mpdu_payload_length = 520*8*number_of_blocks;
      std::vector<char> mpdu_payload(mpdu_payload_length);
      std::generate(mpdu_payload.begin(), mpdu_payload.end(), binaryRandom);

      // dict
      pmt::pmt_t dict = pmt::make_dict();
      dict = pmt::dict_add(dict, pmt::mp("type"), pmt::mp("sof"));

      // blob
      pmt::pmt_t mpdu_payload_blob = pmt::make_blob(mpdu_payload.data(), mpdu_payload.size());

      // pdu
      message_port_pub(pmt::mp("mac out"), pmt::cons(dict, mpdu_payload_blob));
      d_count--;
    }
  } /* namespace plc */
} /* namespace gr */

