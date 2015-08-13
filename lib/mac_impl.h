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

#ifndef INCLUDED_PLC_MAC_IMPL_H
#define INCLUDED_PLC_MAC_IMPL_H

#include <plc/mac.h>

namespace gr {
  namespace plc {

    class mac_impl : public mac
    {
     private:
      boost::shared_ptr<gr::thread::thread> d_thread;
      bool d_finished;
      float d_period_ms;
      int d_count;
      void send_data();
      void run();

     public:
      mac_impl(float period_ms, int count);
      ~mac_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      void set_period(float period_ms) { d_period_ms = period_ms; }
      void set_count(int count) { d_count = count; }
      float period() const { return d_period_ms; }
      int count() const { return d_count; }

      // Overloading these to start and stop the internal thread that
      // periodically produces the message.
      bool start();
      bool stop();
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_MAC_IMPL_H */

