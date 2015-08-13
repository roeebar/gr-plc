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


#ifndef INCLUDED_PLC_MAC_H
#define INCLUDED_PLC_MAC_H

#include <plc/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace plc {

    /*!
     * \brief <+description of block+>
     * \ingroup plc
     *
     */
    class PLC_API mac : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<mac> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of plc::mac.
       *
       * To avoid accidental use of raw pointers, plc::mac's
       * constructor is in a private implementation
       * class. plc::mac::make is the public interface for
       * creating new instances.
       */
      static sptr make(float period_ms, int count);
      virtual void set_period(float period_ms) = 0;
      virtual float period() const = 0;
      virtual void set_count(int count) = 0;
      virtual int count() const = 0;

    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_MAC_H */

