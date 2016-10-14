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

#ifndef INCLUDED_PLC_impulse_source_H
#define INCLUDED_PLC_impulse_source_H

#include <plc/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace plc {

    /*!
     * \brief <+description of block+>
     * \ingroup plc
     *
     */
    class PLC_API impulse_source : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<impulse_source> sptr;
      virtual void add_noise(float iat, float A, float l, float f, float offset) = 0;
      /*!
       * \brief Return a shared_ptr to a new instance of plc::impulse_source.
       *
       * To avoid accidental use of raw pointers, plc::impulse_source's
       * constructor is in a private implementation
       * class. plc::impulse_source::make is the public interface for
       * creating new instances.
       */
      static sptr make(float samp_rate);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_impulse_source_H */
