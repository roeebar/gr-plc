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

#ifndef INCLUDED_PLC_PHY_TX_H
#define INCLUDED_PLC_PHY_TX_H

#include <plc/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace plc {

    /*!
     * \brief <+description of block+>
     * \ingroup plc
     *
     */
    class PLC_API phy_tx : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<phy_tx> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of plc::phy_tx.
       *
       * To avoid accidental use of raw pointers, plc::phy_tx's
       * constructor is in a private implementation
       * class. plc::phy_tx::make is the public interface for
       * creating new instances.
       */
      static sptr make(int log_level);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_PHY_TX_H */
