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

#ifndef INCLUDED_PLC_PHY_TX_IMPL_H
#define INCLUDED_PLC_PHY_TX_IMPL_H

#include <plc/phy_tx.h>
#include <lightplc/phy_service.h>
#include <string>

namespace gr {
  namespace plc {

    class phy_tx_impl : public phy_tx
    {
     private:
      static const int SILENCE_PERIOD;

      light_plc::phy_service d_phy_service;
      int d_interframe_space;
      const int d_log_level;
      bool d_init_done;
      light_plc::vector_complex d_datastream;
      int d_datastream_offset;
      int d_datastream_len;
      int d_samples_since_last_tx;
      bool d_frame_ready;
      enum {READY, PREPARING, TX, HALT} d_transmitter_state;
      std::vector<unsigned char> d_mpdu_fc, d_mpdu_payload;

     public:
      phy_tx_impl(int log_level);
      ~phy_tx_impl();

	  void mac_in (pmt::pmt_t msg);
    void create_ppdu();

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_PHY_TX_IMPL_H */
