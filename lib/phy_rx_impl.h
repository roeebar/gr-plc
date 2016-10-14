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

#ifndef INCLUDED_PLC_PHY_RX_IMPL_H
#define INCLUDED_PLC_PHY_RX_IMPL_H

#include <plc/phy_rx.h>
#include <lightplc/phy_service.h>
#include <list>
#include <string>

namespace gr {
  namespace plc {

    class phy_rx_impl : public phy_rx
    {
     private:
      static const int SYNCP_SIZE;
      static const int COARSE_SYNC_LENGTH;
      static const int FINE_SYNC_LENGTH;
      static const int PREAMBLE_SIZE;
      static const int FRAME_CONTROL_SIZE;
      static const float THRESHOLD;
      static const int MIN_PLATEAU;
      static const int SILENCE_PERIOD;
      static const size_t BUFFER_SIZE;
      static const int MAX_SEARCH_LENGTH;

      light_plc::phy_service d_phy_service;
      const float d_threshold;
      int d_interframe_space;
      const int d_log_level;
      int d_buffer_size;
      light_plc::tone_mask_t d_qpsk_tone_mask;
      bool d_init_done;
      enum {SEARCH, SYNC, COPY_PREAMBLE, COPY_FRAME_CONTROL, COPY_PAYLOAD, RESET, IDLE, HALT} d_receiver_state;
      float d_search_corr;
      float d_energy_a, d_energy_b;
      gr_complex *d_mult, *d_buffer, *d_frame_control, *d_payload;
      float *d_real, *d_energy, *d_corr_history, *d_energy_history;
      int d_plateau;
      int d_payload_size;
      int d_payload_offset;
      pmt::pmt_t d_frame_control_pmt;
      float d_sync_min;
      int d_sync_min_index;
      size_t d_buffer_offset;
      int d_frame_start;
      int d_corr_idx, d_energy_idx;

     public:
      phy_rx_impl(float threshold, int log_level);
      ~phy_rx_impl();
      void mac_in (pmt::pmt_t msg);
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);
      // Copy data into a circular buffer
      void copy_to_circular_buffer(void *buffer, size_t buffer_size, size_t &buffer_offset, const void* src, size_t size, size_t datatype_size);
      // Copy data from a circular buffer
      void copy_from_circular_buffer(void *dest, void *buffer, size_t buffer_size, size_t buffer_offset, size_t size, size_t datatype_size);
      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_PHY_RX_IMPL_H */
