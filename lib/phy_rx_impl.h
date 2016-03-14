/* -*- c++ -*- */

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
      static const int MIN_INTERFRAME_SPACE;
      static const size_t BUFFER_SIZE;
      static const int MAX_SEARCH_LENGTH;

      light_plc::phy_service d_phy_service;
      const bool d_debug;
      const bool d_info;
      bool d_init_done;
      enum {SEARCH, SYNC, COPY_PREAMBLE, COPY_FRAME_CONTROL, COPY_PAYLOAD, RESET, IDLE, HALT} d_receiver_state;
      float d_search_corr;
      float d_energy_a, d_energy_b;
      gr_complex *d_mult, *d_buffer, *d_frame_control, *d_payload;
      float *d_real, *d_energy;
      int d_plateau;
      int d_payload_size;
      int d_payload_offset;
      pmt::pmt_t d_frame_control_pmt;
      float d_sync_min;
      int d_sync_min_index;
      size_t d_buffer_offset;
      float *d_preamble_corr;
      int d_frame_start;
      std::string d_name;

     public:
      phy_rx_impl(bool info, bool debug);
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
