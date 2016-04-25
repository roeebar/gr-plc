/* -*- c++ -*- */

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
      static const int MIN_INTERFRAME_SPACE;

      light_plc::phy_service d_phy_service;
      const int d_log_level;
      bool d_init_done;
      light_plc::vector_complex d_datastream;
      int d_datastream_offset;
      int d_datastream_len;
      unsigned int d_samples_since_last_tx;
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
