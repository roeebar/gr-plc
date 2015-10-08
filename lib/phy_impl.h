/* -*- c++ -*- */

#ifndef INCLUDED_PLC_PHY_IMPL_H
#define INCLUDED_PLC_PHY_IMPL_H

#include <plc/phy.h>
#include <lightplc/plcp.h>
#include <list>
#include <gnuradio/filter/fir_filter.h>

namespace gr {
  namespace plc {

    class phy_impl : public phy
    {
   	 static const int SYNCP_SIZE;
     static const int SYNC_LENGTH;
     static const int PREAMBLE_SIZE;
     static const int FRAME_CONTROL_SIZE;
     static const float THRESHOLD;
     static const float MIN_ENERGY;
     static const int MIN_PLATEAU; 

     private:
      light_plc::Plcp d_plcp;
      light_plc::RoboMode d_robo_mode;
      light_plc::Modulation d_modulation;
      const bool d_debug;

      // Transmitter vars
      bool d_transmitter_disabled;
      light_plc::VectorFloat d_datastream;
      int d_datastream_offset;
      int d_datastream_len;
      enum {READY, BUSY, TX_RESET} d_transmitter_state;

      // Receiver vars
      bool d_receiver_disabled;
      gr::filter::kernel::fir_filter_fff *d_fir;
      enum {SEARCH, SYNC, CHANNEL_ESTIMATE, COPY_FRAME_CONTROL, COPY_PAYLOAD, RX_RESET} d_receiver_state;
      int d_plateau;
      int d_payload_size;
      int d_payload_offset;
      float *d_correlation;
      light_plc::VectorFloat d_preamble;
      light_plc::VectorFloat d_frame_control;
      light_plc::VectorFloat d_payload;
      int d_sync_offset;
      int d_frame_control_offset;
      int d_preamble_offset;
      int d_frame_start;
      light_plc::VectorInt d_output_datastream;
      int d_output_datastream_offset;
      int d_output_datastream_len;
      std::list<std::pair<double, int>> d_cor; 
      std::list<std::pair<double, int>>::iterator d_cor_iter;

     public:
      phy_impl(light_plc::RoboMode robo_mode, light_plc::Modulation modulation, bool disable_receiver, bool disable_transmitter, bool debug);
      ~phy_impl();
	  void mac_in (pmt::pmt_t msg);

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
		       gr_vector_int &ninput_items,
		       gr_vector_const_void_star &input_items,
		       gr_vector_void_star &output_items);
    };

  } // namespace plc
} // namespace gr


#endif /* INCLUDED_PLC_PHY_IMPL_H */

