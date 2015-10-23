/* -*- c++ -*- */

#ifndef INCLUDED_PLC_PHY_RX_IMPL_H
#define INCLUDED_PLC_PHY_RX_IMPL_H

#include <plc/phy_rx.h>
#include <lightplc/plcp.h>
#include <list>
#include <gnuradio/filter/fir_filter.h>

namespace gr {
  namespace plc {

    class phy_rx_impl : public phy_rx
    {
   	 static const int SYNCP_SIZE;
     static const int SYNC_LENGTH;
     static const int PREAMBLE_SIZE;
     static const int FRAME_CONTROL_SIZE;
     static const float THRESHOLD;
     static const float MIN_ENERGY;
     static const int MIN_PLATEAU; 

     private:
      light_plc::plcp d_plcp;
      light_plc::RoboMode d_robo_mode;
      light_plc::Modulation d_modulation;
      const bool d_debug;
      gr::filter::kernel::fir_filter_fff *d_fir;
      enum {SEARCH, SYNC, CHANNEL_ESTIMATE, COPY_FRAME_CONTROL, COPY_PAYLOAD, RX_RESET} d_receiver_state;
      int d_plateau;
      int d_payload_size;
      int d_payload_offset;
      float *d_correlation;
      light_plc::vector_float d_preamble;
      light_plc::vector_float d_frame_control;
      light_plc::vector_float d_payload;
      int d_sync_offset;
      int d_frame_control_offset;
      int d_preamble_offset;
      int d_frame_start;
      light_plc::vector_int d_output_datastream;
      int d_output_datastream_offset;
      int d_output_datastream_len;
      std::list<std::pair<double, int>> d_cor; 
      std::list<std::pair<double, int>>::iterator d_cor_iter;

     public:
      phy_rx_impl(light_plc::RoboMode robo_mode, light_plc::Modulation modulation, bool debug);
      ~phy_rx_impl();
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_PHY_RX_IMPL_H */

