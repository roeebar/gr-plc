/* -*- c++ -*- */

#ifndef INCLUDED_PLC_PHY_RX_IMPL_H
#define INCLUDED_PLC_PHY_RX_IMPL_H

#include <plc/phy_rx.h>
#include <lightplc/phy_service.h>
#include <list>
#include <gnuradio/filter/fir_filter.h>
#include <string>

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
      light_plc::phy_service d_phy_service;
      const bool d_debug;
      bool d_init_done;
      gr::filter::kernel::fir_filter_fff *d_fir;
      enum {SEARCH, SYNC, COPY_PREAMBLE, COPY_FRAME_CONTROL, COPY_PAYLOAD, CONSUME_SPACE, SENSE_SPACE, RESET, IDLE, HALT} d_receiver_state;
      float d_search_corr;
      float d_energy;
      int d_plateau;
      int d_payload_size;
      int d_payload_offset;
      float *d_correlation;
      light_plc::vector_float d_preamble;
      light_plc::vector_float d_frame_control;
      light_plc::vector_float d_payload;
      pmt::pmt_t d_frame_control_pmt;
      int d_sync_offset;
      int d_frame_control_offset;
      int d_preamble_offset;
      int d_frame_start;
      light_plc::vector_int d_output_datastream;
      int d_output_datastream_offset;
      int d_output_datastream_len;
      std::list<std::pair<double, int>> d_cor; 
      std::list<std::pair<double, int>>::iterator d_cor_iter;
      std::vector<float> d_noise;
      int d_noise_offset;
      int d_inter_frame_space_offset;
      std::string d_name;

     public:
      phy_rx_impl(bool debug);
      ~phy_rx_impl();
      void mac_in (pmt::pmt_t msg);
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_PHY_RX_IMPL_H */

