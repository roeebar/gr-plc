/* -*- c++ -*- */

#ifndef INCLUDED_PLC_APP_OUT_IMPL_H
#define INCLUDED_PLC_APP_OUT_IMPL_H

#include <plc/app_out.h>

namespace gr {
  namespace plc {

    class app_out_impl : public app_out
    {

     static const int PAYLOAD_SIZE = 1516;

     private:
      unsigned char d_mac_payload[PAYLOAD_SIZE];
      pmt::pmt_t d_mac_payload_pmt;
      pmt::pmt_t d_dest_pmt;
      int d_mac_payload_offset;
      bool d_payload_sent;
      long int d_time_begin;
      long int d_total_bytes;
      bool d_debug;

     public:
      app_out_impl(std::vector<uint8_t> dest, bool debug);
      ~app_out_impl();

      void send_payload();
      bool start();
      bool stop();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
		       gr_vector_int &ninput_items,
		       gr_vector_const_void_star &input_items,
		       gr_vector_void_star &output_items);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_APP_OUT_IMPL_H */

