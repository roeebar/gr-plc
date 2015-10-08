/* -*- c++ -*- */

#ifndef INCLUDED_PLC_APP_IN_IMPL_H
#define INCLUDED_PLC_APP_IN_IMPL_H

#include <plc/app_in.h>

namespace gr {
  namespace plc {

    class app_in_impl : public app_in
    {
     private:
      unsigned int d_mac_payload_offset;
      size_t d_mac_payload_length;
      const unsigned char *d_mac_payload;
      long int d_time_begin;
      long int d_total_bytes;
      bool d_debug;

     public:
      app_in_impl(bool debug);
      ~app_in_impl();
	  bool start();
      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
		       gr_vector_int &ninput_items,
		       gr_vector_const_void_star &input_items,
		       gr_vector_void_star &output_items);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_APP_IN_IMPL_H */

