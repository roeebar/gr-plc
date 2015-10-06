/* -*- c++ -*- */

#ifndef INCLUDED_PLC_MAC_RECEIVER_IMPL_H
#define INCLUDED_PLC_MAC_RECEIVER_IMPL_H

#include <plc/mac_receiver.h>
#include <fstream>

namespace gr {
  namespace plc {

    class mac_receiver_impl : public mac_receiver
    {
     private:
      long int d_time_begin;
      long int d_total_bytes;      
      bool d_debug;
      std::ofstream d_save_file;
      void parse(pmt::pmt_t msg);

     public:
      mac_receiver_impl(char *filename, bool debug);
      ~mac_receiver_impl();

      /*// Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);*/
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_MAC_RECEIVER_IMPL_H */

