/* -*- c++ -*- */

#ifndef INCLUDED_PLC_MAC_TRANSMITTER_IMPL_H
#define INCLUDED_PLC_MAC_TRANSMITTER_IMPL_H

#include <plc/mac_transmitter.h>
#include <fstream>

namespace gr {
  namespace plc {

    class mac_transmitter_impl : public mac_transmitter
    {
     private:
      boost::shared_ptr<gr::thread::thread> d_thread;
      bool d_finished;
      float d_period_ms;
      bool d_debug;
      std::ifstream d_input_file;
      void send_data();
      void run();


     public:
      mac_transmitter_impl(char *input_filename, float period_ms, bool debug);
      ~mac_transmitter_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      void set_period(float period_ms) { d_period_ms = period_ms; }
      float period() const { return d_period_ms; }

      // Overloading these to start and stop the internal thread that
      // periodically produces the message.
      bool start();
      bool stop();
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_MAC_TRANSMITTER_IMPL_H */

