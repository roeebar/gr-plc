/* -*- c++ -*- */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <sys/time.h> 
#include "app_out_impl.h"
#include "debug.h"
#include <cassert>

namespace gr {
  namespace plc {

    app_out::sptr
    app_out::make(bool debug)
    {
      return gnuradio::get_initial_sptr
        (new app_out_impl(debug));
    }

    /*
     * The private constructor
     */
    app_out_impl::app_out_impl(bool debug)
      : gr::block("app_out",
              gr::io_signature::make(1, 1, sizeof(unsigned char)),
              gr::io_signature::make(0, 0, 0)),
        d_mac_payload_offset(0),
        d_total_bytes(0),
        d_debug(debug)
    {
        message_port_register_out(pmt::mp("mac out"));
        message_port_register_in(pmt::mp("mac in"));

        d_mac_payload_pmt = pmt::make_u8vector(PAYLOAD_SIZE, 0);
    }

    /*
     * Our virtual destructor.
     */
    app_out_impl::~app_out_impl()
    {
    }

    bool app_out_impl::start()
    {
        struct timeval tp;
        gettimeofday(&tp, NULL);
        d_time_begin = tp.tv_sec * 1000 + tp.tv_usec / 1000;
        return true;
    }

    bool app_out_impl::stop () {
      if (d_mac_payload_offset) 
        send_payload();
      return true;
    }

    void
    app_out_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
        /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    void app_out_impl::send_payload() {
      // dict
      pmt::pmt_t dict = pmt::make_dict();
      // payload
      d_mac_payload_pmt = pmt::init_u8vector(d_mac_payload_offset, d_mac_payload);

      bool payload_sent = false;
      while (!payload_sent) {
        pmt::pmt_t msg(delete_head_blocking(pmt::intern("mac in")));
        if (pmt::symbol_to_string(msg) == std::string("READY")) {
          message_port_pub(pmt::mp("mac out"), pmt::cons(dict, d_mac_payload_pmt));
          payload_sent = true;
          dout << "APP Out: sent payload, size = " << d_mac_payload_offset << std::endl;
          d_total_bytes += d_mac_payload_offset;
          d_mac_payload_offset = 0;
        }
      }

      struct timeval tp;
      gettimeofday(&tp, NULL);
      long int now = tp.tv_sec * 1000 + tp.tv_usec / 1000;
      dout << "APP Out: Rate:" << (double)(d_total_bytes)/(now-d_time_begin) << " kB/s" << std::endl;
    }

    int
    app_out_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
        const unsigned char *in = (const unsigned char *) input_items[0];

        int c = 0; // consumed
        while (ninput_items[0] - c) {
          int i = std::min(ninput_items[0] - c, PAYLOAD_SIZE - d_mac_payload_offset);
          memcpy(&d_mac_payload[d_mac_payload_offset], in + c, sizeof(unsigned char) * i);
          d_mac_payload_offset += i;
          c += i;
          if (d_mac_payload_offset == PAYLOAD_SIZE) {
            send_payload();
          }
        }
        consume(0, c);
        return 0;
    }

  } /* namespace plc */
} /* namespace gr */

