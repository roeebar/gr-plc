/* -*- c++ -*- */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <sys/time.h>
#include "app_in_impl.h"
#include "debug.h"

namespace gr {
  namespace plc {

    app_in::sptr
    app_in::make(int log_level)
    {
      return gnuradio::get_initial_sptr
        (new app_in_impl(log_level));
    }

    /*
     * The private constructor
     */
    app_in_impl::app_in_impl(int log_level)
      : gr::block("app_in",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, sizeof(unsigned char))),
        d_mac_payload_offset(0),
        d_mac_payload_length(0),
        d_total_bytes(0),
        d_log_level(log_level)
    {
        message_port_register_in(pmt::mp("mac in"));
    }

    /*
     * Our virtual destructor.
     */
    app_in_impl::~app_in_impl()
    {
    }

    bool app_in_impl::start()
    {
        struct timeval tp;
        gettimeofday(&tp, NULL);
        d_time_begin = tp.tv_sec * 1000 + tp.tv_usec / 1000;
        return true;
    }

    void
    app_in_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
        /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    int
    app_in_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
        unsigned char *out = (unsigned char *) output_items[0];
        while (!d_mac_payload_offset) {
            pmt::pmt_t msg(delete_head_blocking(pmt::intern("mac in")));
            if (pmt::is_pair(msg) && pmt::is_symbol(pmt::car(msg)) && pmt::is_dict(pmt::cdr(msg))) {
                std::string cmd = pmt::symbol_to_string(pmt::car(msg));
                pmt::pmt_t dict = pmt::cdr(msg);
                if (cmd  == std::string("MAC-RXMSDU")) {
                    pmt::pmt_t mac_payload_pmt = pmt::dict_ref(dict, pmt::mp("msdu"), pmt::PMT_NIL);
                    d_mac_payload = (const unsigned char*) pmt::u8vector_elements(mac_payload_pmt, d_mac_payload_length);
                    dout << "APP In: received new payload, size = " << d_mac_payload_length << std::endl;
                    break;
                }
            }
        }

        int i = std::min(noutput_items, (int) (d_mac_payload_length - d_mac_payload_offset));
        std::memcpy(out, d_mac_payload + d_mac_payload_offset, sizeof(unsigned char) * i);
        d_mac_payload_offset += i;
        d_total_bytes += i;
        if (d_mac_payload_offset == d_mac_payload_length) {
            d_mac_payload_offset = 0;
        }

        struct timeval tp;
        gettimeofday(&tp, NULL);
        long int now = tp.tv_sec * 1000 + tp.tv_usec / 1000;
        dout << "APP In: Rate:" << (double)(d_total_bytes)/(now-d_time_begin) << " kB/s" << std::endl;

        // Tell runtime system how many output items we produced.
        return i;
    }
  } /* namespace plc */
} /* namespace gr */
