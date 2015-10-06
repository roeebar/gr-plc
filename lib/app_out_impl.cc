/* -*- c++ -*- */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "app_out_impl.h"
#include "debug.h"
#include <cassert>

namespace gr {
  namespace plc {

    app_out::sptr
    app_out::make(std::vector<uint8_t> dest, bool debug)
    {
      return gnuradio::get_initial_sptr
        (new app_out_impl(dest, debug));
    }

    /*
     * The private constructor
     */
    app_out_impl::app_out_impl(std::vector<uint8_t> dest, bool debug)
      : gr::block("app_out",
              gr::io_signature::make(1, 1, sizeof(unsigned char)),
              gr::io_signature::make(0, 0, 0)),
        d_mac_payload_offset(0),
        d_debug(debug)
    {
        message_port_register_out(pmt::mp("mac out"));
        d_mac_payload_pmt = pmt::make_u8vector(PAYLOAD_SIZE, 0);

        assert (dest.size() == 6);
        d_dest_pmt = pmt::mp(std::string((char *)dest.data(),dest.size()));
    }

    /*
     * Our virtual destructor.
     */
    app_out_impl::~app_out_impl()
    {
    }

    void
    app_out_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
        /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    int
    app_out_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
        const unsigned char *in = (const unsigned char *) input_items[0];

        int i = ninput_items[0];
        int c = 0; // consumed;
        while (i >= PAYLOAD_SIZE) {
            dout << "APP Out: sent payload, size = " << PAYLOAD_SIZE << std::endl;

            // dict
            pmt::pmt_t dict = pmt::make_dict();
            dict = pmt::dict_add(dict, pmt::mp("dest"), d_dest_pmt);
            d_mac_payload_pmt = pmt::init_u8vector(PAYLOAD_SIZE, in + c);

            // mpdu
            message_port_pub(pmt::mp("mac out"), pmt::cons(dict, d_mac_payload_pmt)); 

            i -= PAYLOAD_SIZE;
            c += PAYLOAD_SIZE;
        }

        consume (0, c);
        return 0;
    }

  } /* namespace plc */
} /* namespace gr */

