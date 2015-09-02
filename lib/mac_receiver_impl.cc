/* -*- c++ -*- */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "mac_receiver_impl.h"
#include "debug.h"

namespace gr {
  namespace plc {

    mac_receiver::sptr
    mac_receiver::make(char *output_filename, bool debug)
    {
      return gnuradio::get_initial_sptr
        (new mac_receiver_impl(output_filename, debug));
    }

    /*
     * The private constructor
     */
    mac_receiver_impl::mac_receiver_impl(char *output_filename, bool debug)
      : gr::block("mac_receiver",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(0, 0, 0)),
      d_debug(debug)
    {
      message_port_register_in(pmt::mp("in"));
      set_msg_handler(pmt::mp("in"), boost::bind(&mac_receiver_impl::parse, this, _1));
      d_save_file.open(output_filename,std::ios_base::binary);
      if (!d_save_file)
        dout << "MAC Receiver: error creating file " << std::string(output_filename) << std::endl;
    }

    /*
     * Our virtual destructor.
     */
    mac_receiver_impl::~mac_receiver_impl()
    {
      d_save_file.close();
    }

    void mac_receiver_impl::parse(pmt::pmt_t msg) {
      if(pmt::is_eof_object(msg)) {
        dout << "MAC Receiver: EOF" << std::endl;
        return;
      }

      if(pmt::is_pair(msg)) {
        dout << "MAC Receiver: received new message" << std::endl;
        //gr::thread::scoped_lock lock(d_mutex);

        size_t mpdu_payload_length = pmt::blob_length(pmt::cdr(msg));
        const char* mpdu_payload = reinterpret_cast<const char *>(pmt::blob_data(pmt::cdr(msg)));
        d_save_file.write(mpdu_payload, mpdu_payload_length);
      }
    }
  } /* namespace plc */
} /* namespace gr */

