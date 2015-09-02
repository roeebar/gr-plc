/* -*- c++ -*- */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "mac_transmitter_impl.h"
#include "debug.h"

namespace gr {
  namespace plc {

    mac_transmitter::sptr
    mac_transmitter::make(char *input_filename, float period_ms, bool debug)
    {
      return gnuradio::get_initial_sptr
        (new mac_transmitter_impl(input_filename, period_ms, debug));
    }

    /*
     * The private constructor
     */
    mac_transmitter_impl::mac_transmitter_impl(char *input_filename, float period_ms, bool debug)
      : gr::block("mac_transmitter",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(0, 0, 0)),
              d_finished(false),
              d_period_ms(period_ms),
              d_debug(debug)
    {
      message_port_register_out(pmt::mp("out"));
      d_input_file.open(input_filename, std::ios_base::binary);
      if (!d_input_file) {
        dout << "MAC Receiver: error reading file " << std::string(input_filename) << std::endl;
        d_finished = true;
      }
    }

    /*
     * Our virtual destructor.
     */
    mac_transmitter_impl::~mac_transmitter_impl()
    {
      d_input_file.close();
    }

    void
    mac_transmitter_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
        /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    bool mac_transmitter_impl::start()
    {
      // NOTE: d_finished should be something explicitely thread safe. But since
      // nothing breaks on concurrent access, I'll just leave it as bool.
      d_finished = false;
      d_thread = boost::shared_ptr<gr::thread::thread>
        (new gr::thread::thread(boost::bind(&mac_transmitter_impl::run, this)));

      return block::start();
    }

    bool mac_transmitter_impl::stop()
    {
      // Shut down the thread
      d_finished = true;
      d_thread->interrupt();
      d_thread->join();

      return block::stop();
    }

    void mac_transmitter_impl::run()
    {
      while(!d_finished) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(d_period_ms));
        if(d_finished) {
          dout << "MAC Transmitter: finished" << std::endl;
          return;
        }
        send_data();
      }
      message_port_pub(pmt::mp("out"), pmt::PMT_EOF);
    }

//    int binaryRandom() { return ((rand() % 2) == 0); }

    void mac_transmitter_impl::send_data (void)
    {
      // make MAC frame      
      int number_of_blocks = 2;
      int mpdu_payload_length = 520*8*number_of_blocks;
      std::vector<char> mpdu_payload(mpdu_payload_length);
      d_input_file.read(mpdu_payload.data(), mpdu_payload_length);

      dout << "MAC Transmitter: read " << d_input_file.gcount() << " from file" << std::endl;

      if (d_input_file.eof()) 
        dout << "MAC Transmitter: reached EOF" << std::endl;

      if (!d_input_file.good()) {
        d_finished = true;
        dout << "MAC Transmitter: error reading file" << std::endl;
      } else {
        // dict
        pmt::pmt_t dict = pmt::make_dict();
        //dict = pmt::dict_add(dict, pmt::mp("crc_included"), pmt::PMT_T);

        // blob
        pmt::pmt_t mpdu_payload_blob = pmt::make_blob(mpdu_payload.data(), mpdu_payload.size());

        // pdu
        message_port_pub(pmt::mp("out"), pmt::cons(dict, mpdu_payload_blob));
      }
    }
  } /* namespace plc */
} /* namespace gr */

