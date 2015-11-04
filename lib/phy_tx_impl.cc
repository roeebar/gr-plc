/* -*- c++ -*- */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <boost/thread.hpp>
#include "debug.h"
#include "phy_tx_impl.h"

namespace gr {
  namespace plc {

    phy_tx::sptr
    phy_tx::make(int robo_mode, int modulation, bool debug)
    {
      return gnuradio::get_initial_sptr
        (new phy_tx_impl((light_plc::RoboMode)robo_mode, (light_plc::Modulation) modulation, debug));
    }

    /*
     * The private constructor
     */
    phy_tx_impl::phy_tx_impl(light_plc::RoboMode robo_mode, light_plc::Modulation modulation, bool debug)
      : gr::sync_block("phy_tx",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, sizeof(float))),
            d_robo_mode(robo_mode),
            d_modulation(modulation),
            d_debug (debug),
            d_datastream_offset(0),
            d_datastream_len(0),
            d_transmitter_state(RESET)
    {
      message_port_register_in(pmt::mp("mac in"));
      set_msg_handler(pmt::mp("mac in"), boost::bind(&phy_tx_impl::mac_in, this, _1));
      message_port_register_out(pmt::mp("mac out"));
      d_plcp = light_plc::plcp();
      d_plcp.set_modulation(d_modulation);
      d_plcp.set_robo_mode(d_robo_mode);
      //d_plcp.debug(d_debug);
    }

    /*
     * Our virtual destructor.
     */
    phy_tx_impl::~phy_tx_impl()
    {
    }

    void phy_tx_impl::mac_in (pmt::pmt_t msg) {
      if(pmt::is_pair(msg)) {
        if (pmt::is_u8vector(pmt::cdr(msg)) && pmt::is_dict(pmt::car(msg))) {
          pmt::pmt_t dict = pmt::car(msg);
          pmt::pmt_t key = pmt::mp("type");
          if (!pmt::dict_has_key(dict,key))
            return;
          if (d_transmitter_state == READY) {
            pmt::pmt_t type = pmt::dict_ref(dict, key, pmt::PMT_NIL);
            if (pmt::symbol_to_string(type) == "sof") {
              dout << "PHY Transmitter: received new MPDU (SOF) from MAC" << std::endl;            
              size_t mpdu_payload_length = 0;
              const unsigned char * mpdu_payload = pmt::u8vector_elements(pmt::cdr(msg), mpdu_payload_length);
              d_datastream = d_plcp.create_sof_ppdu(mpdu_payload, mpdu_payload_length);
              d_datastream_len = d_datastream.size();
              d_transmitter_state = TX;
            } else if (pmt::symbol_to_string(type) == "sound") {
              dout << "PHY Transmitter: received new MPDU (Sound) from MAC" << std::endl;            
              d_datastream = d_plcp.create_sound_ppdu(light_plc::STD_ROBO);
              d_datastream_len = d_datastream.size();
              d_transmitter_state = TX;
            }
          } else if (d_transmitter_state != READY) {
            std::cerr << "PHY Transmitter: received MPDU while transmitter is busy, dropping MPDU" << std::endl;             
          }
        }
      }
    }

    int
    phy_tx_impl::work(int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items)
    {
      int i = 0;
      bool done = false;

      while (!done) {
        switch (d_transmitter_state) {
          case TX: {
            float *out = (float *) output_items[0];

            i = std::min(noutput_items, d_datastream_len - d_datastream_offset);
            if (d_datastream_offset == 0 && i > 0) {
              // add tags
              pmt::pmt_t key = pmt::string_to_symbol("packet_len");
              pmt::pmt_t value = pmt::from_long(d_datastream_len);
              pmt::pmt_t srcid = pmt::string_to_symbol(alias());
              add_item_tag(0, nitems_written(0), key, value, srcid);
            }

            std::memcpy(out, &d_datastream[d_datastream_offset], sizeof(light_plc::vector_float::value_type)*i);
            dout << "PHY Transmitter: state = BUSY, copied " << i << "/" << d_datastream_len << std::endl;

            d_datastream_offset += i;

            if(i > 0 && d_datastream_offset == d_datastream_len) {
              dout << "PHY Transmitter: state = BUSY, MPDU sent!" << std::endl;
              d_transmitter_state = RESET;       
            } else {
              done = true;
            }
            break;
          }

          case RESET: {
            dout << "PHY Transmitter: state = RESET" << std::endl;
            d_datastream_offset = 0;
            d_datastream_len = 0;
            d_transmitter_state = READY;
            message_port_pub(pmt::mp("mac out"), pmt::mp("READY"));
            break;
          }

          case READY:
            done = true;
            break;
        }
      }
      // Tell runtime system how many output items we produced.
      return i;
    }
  } /* namespace plc */
} /* namespace gr */

