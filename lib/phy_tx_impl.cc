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
    phy_tx::make(bool debug)
    {
      return gnuradio::get_initial_sptr
        (new phy_tx_impl(debug));
    }

    /*
     * The private constructor
     */
    phy_tx_impl::phy_tx_impl(bool debug)
      : gr::sync_block("phy_tx",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, sizeof(float))),
            d_debug (debug),
            d_datastream_offset(0),
            d_datastream_len(0),
            d_transmitter_state(READY)
    {
      message_port_register_in(pmt::mp("mac in"));
      set_msg_handler(pmt::mp("mac in"), boost::bind(&phy_tx_impl::mac_in, this, _1));
      message_port_register_out(pmt::mp("mac out"));
      d_phy_service = light_plc::phy_service();
      //d_phy_service.debug(d_debug);
    }

    /*
     * Our virtual destructor.
     */
    phy_tx_impl::~phy_tx_impl()
    {
    }

    void phy_tx_impl::mac_in (pmt::pmt_t msg) {     
      if(pmt::is_pair(msg)) {
        if (pmt::is_symbol(pmt::car(msg)) && pmt::is_dict(pmt::cdr(msg))) {
          std::string cmd = pmt::symbol_to_string(pmt::car(msg));
          pmt::pmt_t dict = pmt::cdr(msg);
          if (cmd == "SACK") {
            if (!pmt::dict_has_key(dict, pmt::mp("sackd")))
              return;
            dout << "PHY Transmitter: received new MPDU (SACK) from MAC" << std::endl;            
            size_t sackd_length = 0;
            pmt::pmt_t sackd_u8vector = pmt::dict_ref(dict, pmt::mp("sackd"), pmt::PMT_NIL);
            const unsigned char * sackd = pmt::u8vector_elements(sackd_u8vector, sackd_length);
            d_datastream = d_phy_service.create_sack_ppdu(sackd, sackd_length);
            d_datastream_len = d_datastream.size();
            d_transmitter_state = TX;
          } else if (cmd == "SOUND") {
            dout << "PHY Transmitter: received new MPDU (Sound) from MAC" << std::endl;            
            d_datastream = d_phy_service.create_sound_ppdu(light_plc::STD_ROBO);
            d_datastream_len = d_datastream.size();
            d_transmitter_state = TX;
          }
        }   
        if (pmt::is_u8vector(pmt::cdr(msg)) && pmt::is_dict(pmt::car(msg))) {
          pmt::pmt_t dict = pmt::car(msg);
          pmt::pmt_t key = pmt::mp("type");
          if (!pmt::dict_has_key(dict,key))
            return;
          if (d_transmitter_state == READY) {
            pmt::pmt_t type = pmt::dict_ref(dict, key, pmt::PMT_NIL);
            if (pmt::symbol_to_string(type) == "sof") {
              uint64_t modulation = pmt::to_uint64(pmt::dict_ref(dict, pmt::mp("modulation"), pmt::PMT_NIL));
              uint64_t robo_mode = pmt::to_uint64(pmt::dict_ref(dict, pmt::mp("robo_mode"), pmt::PMT_NIL));
              dout << "PHY Transmitter: received new MPDU (SOF, robo_mode=" << robo_mode << ", modulation=" << modulation << ") from MAC" << std::endl;            
              d_phy_service.set_modulation((light_plc::modulation_type)modulation);
              d_phy_service.set_robo_mode((light_plc::RoboMode)robo_mode);
              size_t mpdu_payload_length = 0;
              const unsigned char * mpdu_payload = pmt::u8vector_elements(pmt::cdr(msg), mpdu_payload_length);
              d_datastream = d_phy_service.create_sof_ppdu(mpdu_payload, mpdu_payload_length);
              d_datastream_len = d_datastream.size();
              d_transmitter_state = TX;
            } 
          } else {
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
      float *out = (float *) output_items[0];

      //while (!done) {
        switch (d_transmitter_state) {
          case TX: {

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
            pmt::pmt_t dict = pmt::make_dict();
            message_port_pub(pmt::mp("mac out"), pmt::cons(pmt::mp("PHY-TXEND"), dict));            
            break;
          }

          case READY: {
            i = noutput_items;
            std::memset(out, 0, sizeof(float)*i);            
            pmt::pmt_t key = pmt::string_to_symbol("packet_len");
            pmt::pmt_t value = pmt::from_long(i);
            pmt::pmt_t srcid = pmt::string_to_symbol(alias());
            add_item_tag(0, nitems_written(0), key, value, srcid);            
            done = true;
            break;
          }
        }
      //}
      // Tell runtime system how many output items we produced.
      return i;
    }
  } /* namespace plc */
} /* namespace gr */

