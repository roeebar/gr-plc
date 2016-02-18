/* -*- c++ -*- */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <boost/thread.hpp>
#include "debug.h"
#include "phy_tx_impl.h"
#include <thread>

namespace gr {
  namespace plc {

    const int phy_tx_impl::MIN_INTERFRAME_SPACE = light_plc::phy_service::MIN_INTERFRAME_SPACE;

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
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
            d_debug (debug),
            d_init_done(false),
            d_datastream_offset(0),
            d_datastream_len(0),
            d_samples_since_last_tx(0),
            d_frame_ready(false),
            d_transmitter_state(HALT),
            d_name("PHY Tx")
    {
      message_port_register_in(pmt::mp("mac in"));
      set_msg_handler(pmt::mp("mac in"), boost::bind(&phy_tx_impl::mac_in, this, _1));
      message_port_register_out(pmt::mp("mac out"));
    }

    /*
     * Our virtual destructor.
     */
    phy_tx_impl::~phy_tx_impl()
    {
    }

    void phy_tx_impl::mac_in (pmt::pmt_t msg) {
      if (!(pmt::is_pair(msg) && pmt::is_symbol(pmt::car(msg)) && pmt::is_dict(pmt::cdr(msg))))
          return;

      std::string cmd = pmt::symbol_to_string(pmt::car(msg));
      pmt::pmt_t dict = pmt::cdr(msg);

      if (cmd == "PHY-TXCONFIG") {
        if (d_transmitter_state == PREPARING) {
          std::cerr << d_name << ": ERROR: cannot config while preparing for tx" << std::endl;
          return;
        }
        // Set tone map
        if (pmt::dict_has_key(dict,pmt::mp("tone_map"))) {
          dout << d_name << ": setting custom tx tone map" << std::endl;
          pmt::pmt_t tone_map_pmt = pmt::dict_ref(dict, pmt::mp("tone_map"), pmt::PMT_NIL);
          size_t tone_map_len = 0;
          const uint8_t *tone_map_blob = pmt::u8vector_elements(tone_map_pmt, tone_map_len);
          light_plc::tone_map_t tone_map;
          for (size_t j = 0; j<tone_map_len; j++)
            tone_map[j] = (light_plc::modulation_type_t)tone_map_blob[j];
          d_phy_service.set_tone_map(tone_map);
        }
      }

      else if (cmd == "PHY-TXINIT") {
        if (!d_init_done) {
          light_plc::tone_mask_t tone_mask;
          light_plc::sync_tone_mask_t sync_tone_mask;
          if (pmt::dict_has_key(dict,pmt::mp("broadcast_tone_mask")) &&
              pmt::dict_has_key(dict,pmt::mp("sync_tone_mask")))
          {
            dout << d_name << ": setting tone masks" << std::endl;

            // Set broadcast tone mask
            pmt::pmt_t tone_mask_pmt = pmt::dict_ref(dict, pmt::mp("broadcast_tone_mask"), pmt::PMT_NIL);
            size_t tone_mask_len = 0;
            const uint8_t *tone_mask_blob = pmt::u8vector_elements(tone_mask_pmt, tone_mask_len);
            assert(tone_mask_len == tone_mask.size());
            for (size_t j = 0; j<tone_mask_len; j++)
              tone_mask[j] = tone_mask_blob[j];

            // Set sync tone mask
            pmt::pmt_t sync_tone_mask_pmt = pmt::dict_ref(dict, pmt::mp("sync_tone_mask"), pmt::PMT_NIL);
            size_t sync_tone_mask_len = 0;
            const uint8_t *sync_tone_mask_blob = pmt::u8vector_elements(sync_tone_mask_pmt, sync_tone_mask_len);
            assert(sync_tone_mask_len == sync_tone_mask.size());
            for (size_t j = 0; j<sync_tone_mask_len; j++)
              sync_tone_mask[j] = sync_tone_mask_blob[j];

            d_phy_service = light_plc::phy_service(tone_mask, tone_mask, sync_tone_mask);
            //d_phy_service.debug(d_debug);
          }

          if (pmt::dict_has_key(dict,pmt::mp("id")))
            d_name = "PHY Tx (" + pmt::symbol_to_string(pmt::dict_ref(dict, pmt::mp("id"), pmt::PMT_NIL)) + ")";

          d_transmitter_state = READY;
          d_init_done = true;
          dout << d_name << ": init done" << std::endl;
        } else
          std::cerr << d_name << ": ERROR: cannot init more than once" << std::endl;
      }

      else if (cmd == "PHY-TXSTART") {
        if (d_transmitter_state == READY) {
          // Get frame control
          pmt::pmt_t mpdu_fc_pmt = pmt::dict_ref(dict, pmt::mp("frame_control"), pmt::PMT_NIL);
          size_t mpdu_fc_length = 0;
          const unsigned char *mpdu_fc = pmt::u8vector_elements(mpdu_fc_pmt, mpdu_fc_length);
          d_mpdu_fc = std::vector<unsigned char>(mpdu_fc, mpdu_fc + mpdu_fc_length);
          // Get payload
          size_t mpdu_payload_length = 0;
          const unsigned char *mpdu_payload = NULL;
          d_mpdu_payload = std::vector<unsigned char>(0);
          if (pmt::dict_has_key(dict,pmt::mp("payload"))) {
            pmt::pmt_t mpdu_payload_pmt = pmt::dict_ref(dict, pmt::mp("payload"), pmt::PMT_NIL);
            mpdu_payload = pmt::u8vector_elements(mpdu_payload_pmt, mpdu_payload_length);
            d_mpdu_payload = std::vector<unsigned char>(mpdu_payload, mpdu_payload + mpdu_payload_length);
          }
          dout << d_name << ": received new MPDU from MAC" << std::endl;
          d_transmitter_state = PREPARING;
          std::thread{&phy_tx_impl::create_ppdu, this}.detach(); // creating the PPDU in a new thread not to starve the work routine
        } else {
          std::cerr << d_name << ": received MPDU while transmitter is not ready, dropping MPDU" << std::endl;
        }
      }
    }

    void phy_tx_impl::create_ppdu() {
      d_datastream = d_phy_service.create_ppdu(d_mpdu_fc.data(), d_mpdu_fc.size(), d_mpdu_payload.data(), d_mpdu_payload.size());
      d_datastream_len = d_datastream.size();
      d_frame_ready = true;
      return;
    }

    int
    phy_tx_impl::work(int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items)
    {
      int i = 0;
      gr_complex *out = (gr_complex *) output_items[0];

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

            std::memcpy(out, &d_datastream[d_datastream_offset], sizeof(light_plc::vector_complex::value_type)*i);
            dout << d_name << ": state = TX, copied " << i << "/" << d_datastream_len << std::endl;

            d_datastream_offset += i;

            if(i > 0 && d_datastream_offset == d_datastream_len) {
              dout << d_name << ": state = TX, MPDU sent!" << std::endl;
              d_datastream_offset = 0;
              d_datastream_len = 0;
              d_samples_since_last_tx = 0;
              d_transmitter_state = READY;
              d_frame_ready = false;
              pmt::pmt_t dict = pmt::make_dict();
              message_port_pub(pmt::mp("mac out"), pmt::cons(pmt::mp("PHY-TXEND"), dict));
            }
            break;
          }

          case PREPARING:
            if (d_frame_ready) {
              if (d_samples_since_last_tx >= MIN_INTERFRAME_SPACE)
                d_transmitter_state = TX;
              else {
                i = std::min(MIN_INTERFRAME_SPACE - d_samples_since_last_tx, (unsigned int)noutput_items);
                d_samples_since_last_tx += i;
                std::memset(out, 0, sizeof(gr_complex)*i);
              }
              break;
            }

          case READY: {
            i = noutput_items;
            std::memset(out, 0, sizeof(gr_complex)*i);
            pmt::pmt_t key = pmt::string_to_symbol("packet_len");
            pmt::pmt_t value = pmt::from_long(i);
            pmt::pmt_t srcid = pmt::string_to_symbol(alias());
            add_item_tag(0, nitems_written(0), key, value, srcid);
            d_samples_since_last_tx += i;
            break;
          }

          case HALT:
            break;
        }
      // Tell runtime system how many output items we produced.
      return i;
    }
  } /* namespace plc */
} /* namespace gr */
