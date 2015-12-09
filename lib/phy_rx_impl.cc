/* -*- c++ -*- */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "phy_rx_impl.h"
#include "debug.h"
#include <gnuradio/fft/fft.h>

namespace gr {
  namespace plc {

    const int phy_rx_impl::SYNCP_SIZE = light_plc::phy_service::SYNCP_SIZE;
    const int phy_rx_impl::SYNC_LENGTH = phy_rx_impl::SYNCP_SIZE + light_plc::phy_service::ROLLOFF_INTERVAL; // length for frame alignment attempt
    const int phy_rx_impl::PREAMBLE_SIZE = light_plc::phy_service::PREAMBLE_SIZE;
    const int phy_rx_impl::FRAME_CONTROL_SIZE = light_plc::phy_service::FRAME_CONTROL_SIZE;
    const float phy_rx_impl::THRESHOLD = 0.9; // autocorrelation threshold
    const float phy_rx_impl::MIN_ENERGY = 1e-3; // signal minimum energy
    const int phy_rx_impl::MIN_PLATEAU = 5.5 * phy_rx_impl::SYNCP_SIZE - light_plc::phy_service::ROLLOFF_INTERVAL; // minimum autocorrelation plateau

    phy_rx::sptr
    phy_rx::make(bool debug)
    {
      return gnuradio::get_initial_sptr
        (new phy_rx_impl(debug));
    }

    /*
     * The private constructor
     */
    phy_rx_impl::phy_rx_impl(bool debug)
      : gr::sync_block("phy_rx",
              gr::io_signature::make(1, 1, sizeof(float)),
              gr::io_signature::make(0, 0, 0)),
            d_debug (debug),
            d_init_done(false),
            d_receiver_state(HALT),
            d_search_corr(0),
            d_energy(0),
            d_plateau(0),
            d_payload_size(0),
            d_payload_offset(0),
            d_sync_offset(0),
            d_frame_control_offset(0),
            d_preamble_offset(0),
            d_frame_start(0),
            d_output_datastream_offset(0),
            d_output_datastream_len(0),
            d_noise(0),
            d_noise_offset(0),
            d_inter_frame_space_offset(0),
            d_name("PHY Rx")
    {

      message_port_register_out(pmt::mp("mac out"));
      message_port_register_in(pmt::mp("mac in"));
      set_msg_handler(pmt::mp("mac in"), boost::bind(&phy_rx_impl::mac_in, this, _1));    
    }

    /*
     * Our virtual destructor.
     */
    phy_rx_impl::~phy_rx_impl()
    {
      delete d_fir;
      gr::fft::free(d_correlation);        
    }

    void phy_rx_impl::mac_in (pmt::pmt_t msg) {
      if (!(pmt::is_pair(msg) && pmt::is_symbol(pmt::car(msg)) && pmt::is_dict(pmt::cdr(msg))))
          return;
        
      std::string cmd = pmt::symbol_to_string(pmt::car(msg));
      pmt::pmt_t dict = pmt::cdr(msg);

      if (cmd == "PHY-RXCALCTONEMAP.request") {
        dout << d_name << ": recalculating tone map" << std::endl;
        light_plc::tone_map_t tone_map = d_phy_service.calculate_tone_map(0.001);
        d_phy_service.set_tone_map(tone_map);
        pmt::pmt_t tone_map_pmt = pmt::make_u8vector(tone_map.size(), 0);
        size_t len;
        uint8_t *tone_map_blob = (uint8_t*)pmt::u8vector_writable_elements(tone_map_pmt, len);
        for (size_t j=0; j<len; j++)
          tone_map_blob[j] = (uint8_t)tone_map[j];
        pmt::pmt_t dict = pmt::make_dict();
        dict = pmt::dict_add(dict, pmt::mp("tone_map"), tone_map_pmt);
        message_port_pub(pmt::mp("mac out"), pmt::cons(pmt::mp("PHY-RXCALCTONEMAP.response"), dict));
      } 

      else if (cmd == "PHY-RXUTILPAYLOAD") {
        dout << d_name << ": utilizing payload" << std::endl;
        d_phy_service.utilize_payload();
      } 

      else if (cmd == "PHY-RXINIT") {
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
            std::cout << sync_tone_mask_len;
            assert(sync_tone_mask_len == sync_tone_mask.size());
            for (size_t j = 0; j<sync_tone_mask_len; j++)
              sync_tone_mask[j] = sync_tone_mask_blob[j];

            d_phy_service = light_plc::phy_service(tone_mask, tone_mask, sync_tone_mask);
            //d_phy_service.debug(d_debug);
          }

          if (pmt::dict_has_key(dict,pmt::mp("id"))) 
            d_name = "PHY Rx (" + pmt::symbol_to_string(pmt::dict_ref(dict, pmt::mp("id"), pmt::PMT_NIL)) + ")";

          d_init_done = true;

          // Set the correlation filter
          light_plc::vector_float syncp (d_phy_service.preamble() + SYNCP_SIZE * 7.5, d_phy_service.preamble() + SYNCP_SIZE * 8.5);
          std::reverse(syncp.begin(), syncp.end());
          d_fir = new gr::filter::kernel::fir_filter_fff(1, syncp);    
          d_correlation = gr::fft::malloc_float(SYNC_LENGTH); 

          // Init some vectors
          d_preamble = light_plc::vector_float(PREAMBLE_SIZE);
          d_frame_control = light_plc::vector_float(FRAME_CONTROL_SIZE);

          d_receiver_state = RESET;
          dout << d_name << ": init done" << std::endl;
        } else 
          std::cerr << d_name << ": ERROR: cannot init more than once" << std::endl;
      }
    }

    void
    phy_rx_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      if (d_receiver_state == SYNC) {
        ninput_items_required[0] = SYNC_LENGTH + SYNCP_SIZE - 1;
        ninput_items_required[1] = SYNC_LENGTH + SYNCP_SIZE - 1;
        ninput_items_required[2] = SYNC_LENGTH + SYNCP_SIZE - 1;
      } else if (d_receiver_state == RESET) {
        ninput_items_required[0] = 2 * SYNCP_SIZE;
        ninput_items_required[1] = 2 * SYNCP_SIZE;
        ninput_items_required[2] = 2 * SYNCP_SIZE;
      } else if (d_receiver_state == SENSE_SPACE) {
        ninput_items_required[0] = d_phy_service.get_inter_frame_space();
        ninput_items_required[1] = d_phy_service.get_inter_frame_space();
        ninput_items_required[2] = d_phy_service.get_inter_frame_space();
      } else {
        ninput_items_required[0] = noutput_items;
        ninput_items_required[1] = noutput_items;
        ninput_items_required[2] = noutput_items;
      }
    }

    int
    phy_rx_impl::work(int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items)
    {
      const float *in = (const float *) input_items[0]; 
      int ninput = noutput_items;
      int i = 0;

      switch(d_receiver_state) {

        case SEARCH:
          while (i + 2 * SYNCP_SIZE < ninput) {
            d_search_corr += (in[i + SYNCP_SIZE] * in[i + SYNCP_SIZE * 2] - in[i] * in[i + SYNCP_SIZE]);
            d_energy += (in[i + SYNCP_SIZE * 2] * in[i + SYNCP_SIZE * 2] - in[i + SYNCP_SIZE] * in[i + SYNCP_SIZE]);
            if(d_energy > MIN_ENERGY && d_search_corr / d_energy > THRESHOLD) {
              if(d_plateau < MIN_PLATEAU) {
                d_plateau++;
              } else {
                dout << d_name << ": state = SEARCH, Found frame!" << std::endl;
                i += 2 * SYNCP_SIZE;                
                d_receiver_state = SYNC;
                break;
              }   
            } else {
              d_plateau = 0;
            }
            d_preamble[d_preamble_offset++] = in[i + 2 * SYNCP_SIZE];
            d_preamble_offset = d_preamble_offset % PREAMBLE_SIZE;
            i++;
          }
          break;

        case SYNC: {
          d_fir->filterN(d_correlation, in, SYNC_LENGTH); // in[] should have at least SYNC_LENGTH + SYNCP_SIZE elements
          int max_index = 0;
          float max_value = d_correlation[0] * d_correlation[SYNCP_SIZE];
          
          for (int j=0; j < SYNC_LENGTH - SYNCP_SIZE; j++) {
            if ((d_correlation[j] * d_correlation[j + SYNCP_SIZE]) > max_value) {
              max_index = j;
              max_value = d_correlation[j] * d_correlation[j + SYNCP_SIZE];
            }
          }

          dout << d_name << ": state = SYNC, max_index = " << max_index << ", max_value = "  << max_value << std::endl;
          d_frame_start = 2.5 * SYNCP_SIZE + max_index;
          d_receiver_state = COPY_PREAMBLE;
          break;
        }

        case COPY_PREAMBLE:
          dout << d_name << ": state = COPY_PREAMBLE, d_sync_offset = " << d_sync_offset << std::endl;
          while (i < ninput && d_sync_offset < d_frame_start) {
            d_preamble[d_preamble_offset++] = in[i];
            d_preamble_offset = d_preamble_offset % PREAMBLE_SIZE;
            d_sync_offset++;
            i++;
          }
          if (d_sync_offset == d_frame_start) {
            light_plc::vector_float preamble_aligned (d_preamble.size());
            light_plc::vector_float::iterator preamble_aligned_iter (preamble_aligned.begin());
            preamble_aligned_iter = std::copy(d_preamble.begin() + d_preamble_offset, d_preamble.end(), preamble_aligned_iter);
            std::copy(d_preamble.begin(), d_preamble.begin() + d_preamble_offset, preamble_aligned_iter);
            d_phy_service.process_ppdu_preamble(preamble_aligned.begin(), preamble_aligned.end());
            d_receiver_state = COPY_FRAME_CONTROL;
          }
          break;

        case COPY_FRAME_CONTROL:
          dout << d_name << ": state = COPY_FRAME_CONTROL" << std::endl;
          while (i < ninput) {
            if (d_frame_control_offset < FRAME_CONTROL_SIZE) {
              d_frame_control[d_frame_control_offset] = in[i];
            } else  {
              d_frame_control_pmt = pmt::make_u8vector(light_plc::phy_service::FRAME_CONTROL_SIZE, 0);
              size_t len;
              unsigned char *fc_blob = (unsigned char*)pmt::u8vector_writable_elements(d_frame_control_pmt, len);
              if (d_phy_service.process_ppdu_frame_control(d_frame_control.begin(), fc_blob) == false) {
                std::cerr << d_name << ": state = COPY_FRAME_CONTROL, ERROR: cannot parse frame control" << std::endl;
                d_receiver_state = RESET;
              } else {
                d_receiver_state = COPY_PAYLOAD;
                d_payload_size = d_phy_service.get_ppdu_payload_length();
                dout << d_name << ": Frame control is OK!" << std::endl;
                d_payload = light_plc::vector_float(d_payload_size);
              }
              break;
            }
            i++;
            d_frame_control_offset++;
          }
          break;

        case COPY_PAYLOAD: {
          int k = std::min(d_payload_size - d_payload_offset, ninput);
          std::copy(in, in + k, d_payload.begin() + d_payload_offset);
          d_payload_offset += k;
          i += k;
          if (d_payload_offset == d_payload_size) {
            pmt::pmt_t payload_pmt = pmt::make_u8vector(d_phy_service.get_mpdu_payload_size(), 0);
            size_t len;
            unsigned char *payload_blob = (unsigned char*)pmt::u8vector_writable_elements(payload_pmt, len);
            d_phy_service.process_ppdu_payload(d_payload.begin(), payload_blob);      // get payload data
            dout << d_name << ": payload resolved. Payload size (bytes) = " << d_phy_service.get_mpdu_payload_size() << std::endl;
            pmt::pmt_t dict = pmt::make_dict();
            dict = pmt::dict_add(dict, pmt::mp("frame_control"), d_frame_control_pmt);  // add frame control information  
            dict = pmt::dict_add(dict, pmt::mp("payload"), payload_pmt);              
            message_port_pub(pmt::mp("mac out"), pmt::cons(pmt::mp("PHY-RXSTART"), dict));
            d_receiver_state = SENSE_SPACE;
          }
          break;
        }

        case CONSUME_SPACE: {
          int j = std::min(d_phy_service.get_inter_frame_space() - d_inter_frame_space_offset, ninput);
          i += j;
          d_inter_frame_space_offset += j;
          dout << d_name << ": state = CONSUME_SPACE" << std::endl;
          if (d_inter_frame_space_offset == d_phy_service.get_inter_frame_space()) {
            pmt::pmt_t dict = pmt::make_dict();
            message_port_pub(pmt::mp("mac out"), pmt::cons(pmt::mp("PHY-RXEND"), dict));
            d_receiver_state = RESET;
          }
          break;
        }

        case SENSE_SPACE: {
          if (d_inter_frame_space_offset == 0)
            d_noise = std::vector<float>(d_phy_service.get_inter_frame_space());
          int j = std::min(d_phy_service.get_inter_frame_space() - d_inter_frame_space_offset, ninput);
          // Get noise data
          while (i<j)
            d_noise[d_inter_frame_space_offset++] = in[i++];
          if (d_inter_frame_space_offset == d_phy_service.get_inter_frame_space()) {
            // Calculate noise PSD
            d_phy_service.process_noise(d_noise.begin(), d_noise.end());
            dout << d_name << ": state = SENSE_SPACE, length = " << d_phy_service.get_inter_frame_space() <<  std::endl;
            pmt::pmt_t dict = pmt::make_dict();
            message_port_pub(pmt::mp("mac out"), pmt::cons(pmt::mp("PHY-RXEND"), dict));
            d_receiver_state = RESET;
          }
          break;
        }

        case RESET:
          dout << d_name << ": state = RESET" << std::endl;
          d_plateau = 0;
          d_frame_control_offset = 0;
          d_sync_offset = 0;
          d_preamble_offset = 0;
          d_frame_control_offset = 0;
          d_payload_size = 0;
          d_payload_offset = 0;
          d_cor.clear();
          d_output_datastream_offset = 0;
          d_output_datastream_len = 0;
          d_noise_offset = 0;
          d_search_corr = 0;
          d_energy = 0;
          d_inter_frame_space_offset = 0;
          for (int j=0; j<SYNCP_SIZE; j++) {
            d_search_corr += in[j] * in[j + SYNCP_SIZE];
            d_energy += in[j + SYNCP_SIZE] * in[j + SYNCP_SIZE];
            d_preamble[d_preamble_offset++] = in[j * 2];
            d_preamble[d_preamble_offset++] = in[j * 2 + 1];
          }
          d_receiver_state = SEARCH;
          break;

       case IDLE:
         dout << d_name << ": state = IDLE, ninput = " << ninput << std::endl;       
         i = ninput;

        case HALT:
            break;
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (i);
      return 0;
    }

  } /* namespace plc */
} /* namespace gr */


