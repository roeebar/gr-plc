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

    const int phy_rx_impl::SYNCP_SIZE = light_plc::plcp::SYNCP_SIZE;
    const int phy_rx_impl::SYNC_LENGTH = 2 * phy_rx_impl::SYNCP_SIZE; // length for frame alignment attempt
    const int phy_rx_impl::PREAMBLE_SIZE = light_plc::plcp::PREAMBLE_SIZE;
    const int phy_rx_impl::FRAME_CONTROL_SIZE = light_plc::plcp::FRAME_CONTROL_SIZE;
    const float phy_rx_impl::THRESHOLD = 0.9; // autocorrelation threshold
    const float phy_rx_impl::MIN_ENERGY = 1e-3; // signal minimum energy
    const int phy_rx_impl::MIN_PLATEAU = 5.5 * phy_rx_impl::SYNCP_SIZE; // minimum autocorrelation plateau

    phy_rx::sptr
    phy_rx::make(int robo_mode, int modulation, bool debug)
    {
      return gnuradio::get_initial_sptr
        (new phy_rx_impl((light_plc::RoboMode)robo_mode, (light_plc::Modulation) modulation, debug));
    }

    /*
     * The private constructor
     */
    phy_rx_impl::phy_rx_impl(light_plc::RoboMode robo_mode, light_plc::Modulation modulation, bool debug)
      : gr::sync_block("phy_rx",
              gr::io_signature::make(1, 1, sizeof(float)),
              gr::io_signature::make(0, 0, 0)),
            d_robo_mode(robo_mode),
            d_modulation(modulation),
            d_debug (debug),
            d_receiver_state(RESET),
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
            d_inter_frame_space_offset(0)
    {

      message_port_register_out(pmt::mp("mac out"));
      message_port_register_in(pmt::mp("mac in"));
      set_msg_handler(pmt::mp("mac in"), boost::bind(&phy_rx_impl::mac_in, this, _1));

      d_plcp = light_plc::plcp();
      d_plcp.set_modulation(d_modulation);
      d_plcp.set_robo_mode(d_robo_mode);      
      //d_plcp.debug(d_debug);

      // Set the correlation filter
      light_plc::vector_float syncp (d_plcp.preamble() + SYNCP_SIZE * 7.5, d_plcp.preamble() + SYNCP_SIZE * 8.5);
      std::reverse(syncp.begin(), syncp.end());
      d_fir = new gr::filter::kernel::fir_filter_fff(1, syncp);    
      d_correlation = gr::fft::malloc_float(SYNC_LENGTH); 

      // Init some vectors
      d_preamble = light_plc::vector_float(PREAMBLE_SIZE); 
      d_frame_control = light_plc::vector_float(FRAME_CONTROL_SIZE);        
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
      if (pmt::is_pair(msg)) {
        if (!pmt::is_symbol(car(msg)))
          return;
        std::string cmd = pmt::symbol_to_string(car(msg));
        if (cmd == "PHY-SEARCHPPDU")
          d_receiver_state = RESET;         
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
        ninput_items_required[0] = d_plcp.get_inter_frame_space();
        ninput_items_required[1] = d_plcp.get_inter_frame_space();
        ninput_items_required[2] = d_plcp.get_inter_frame_space();
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
          while (i + 2*SYNCP_SIZE < ninput) {
            d_search_corr += (in[i + SYNCP_SIZE] * in[i + SYNCP_SIZE * 2] - in[i] * in[i + SYNCP_SIZE]);
            d_energy += (in[i + SYNCP_SIZE] * in[i + SYNCP_SIZE] - in[i] * in[i]);
            if(d_energy > MIN_ENERGY && d_search_corr / d_energy > THRESHOLD) {
              if(d_plateau < MIN_PLATEAU) {
                d_plateau++;
              } else {
                dout << "PHY Receiver: state = SEARCH, Found frame!" << std::endl;
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
          dout << "PHY Receiver: state = SYNC, d_sync_offset = " << d_sync_offset << " ninput = " << ninput << std::endl;
          d_fir->filterN(d_correlation, in, SYNC_LENGTH);
          
          int max_index = 0;
          float max_value = d_correlation[0] * d_correlation[SYNCP_SIZE];
          for (; i < SYNC_LENGTH - SYNCP_SIZE; i++) {
            d_preamble[d_preamble_offset++] = in[i];
            d_preamble_offset = d_preamble_offset % PREAMBLE_SIZE;
            if ((d_correlation[i] * d_correlation[i + SYNCP_SIZE]) > max_value) {
              max_index = i;
              max_value = d_correlation[i] * d_correlation[i + SYNCP_SIZE];
            }
          }
          for (; i < SYNC_LENGTH; i++) {
            d_preamble[d_preamble_offset++] = in[i];
            d_preamble_offset = d_preamble_offset % PREAMBLE_SIZE;
          }
          dout << "PHY Receiver: state = SYNC, max_index = " << max_index << ", " << "max_value = "  << max_value << std::endl;
                   
          d_frame_start = 2.5 * SYNCP_SIZE + max_index - i;  // frame begins 1.5 syncp after last min
          if (d_frame_start < 0) {
            d_receiver_state = RESET; // If sync_pos does not make sense
          } else {
            d_receiver_state = COPY_PREAMBLE;
          }
          break;
        }

        case COPY_PREAMBLE:
          dout << "PHY Receiver: state = COPY_PREAMBLE, d_sync_offset = " << d_sync_offset << std::endl;
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
            d_plcp.resolve_preamble(preamble_aligned.begin(), preamble_aligned.end());
            d_receiver_state = COPY_FRAME_CONTROL;
          }
          break;

        case COPY_FRAME_CONTROL:
          dout << "PHY Receiver: state = COPY_FRAME_CONTROL" << std::endl;
          while (i < ninput) {
            if (d_frame_control_offset < FRAME_CONTROL_SIZE) {
              d_frame_control[d_frame_control_offset] = in[i];
            } else  {
              if (d_plcp.resolve_frame_control(d_frame_control.begin()) == false) {
                std::cerr << "PHY Receiver: state = COPY_FRAME_CONTROL, ERROR: cannot parse frame control" << std::endl;
                d_receiver_state = RESET;
              } else {
                d_receiver_state = COPY_PAYLOAD;
                d_payload_size = d_plcp.get_ppdu_payload_size();
                dout << "PHY Receiver: Frame control is OK!" << std::endl;
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
            pmt::pmt_t payload_pmt = pmt::make_u8vector(d_plcp.get_mpdu_payload_size(), 0);
            size_t len;
            unsigned char *payload_blob = (unsigned char*)pmt::u8vector_writable_elements(payload_pmt, len);
            d_plcp.resolve_payload(d_payload.begin(), payload_blob);
            dout << "PHY Receiver: payload resolved. Payload size (bytes) = " << d_plcp.get_mpdu_payload_size() << ", type = " << d_plcp.get_frame_type() << std::endl;
            if (d_plcp.get_frame_type() == light_plc::MPDU_TYPE_SOF) {
              pmt::pmt_t dict = pmt::make_dict();
              //dict = pmt::dict_add(dict, pmt::mp("type"), pmt::mp("sof"));
              dict = pmt::dict_add(dict, pmt::mp("payload"), payload_pmt);              
              message_port_pub(pmt::mp("mac out"), pmt::cons(pmt::mp("PHY-RXSOF"), dict));
              //message_port_pub(pmt::mp("mac out"), pmt::cons(dict, payload_pmt));
              d_receiver_state = CONSUME_SPACE;
            } else if (d_plcp.get_frame_type() == light_plc::MPDU_TYPE_SACK) {
              pmt::pmt_t sackd_pmt = pmt::make_u8vector(d_plcp.get_sackd_size(), 0);
              size_t len;              
              d_plcp.get_sackd((unsigned char*)pmt::u8vector_writable_elements(sackd_pmt, len));
              pmt::pmt_t dict = pmt::make_dict();
              dict = pmt::dict_add(dict, pmt::mp("sackd"), sackd_pmt);
              message_port_pub(pmt::mp("mac out"), pmt::cons(pmt::mp("PHY-RXSACK"), dict));
              d_receiver_state = CONSUME_SPACE;
            } else if (d_plcp.get_frame_type() == light_plc::MPDU_TYPE_SOUND) {
              pmt::pmt_t dict = pmt::make_dict();
              message_port_pub(pmt::mp("mac out"), pmt::cons(pmt::mp("PHY-RXSOUND"), dict));
              d_receiver_state = SENSE_SPACE;
            } else {
              std::cerr << "PHY Receiver: Error: unsupported frame type" << std::endl;
            }
          }
          break;
        }

        case CONSUME_SPACE: {
          int j = std::min(d_plcp.get_inter_frame_space() - d_inter_frame_space_offset, ninput);
          i += j;
          d_inter_frame_space_offset += j;
          if (d_inter_frame_space_offset == d_plcp.get_inter_frame_space()) {
            pmt::pmt_t dict = pmt::make_dict();
            message_port_pub(pmt::mp("mac out"), pmt::cons(pmt::mp("PHY-RXEND"), dict));
            d_receiver_state = HALT;
          }
          dout << "PHY Receiver: state = CONSUME_SPACE" << std::endl;
          break;
        }

        case SENSE_SPACE: {
          if (d_inter_frame_space_offset == 0)
            d_noise = std::vector<float>(d_plcp.get_inter_frame_space());
          int j = std::min(d_plcp.get_inter_frame_space() - d_inter_frame_space_offset, ninput);
          // Get noise data
          while (i<j)
            d_noise[d_inter_frame_space_offset++] = in[i++];
          if (d_inter_frame_space_offset == d_plcp.get_inter_frame_space()) {
            // Calculate noise PSD
            float noise_var(0);
            for (std::vector<float>::const_iterator iter = d_noise.begin(); iter != d_noise.end(); iter++)
              noise_var += (*iter)*(*iter)/d_plcp.get_inter_frame_space();
            d_plcp.set_noise_psd(noise_var * 2);
            dout << "PHY Receiver: state = SENSE_SPACE, length = " << d_plcp.get_inter_frame_space() << ", estimated noise psd = " << noise_var*2 << std::endl;
            pmt::pmt_t dict = pmt::make_dict();
            message_port_pub(pmt::mp("mac out"), pmt::cons(pmt::mp("PHY-RXEND"), dict));
            d_receiver_state = HALT;
          }
          break;
        }

        case RESET:
          dout << "PHY Receiver: state = RESET" << std::endl;
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
            d_energy += in[j] * in[j];
            d_preamble[d_preamble_offset++] = in[j * 2];
            d_preamble[d_preamble_offset++] = in[j * 2 + 1];
          }
          d_receiver_state = SEARCH;
          break;

       case IDLE:
         dout << "PHY Receiver: state = IDLE, ninput = " << ninput << std::endl;       
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


