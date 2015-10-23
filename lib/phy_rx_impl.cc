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
              gr::io_signature::make(3, 3, sizeof(float)),
              gr::io_signature::make(0, 0, 0)),
            d_robo_mode(robo_mode),
            d_modulation(modulation),
            d_debug (debug),
            d_receiver_state(RX_RESET),
            d_plateau(0),
            d_payload_size(0),
            d_payload_offset(0),
            d_sync_offset(0),
            d_frame_control_offset(0),
            d_preamble_offset(0),
            d_frame_start(0),
            d_output_datastream_offset(0),
            d_output_datastream_len(0)
    {
      message_port_register_out(pmt::mp("mac out"));
      d_plcp = light_plc::plcp();
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

    void
    phy_rx_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      if (d_receiver_state == SYNC) {
        ninput_items_required[0] = SYNC_LENGTH + SYNCP_SIZE - 1;
        ninput_items_required[1] = SYNC_LENGTH + SYNCP_SIZE - 1;
        ninput_items_required[2] = SYNC_LENGTH + SYNCP_SIZE - 1;
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
      const float *in0 = (const float *) input_items[0]; // auto-correlation 
      const float *in1 = (const float *) input_items[1]; // energy
      const float *in2 = (const float *) input_items[2]; // raw data
      int ninput = noutput_items;
      //int ninput = std::min(ninput_items[0], ninput_items[1]);
      int i = 0;

      switch(d_receiver_state) {

        case SEARCH:
          while (i < ninput) {
            if(in1[i] > MIN_ENERGY && in0[i]/in1[i] > THRESHOLD) {
              if(d_plateau < MIN_PLATEAU) {
                d_plateau++;
              } else {
                dout << "PHY Receiver: state = SEARCH, Found frame!" << std::endl;
                d_receiver_state = SYNC;
                break;
              }   
            } else {
              d_plateau = 0;
            }
            d_preamble[d_preamble_offset++] = in2[i];
            d_preamble_offset = d_preamble_offset % PREAMBLE_SIZE;
            i++;
          }
          break;

        case SYNC: {
          dout << "PHY Receiver: state = SYNC, d_sync_offset = " << d_sync_offset << " ninput = " << ninput << std::endl;
          d_fir->filterN(d_correlation, in2, SYNC_LENGTH);
          
          int m_index = 0;
          float m_value = d_correlation[0] * d_correlation[SYNCP_SIZE];
          for (; i < SYNC_LENGTH - SYNCP_SIZE; i++) {
            d_preamble[d_preamble_offset++] = in2[i];
            d_preamble_offset = d_preamble_offset % PREAMBLE_SIZE;
            if ((d_correlation[i] * d_correlation[i+SYNCP_SIZE]) > m_value) {
              m_index = i + SYNCP_SIZE;
              m_value = d_correlation[i] * d_correlation[i+SYNCP_SIZE];
            }
          }
          for (; i < SYNC_LENGTH; i++) {
            d_preamble[d_preamble_offset++] = in2[i];
            d_preamble_offset = d_preamble_offset % PREAMBLE_SIZE;
          }
          dout << "PHY Receiver: state = SYNC, m_index = " << m_index << ", " << "m_value = "  << m_value << std::endl;
          unsigned int sync_pos = m_index;
                   
          d_frame_start = 1.5 * SYNCP_SIZE + sync_pos - i;  // frame begins 1.5 syncp after last min
          if (d_frame_start < 0) {
            d_receiver_state = RX_RESET; // If sync_pos does not make sense
          } else {
            d_receiver_state = CHANNEL_ESTIMATE;
          }
          break;
        }

        case CHANNEL_ESTIMATE:
          dout << "PHY Receiver: state = CHANNEL_ESTIMATE, d_frame_control_offset = " << d_frame_start << std::endl;
          while (i < ninput && d_sync_offset < d_frame_start) {
            d_preamble[d_preamble_offset++] = in2[i];
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
              d_frame_control[d_frame_control_offset] = in2[i];
            } else  {
              d_payload_size = d_plcp.resolve_frame_control(d_frame_control.begin(), light_plc::RATE_1_2, d_modulation);
              if (d_payload_size == -1) {
                std::cerr << "PHY Receiver: state = COPY_FRAME_CONTROL, ERROR: cannot parse frame control" << std::endl;
                d_receiver_state = RX_RESET;
              } else {
                d_receiver_state = COPY_PAYLOAD;
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
          std::copy(in2, in2 + k, d_payload.begin() + d_payload_offset);
          d_payload_offset += k;
          i += k;
          if (d_payload_offset == d_payload_size) {
            pmt::pmt_t mpdu_payload_pmt = pmt::make_u8vector(d_plcp.payload_size(), 0);
            size_t len;
            unsigned char *mpdu_payload_blob = (unsigned char*)pmt::u8vector_writable_elements(mpdu_payload_pmt, len);
            d_plcp.resolve_payload(d_payload.begin(), mpdu_payload_blob);
            dout << "PHY Receiver: payload resolved. Payload size (bytes) = " << d_plcp.payload_size() << ", type = " << d_plcp.frame_type() << std::endl;
            if (d_plcp.frame_type() == light_plc::MPDU_TYPE_SOF) {
              // dict
              pmt::pmt_t dict = pmt::make_dict();

              // mpdu
              message_port_pub(pmt::mp("mac out"), pmt::cons(dict, mpdu_payload_pmt));
            }

            d_receiver_state = RX_RESET;
          }
          break;
        }

        case RX_RESET:
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
          d_receiver_state = SEARCH;
          break;
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (i);
      return 0;
    }

  } /* namespace plc */
} /* namespace gr */


