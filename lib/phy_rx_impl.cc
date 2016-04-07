/* -*- c++ -*- */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "phy_rx_impl.h"
#include "debug.h"
#include <gnuradio/fft/fft.h>
#include <volk/volk.h>

namespace gr {
  namespace plc {

    const int phy_rx_impl::SYNCP_SIZE = light_plc::phy_service::SYNCP_SIZE;
    const int phy_rx_impl::PREAMBLE_SIZE = light_plc::phy_service::PREAMBLE_SIZE;
    const int phy_rx_impl::FRAME_CONTROL_SIZE = light_plc::phy_service::FRAME_CONTROL_SIZE;
    const int phy_rx_impl::MIN_INTERFRAME_SPACE = light_plc::phy_service::MIN_INTERFRAME_SPACE;
    const size_t phy_rx_impl::BUFFER_SIZE = light_plc::phy_service::MIN_INTERFRAME_SPACE + phy_rx_impl::PREAMBLE_SIZE;
    const int phy_rx_impl::MAX_SEARCH_LENGTH = 16384; // maximum search length determines the volk memory allocation
    const int phy_rx_impl::COARSE_SYNC_LENGTH = 2 * phy_rx_impl::SYNCP_SIZE; // length for frame alignment attempt
    const int phy_rx_impl::FINE_SYNC_LENGTH = 10; // length for fine frame alignment attempt
    const float phy_rx_impl::THRESHOLD = 0.9; // autocorrelation threshold
    const int phy_rx_impl::MIN_PLATEAU = 5.5 * phy_rx_impl::SYNCP_SIZE - light_plc::phy_service::ROLLOFF_INTERVAL; // minimum autocorrelation plateau

    phy_rx::sptr
    phy_rx::make(bool info, int debug_level)
    {
      return gnuradio::get_initial_sptr
        (new phy_rx_impl(info, debug_level));
    }

    /*
     * The private constructor
     */
    phy_rx_impl::phy_rx_impl(bool info, int debug_level)
      : gr::sync_block("phy_rx",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0, 0, 0)),
            d_debug_level(debug_level),
            d_info(info),
            d_qpsk_tone_mask(light_plc::tone_mask_t()),
            d_init_done(false),
            d_receiver_state(HALT),
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
      volk_free(d_buffer);
      volk_free(d_mult);
      volk_free(d_real);
      volk_free(d_energy);
      volk_free(d_frame_control);
      free(d_preamble_corr);
    }

    void phy_rx_impl::mac_in (pmt::pmt_t msg) {
      if (!(pmt::is_pair(msg) && pmt::is_symbol(pmt::car(msg)) && pmt::is_dict(pmt::cdr(msg))))
          return;

      std::string cmd = pmt::symbol_to_string(pmt::car(msg));
      pmt::pmt_t dict = pmt::cdr(msg);

      if (cmd == "PHY-RXCALCTONEMAP.request") {
        dout << d_name << ": recalculating tone map" << std::endl;
        float target_ber = pmt::to_float(pmt::dict_ref(dict, pmt::mp("target_ber"), pmt::PMT_NIL));
        light_plc::tone_map_t tone_map = d_phy_service.calculate_tone_map(target_ber, d_qpsk_tone_mask);
        d_phy_service.set_tone_map(tone_map);
        pmt::pmt_t tone_map_pmt = pmt::make_u8vector(tone_map.size(), 0);
        size_t len;
        uint8_t *tone_map_blob = (uint8_t*)pmt::u8vector_writable_elements(tone_map_pmt, len);
        for (size_t j=0; j<len; j++)
          tone_map_blob[j] = (uint8_t)tone_map[j];
        pmt::pmt_t dict = pmt::make_dict();
        dict = pmt::dict_add(dict, pmt::mp("tone_map"), tone_map_pmt);
        message_port_pub(pmt::mp("mac out"), pmt::cons(pmt::mp("PHY-RXCALCTONEMAP.response"), dict));
        PRINT_INFO_VECTOR(d_phy_service.stats.snr,"snr");
      }

      else if (cmd == "PHY-RXPOSTPROCESSPAYLOAD") {
        dout << d_name << ": post processing payload" << std::endl;
        d_phy_service.post_process_ppdu_payload();
        PRINT_INFO_VAR(d_phy_service.stats.tone_mode, "toneMode");
        PRINT_INFO_VAR(d_phy_service.stats.n_bits, "nBits");
        PRINT_INFO_VAR(d_phy_service.stats.ber, "ber");
        if (d_info) {
          std::cout << "'" << d_name << "'; channelGain = [";
          for (auto iter=d_phy_service.stats.channel.begin(); iter != d_phy_service.stats.channel.end(); iter++)
            std::cout << std::abs(*iter) << ",";
          std::cout << "];" << std::endl;
        }
      }

      else if (cmd == "PHY-RXINIT") {
        if (!d_init_done) {

          if (pmt::dict_has_key(dict,pmt::mp("id")))
            d_name = "PHY Rx (" + pmt::symbol_to_string(pmt::dict_ref(dict, pmt::mp("id"), pmt::PMT_NIL)) + ")";

          light_plc::tone_mask_t tone_mask;
          light_plc::sync_tone_mask_t sync_tone_mask;
          if (pmt::dict_has_key(dict,pmt::mp("broadcast_tone_mask")) &&
              pmt::dict_has_key(dict,pmt::mp("sync_tone_mask")))
          {
            dout << d_name << ": initializing receiver" << std::endl;

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

            // Set channel estimation mode
            pmt::pmt_t channel_est_mode_pmt = pmt::dict_ref(dict, pmt::mp("channel_est_mode"), pmt::PMT_NIL);
            light_plc::channel_est_t channel_est_mode = (light_plc::channel_est_t)pmt::to_uint64(channel_est_mode_pmt);

            d_phy_service = light_plc::phy_service(tone_mask, tone_mask, sync_tone_mask, channel_est_mode, d_debug_level == 2);
          }

          if (pmt::dict_has_key(dict,pmt::mp("force_tone_mask"))) {
            dout << d_name << ": setting force tone mask" << std::endl;
            pmt::pmt_t tone_mask_pmt = pmt::dict_ref(dict, pmt::mp("force_tone_mask"), pmt::PMT_NIL);
            size_t tone_mask_len = 0;
            const uint8_t *tone_mask_blob = pmt::u8vector_elements(tone_mask_pmt, tone_mask_len);
            assert(tone_mask_len == d_qpsk_tone_mask.size());
            for (size_t j = 0; j<tone_mask_len; j++)
              d_qpsk_tone_mask[j] = tone_mask_blob[j];
          }

          d_init_done = true;

          // Init some vectors
          unsigned int alignment = volk_get_alignment();
          d_buffer = (gr_complex*)volk_malloc(sizeof(gr_complex) * BUFFER_SIZE, alignment); // buffer
          d_mult = (gr_complex*)volk_malloc(sizeof(gr_complex) * (MAX_SEARCH_LENGTH - SYNCP_SIZE), alignment); // for preamble correlation
          d_real = (float*)volk_malloc(sizeof(float) * (MAX_SEARCH_LENGTH - SYNCP_SIZE), alignment); // real part of preamble correlation
          d_energy = (float*)volk_malloc(sizeof(float) * MAX_SEARCH_LENGTH, alignment); // energy of preamble
          assert (MAX_SEARCH_LENGTH > COARSE_SYNC_LENGTH + 2 * SYNCP_SIZE); // the volk vectors are used during SYNC as well
          d_frame_control = (gr_complex*)volk_malloc(sizeof(gr_complex) * FRAME_CONTROL_SIZE, alignment); // frame control
          d_preamble_corr = (float*)malloc(SYNCP_SIZE * sizeof(float));

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
        ninput_items_required[0] = COARSE_SYNC_LENGTH + 2 * SYNCP_SIZE;
      } else if (d_receiver_state == COPY_PREAMBLE) {
        ninput_items_required[0] = d_frame_start;
      } else if (d_receiver_state == COPY_FRAME_CONTROL) {
        ninput_items_required[0] = FRAME_CONTROL_SIZE;
      } else if (d_receiver_state == RESET) {
        ninput_items_required[0] = 2 * SYNCP_SIZE;
      } else {
        ninput_items_required[0] = noutput_items;
      }
    }

    void phy_rx_impl::copy_to_circular_buffer(void *buffer, size_t buffer_size, size_t &buffer_offset, const void* src, size_t size, size_t datatype_size){
      buffer_size *= datatype_size;
      size *= datatype_size;
      buffer_offset *= datatype_size;
      buffer_offset = (buffer_offset + size) % buffer_size;
      size_t count = std::min(buffer_size, size); // number of samples to copy
      buffer_offset = (buffer_offset + buffer_size - count) % buffer_size;
      const void *src_start = (const uint8_t *)src + size - count; // position of source first sample
      size_t first_copy_count = std::min(count, buffer_size - buffer_offset); // length of first part
      memcpy((uint8_t*)buffer + buffer_offset, src_start, first_copy_count); // copy first part
      buffer_offset = (buffer_offset + first_copy_count) % buffer_size; // update offset pointer
      memcpy((uint8_t*)buffer + buffer_offset, (uint8_t*)src_start + first_copy_count, count - first_copy_count); // copy second part (if exists)
      buffer_offset = (buffer_offset + count - first_copy_count) % buffer_size; // update offset pointer
      buffer_offset /= datatype_size;
    }

    void phy_rx_impl::copy_from_circular_buffer(void *dest, void *buffer, size_t buffer_size, size_t buffer_offset, size_t size, size_t datatype_size){
      assert(size <= buffer_size); // cannot copy more than buffer size
      buffer_size *= datatype_size;
      size *= datatype_size;
      buffer_offset *= datatype_size;
      size_t start = (buffer_offset + buffer_size) % buffer_size; // first sample to copy
      size_t first_copy_count = std::min(buffer_size - start, size); // length of first part
      memcpy(dest, (uint8_t*)buffer + start, first_copy_count);  // copy first part
      memcpy((uint8_t*)dest + first_copy_count, buffer, size - first_copy_count);  // copy second part (if exists)
    }

    int
    phy_rx_impl::work(int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      int ninput = noutput_items;
      int i = 0;

      switch(d_receiver_state) {

        case SEARCH: {
          int search_len = std::min(ninput, MAX_SEARCH_LENGTH);
          volk_32fc_x2_multiply_conjugate_32fc(d_mult, in, in + SYNCP_SIZE, search_len - SYNCP_SIZE);
          volk_32fc_deinterleave_real_32f(d_real, d_mult, search_len - SYNCP_SIZE);
          volk_32fc_magnitude_squared_32f(d_energy, in, search_len);

          float correlation = 0;
          while (i + 2 * SYNCP_SIZE < search_len && d_plateau < MIN_PLATEAU) {
            d_search_corr += d_real[i + SYNCP_SIZE] - d_real[i]; // update correlation window
            d_energy_a += d_energy[i + SYNCP_SIZE] - d_energy[i]; // update energy window
            d_energy_b += d_energy[i + SYNCP_SIZE * 2] - d_energy[i + SYNCP_SIZE]; // update energy window
            correlation = d_search_corr / std::sqrt(d_energy_a*d_energy_b);
            if(correlation > THRESHOLD) {
              d_plateau++;
            } else { // correlation <= THRESHOLD
              d_plateau = 0;
            }
            i++;
          }

          copy_to_circular_buffer(d_buffer, BUFFER_SIZE, d_buffer_offset, in + 2 * SYNCP_SIZE, i, sizeof(gr_complex));

          // If plateau length is reached...
          if (d_plateau == MIN_PLATEAU) {
            dout << d_name << ": state = SEARCH, Found frame!" << std::endl;
            d_sync_min = correlation;
            d_sync_min_index = d_buffer_offset;
            d_receiver_state = SYNC;
          }
          break;
        }

        case SYNC: {
          // Perform coarse sync
          volk_32fc_x2_multiply_conjugate_32fc(d_mult, in, in + SYNCP_SIZE, COARSE_SYNC_LENGTH + SYNCP_SIZE);
          volk_32fc_deinterleave_real_32f(d_real, d_mult, COARSE_SYNC_LENGTH + SYNCP_SIZE);
          volk_32fc_magnitude_squared_32f(d_energy, in, COARSE_SYNC_LENGTH + 2 * SYNCP_SIZE);
          while (i < COARSE_SYNC_LENGTH) {
            d_search_corr += d_real[i + SYNCP_SIZE] - d_real[i]; // update correlation window
            d_energy_a += d_energy[i + SYNCP_SIZE] - d_energy[i]; // update energy window
            d_energy_b += d_energy[i + SYNCP_SIZE * 2] - d_energy[i + SYNCP_SIZE]; // update energy window
            i++;
            float correlation = d_search_corr / std::sqrt(d_energy_a * d_energy_b);
            if (correlation < d_sync_min) {
                d_sync_min = correlation;
                d_sync_min_index = (d_buffer_offset + i) % BUFFER_SIZE;
            }
          }
          copy_to_circular_buffer(d_buffer, BUFFER_SIZE, d_buffer_offset, in + 2 * SYNCP_SIZE, i, sizeof(gr_complex));
          i += 2 * SYNCP_SIZE;
          dout << d_name << ": state = SYNC, min = " << d_sync_min << std::endl;

          // Perform fine sync
          int N = SYNCP_SIZE;
          int start = (BUFFER_SIZE + d_sync_min_index - 7 * N - FINE_SYNC_LENGTH) % BUFFER_SIZE;
          float fine_sync_corr = 0;
          float fine_sync_min = 0;
          int fine_sync_min_index = -1;
          for (int k=0; k<N+2*FINE_SYNC_LENGTH; k++) {
            gr_complex p = d_buffer[(start + k + 5 * N) % BUFFER_SIZE];
            gr_complex m = d_buffer[(start + k + 6 * N) % BUFFER_SIZE];
            float new_y = std::real((d_buffer[(start + k) % BUFFER_SIZE] +
                           d_buffer[(start + k + N) % BUFFER_SIZE] +
                           d_buffer[(start + k + 2 * N) % BUFFER_SIZE] +
                           d_buffer[(start + k + 3 * N) % BUFFER_SIZE] +
                           d_buffer[(start + k + 4 * N) % BUFFER_SIZE]) * (std::conj(m-p)) + p * std::conj(m));
            fine_sync_corr = fine_sync_corr + new_y - d_preamble_corr[k % N];
            d_preamble_corr[k % N] = new_y;
            if (k == N-1) {
              fine_sync_min = fine_sync_corr;
              fine_sync_min_index = k;
            } else if (k >= N && fine_sync_corr < fine_sync_min) {
              fine_sync_min = fine_sync_corr;
              fine_sync_min_index = k;
            }
          }
          d_frame_start = 3 * (N / 2) - ((BUFFER_SIZE + d_buffer_offset - d_sync_min_index) % BUFFER_SIZE) + (fine_sync_min_index - N + 1 - FINE_SYNC_LENGTH);
          dout << d_name << ": state = SYNC, fine sync, min = " << fine_sync_min << ", correction = "  << fine_sync_min_index-N+1-FINE_SYNC_LENGTH<< std::endl;
          d_receiver_state = COPY_PREAMBLE;
          break;
        }

        case COPY_PREAMBLE: {
          dout << d_name << ": state = COPY_PREAMBLE" << std::endl;
          i+=d_frame_start;
          copy_to_circular_buffer(d_buffer, BUFFER_SIZE, d_buffer_offset, in, i, sizeof(gr_complex));

          // Process preamble
          light_plc::vector_complex preamble_aligned (PREAMBLE_SIZE);
          copy_from_circular_buffer(preamble_aligned.data(), d_buffer, BUFFER_SIZE, d_buffer_offset - PREAMBLE_SIZE, PREAMBLE_SIZE, sizeof(gr_complex));
          d_phy_service.process_ppdu_preamble(preamble_aligned.begin(), preamble_aligned.end());

          // Process noise
          light_plc::vector_complex noise_aligned (MIN_INTERFRAME_SPACE);
          copy_from_circular_buffer(noise_aligned.data(), d_buffer, BUFFER_SIZE, d_buffer_offset - PREAMBLE_SIZE - MIN_INTERFRAME_SPACE, MIN_INTERFRAME_SPACE, sizeof(gr_complex));
          d_phy_service.process_noise(noise_aligned.begin(), noise_aligned.end());

          // Print the calculated noise PSD
          PRINT_INFO_VECTOR(d_phy_service.stats.noise_psd, "noisePsd");

          d_receiver_state = COPY_FRAME_CONTROL;
          break;
        }

        case COPY_FRAME_CONTROL: {
          dout << d_name << ": state = COPY_FRAME_CONTROL" << std::endl;
          memcpy(d_frame_control, in, FRAME_CONTROL_SIZE * sizeof(gr_complex));
          i += FRAME_CONTROL_SIZE;

          d_frame_control_pmt = pmt::make_u8vector(light_plc::phy_service::FRAME_CONTROL_SIZE, 0);
          size_t len;
          unsigned char *fc_blob = (unsigned char*)pmt::u8vector_writable_elements(d_frame_control_pmt, len);
          if (d_phy_service.process_ppdu_frame_control((light_plc::vector_complex::const_iterator)d_frame_control, fc_blob) == false) {
            std::cerr << d_name << ": state = COPY_FRAME_CONTROL, ERROR: cannot parse frame control" << std::endl;
            d_receiver_state = RESET;
          } else {
            d_receiver_state = COPY_PAYLOAD;
            d_payload_size = d_phy_service.get_ppdu_payload_length();
            dout << d_name << ": frame control is OK!" << std::endl;
            d_payload = (gr_complex*)volk_malloc(sizeof(gr_complex) * d_payload_size, volk_get_alignment());
          }
          break;
        }

        case COPY_PAYLOAD: {
          i = std::min(d_payload_size - d_payload_offset, ninput);
          memcpy(d_payload + d_payload_offset, in, i * sizeof(gr_complex));
          d_payload_offset += i;
          if (d_payload_offset == d_payload_size) {
            pmt::pmt_t payload_pmt = pmt::make_u8vector(d_phy_service.get_mpdu_payload_size(), 0);
            size_t len;
            unsigned char *payload_blob = (unsigned char*)pmt::u8vector_writable_elements(payload_pmt, len);
            d_phy_service.process_ppdu_payload((light_plc::vector_complex::iterator)d_payload, payload_blob);      // get payload data
            PRINT_INFO_VECTOR(d_phy_service.stats.channel, "channelCarriers");
            dout << d_name << ": payload resolved. Payload size (bytes) = " << d_phy_service.get_mpdu_payload_size() << std::endl;
            pmt::pmt_t dict = pmt::make_dict();
            dict = pmt::dict_add(dict, pmt::mp("frame_control"), d_frame_control_pmt);  // add frame control information
            dict = pmt::dict_add(dict, pmt::mp("payload"), payload_pmt);
            message_port_pub(pmt::mp("mac out"), pmt::cons(pmt::mp("PHY-RXSTART"), dict));

            dict = pmt::make_dict();
            message_port_pub(pmt::mp("mac out"), pmt::cons(pmt::mp("PHY-RXEND"), dict));
            volk_free(d_payload);
            d_receiver_state = RESET;
          }
          break;
        }

        case RESET: {
          dout << d_name << ": state = RESET" << std::endl;
          d_sync_min = 1;
          d_plateau = 0;
          d_buffer_offset = 0;
          d_payload_size = 0;
          d_payload_offset = 0;
          d_search_corr = 0;
          d_energy_a = 0;
          d_energy_b = 0;
          volk_32fc_x2_multiply_conjugate_32fc(d_mult, in, in + SYNCP_SIZE, 2 * SYNCP_SIZE - 1);
          volk_32fc_deinterleave_real_32f(d_real, d_mult, 2 * SYNCP_SIZE - 1);
          volk_32fc_magnitude_squared_32f(d_energy, in, 2 * SYNCP_SIZE - 1);
          for (int j=0; j<SYNCP_SIZE; j++) {
            d_preamble_corr[j] = 0;
          }
          for (int j=0; j<SYNCP_SIZE-1; j++) {
            d_search_corr += d_real[j];
            d_energy_a += d_energy[j];
            d_energy_b += d_energy[j + SYNCP_SIZE]; // update energy window
          }
          copy_to_circular_buffer(d_buffer, BUFFER_SIZE, d_buffer_offset, in, 2 * SYNCP_SIZE - 1, sizeof(gr_complex));
          d_receiver_state = SEARCH;
          break;
        }

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
