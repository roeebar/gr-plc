/* -*- c++ -*- */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "impulse_noise_source_impl.h"
#include <complex>

namespace gr {
  namespace plc {

    impulse_noise_source::sptr
    impulse_noise_source::make(float samp_rate)
    {
      return gnuradio::get_initial_sptr
        (new impulse_noise_source_impl(samp_rate));
    }

    /*
     * The private constructor
     */
    impulse_noise_source_impl::impulse_noise_source_impl(float samp_rate)
      : gr::sync_block("impulse_noise_source",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_samp_rate(samp_rate)
    {}

    /*
     * Our virtual destructor.
     */
    impulse_noise_source_impl::~impulse_noise_source_impl()
    {
    }

    int
    impulse_noise_source_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      gr_complex *out = (gr_complex *) output_items[0];
      memset(out, 0, noutput_items * sizeof(gr_complex)); // zero output

      for (size_t j=0; j<d_impulses.size(); j++) {
        int i = 0; // output
        int pos = d_impulses[j].pos; // impulse last position
        std::vector<gr_complex> &s = d_impulses[j].signal; // impulse signal
        int len = s.size(); // impulse length
        int k = pos; // signal position
        int m = std::min(len-pos, noutput_items);
        while (i < m) // copy remainder of previous iteration
          out[i++] += s[k++];

        k = 0;
        while (i < noutput_items) // keep filling with noise till no more room
            out[i++] += s[k++ % len];

        d_impulses[j].pos = (pos + i) % len; // update position
      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    void impulse_noise_source_impl::add_noise(float iat, float A, float l, float f, float offset) {
      std::cout <<  "I" << d_impulses.size() << ": IAT=" << iat <<  " A=" << A <<" l=" << l  << " f=" << f << " offset=" << offset << std::endl;
      d_impulses.push_back(create_impulse(iat, A, l, f, offset));
    }

    impulse_noise_source_impl::impulse_t impulse_noise_source_impl::create_impulse(float iat, float A, float l, float f, float offset) {
      impulse_t impulse;

      size_t N = (size_t)(iat * d_samp_rate);
      size_t samples_offset = floor(offset * N);
      std::vector<float> t(N);
      for (size_t i=0; i<N; i++)
        t[i] = i/d_samp_rate;

      int j = 0;
      impulse.signal = std::vector<gr_complex>(N);
      for (size_t i=N-samples_offset; i<N; i++) {
        impulse.signal[j++] = A * std::exp(gr_complex(-l * t[i], 2 * M_PI * f * t[i] - M_PI / 2));
      }
      for (size_t i=0; i<N-samples_offset; i++) {
        impulse.signal[j++] = A * std::exp(gr_complex(-l * t[i], 2 * M_PI * f * t[i] - M_PI / 2));
      }
      impulse.pos = 0;
      return impulse;
    }

  } /* namespace plc */
} /* namespace gr */
