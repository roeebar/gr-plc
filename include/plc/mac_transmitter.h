/* -*- c++ -*- */

#ifndef INCLUDED_PLC_MAC_TRANSMITTER_H
#define INCLUDED_PLC_MAC_TRANSMITTER_H

#include <plc/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace plc {

    /*!
     * \brief <+description of block+>
     * \ingroup plc
     *
     */
    class PLC_API mac_transmitter : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<mac_transmitter> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of plc::mac_transmitter.
       *
       * To avoid accidental use of raw pointers, plc::mac_transmitter's
       * constructor is in a private implementation
       * class. plc::mac_transmitter::make is the public interface for
       * creating new instances.
       */
      static sptr make(char *input_filename, float period_ms, bool debug = false);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_MAC_TRANSMITTER_H */

