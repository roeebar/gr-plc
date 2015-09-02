/* -*- c++ -*- */

#ifndef INCLUDED_PLC_MAC_RECEIVER_H
#define INCLUDED_PLC_MAC_RECEIVER_H

#include <plc/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace plc {

    /*!
     * \brief <+description of block+>
     * \ingroup plc
     *
     */
    class PLC_API mac_receiver : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<mac_receiver> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of plc::mac_receiver.
       *
       * To avoid accidental use of raw pointers, plc::mac_receiver's
       * constructor is in a private implementation
       * class. plc::mac_receiver::make is the public interface for
       * creating new instances.
       */
      static sptr make(char *output_filename, bool debug = false);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_MAC_RECEIVER_H */

