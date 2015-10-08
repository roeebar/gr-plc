/* -*- c++ -*- */

#ifndef INCLUDED_PLC_PHY_H
#define INCLUDED_PLC_PHY_H

#include <plc/api.h>
#include <gnuradio/block.h>
namespace gr {
  namespace plc {

    /*!
     * \brief <+description of block+>
     * \ingroup plc
     *
     */
    class PLC_API phy : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<phy> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of plc::phy.
       *
       * To avoid accidental use of raw pointers, plc::phy's
       * constructor is in a private implementation
       * class. plc::phy::make is the public interface for
       * creating new instances.
       */
      static sptr make(int robo_mode, int modulation, bool disable_transmitter, bool disable_receiver, bool debug);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_PHY_H */

