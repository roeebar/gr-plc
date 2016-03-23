/* -*- c++ -*- */

#ifndef INCLUDED_PLC_PHY_RX_H
#define INCLUDED_PLC_PHY_RX_H

#include <plc/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace plc {

    /*!
     * \brief <+description of block+>
     * \ingroup plc
     *
     */
    class PLC_API phy_rx : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<phy_rx> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of plc::phy_rx.
       *
       * To avoid accidental use of raw pointers, plc::phy_rx's
       * constructor is in a private implementation
       * class. plc::phy_rx::make is the public interface for
       * creating new instances.
       */
      static sptr make(bool info, int debug_level);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_PHY_RX_H */
