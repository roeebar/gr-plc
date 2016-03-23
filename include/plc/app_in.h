/* -*- c++ -*- */

#ifndef INCLUDED_PLC_APP_IN_H
#define INCLUDED_PLC_APP_IN_H

#include <plc/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace plc {

    /*!
     * \brief <+description of block+>
     * \ingroup plc
     *
     */
    class PLC_API app_in : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<app_in> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of plc::app_in.
       *
       * To avoid accidental use of raw pointers, plc::app_in's
       * constructor is in a private implementation
       * class. plc::app_in::make is the public interface for
       * creating new instances.
       */
      static sptr make(int debug_level);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_APP_IN_H */
