/* -*- c++ -*- */

#ifndef INCLUDED_PLC_APP_OUT_H
#define INCLUDED_PLC_APP_OUT_H

#include <plc/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace plc {

    /*!
     * \brief <+description of block+>
     * \ingroup plc
     *
     */
    class PLC_API app_out : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<app_out> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of plc::app_out.
       *
       * To avoid accidental use of raw pointers, plc::app_out's
       * constructor is in a private implementation
       * class. plc::app_out::make is the public interface for
       * creating new instances.
       */
      static sptr make(std::vector<uint8_t> dest, bool debug);
    };

  } // namespace plc
} // namespace gr

#endif /* INCLUDED_PLC_APP_OUT_H */

