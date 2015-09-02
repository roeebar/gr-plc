/* -*- c++ -*- */

#define PLC_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "plc_swig_doc.i"

%{
#include "plc/plcp_transmit.h"
#include "plc/mac.h"
#include "plc/plcp_receiver.h"
#include "plc/mac_receiver.h"
#include "plc/mac_transmitter.h"
%}

%include "plc/plcp_transmit.h"
GR_SWIG_BLOCK_MAGIC2(plc, plcp_transmit);
%include "plc/mac.h"
GR_SWIG_BLOCK_MAGIC2(plc, mac);
%include "plc/plcp_receiver.h"
GR_SWIG_BLOCK_MAGIC2(plc, plcp_receiver);

%include "plc/mac_receiver.h"
GR_SWIG_BLOCK_MAGIC2(plc, mac_receiver);

%include "plc/mac_transmitter.h"
GR_SWIG_BLOCK_MAGIC2(plc, mac_transmitter);
