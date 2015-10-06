/* -*- c++ -*- */

#define PLC_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "plc_swig_doc.i"

%{
#include "plc/plcp_transmit.h"
#include "plc/plcp_receiver.h"
#include "plc/mac_receiver.h"
#include "plc/mac_transmitter.h"
#include "plc/phy.h"
#include "plc/app_out.h"
#include "plc/app_in.h"
%}

%include "plc/plcp_transmit.h"
GR_SWIG_BLOCK_MAGIC2(plc, plcp_transmit);

%include "plc/plcp_receiver.h"
GR_SWIG_BLOCK_MAGIC2(plc, plcp_receiver);

%include "plc/mac_receiver.h"
GR_SWIG_BLOCK_MAGIC2(plc, mac_receiver);

%include "plc/mac_transmitter.h"
GR_SWIG_BLOCK_MAGIC2(plc, mac_transmitter);
%include "plc/phy.h"
GR_SWIG_BLOCK_MAGIC2(plc, phy);
%include "plc/app_out.h"
GR_SWIG_BLOCK_MAGIC2(plc, app_out);
%include "plc/app_in.h"
GR_SWIG_BLOCK_MAGIC2(plc, app_in);
