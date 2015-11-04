/* -*- c++ -*- */

#define PLC_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "plc_swig_doc.i"

%{
#include "plc/phy_tx.h"
#include "plc/phy_rx.h"
#include "plc/mac_receiver.h"
#include "plc/mac_transmitter.h"
#include "plc/app_out.h"
#include "plc/app_in.h"
%}

%include "plc/phy_tx.h"
GR_SWIG_BLOCK_MAGIC2(plc, phy_tx);

%include "plc/phy_rx.h"
GR_SWIG_BLOCK_MAGIC2(plc, phy_rx);

%include "plc/mac_receiver.h"
GR_SWIG_BLOCK_MAGIC2(plc, mac_receiver);

%include "plc/mac_transmitter.h"
GR_SWIG_BLOCK_MAGIC2(plc, mac_transmitter);

%include "plc/app_out.h"
GR_SWIG_BLOCK_MAGIC2(plc, app_out);

%include "plc/app_in.h"
GR_SWIG_BLOCK_MAGIC2(plc, app_in);

