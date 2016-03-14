/* -*- c++ -*- */

#define PLC_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "plc_swig_doc.i"

%{
#include "plc/phy_tx.h"
#include "plc/phy_rx.h"
#include "plc/app_out.h"
#include "plc/app_in.h"
#include "plc/impulse_source.h"
%}

%include "plc/phy_tx.h"
GR_SWIG_BLOCK_MAGIC2(plc, phy_tx);

%include "plc/phy_rx.h"
GR_SWIG_BLOCK_MAGIC2(plc, phy_rx);


%include "plc/app_out.h"
GR_SWIG_BLOCK_MAGIC2(plc, app_out);

%include "plc/app_in.h"
GR_SWIG_BLOCK_MAGIC2(plc, app_in);

%include "plc/impulse_source.h"
GR_SWIG_BLOCK_MAGIC2(plc, impulse_source);
