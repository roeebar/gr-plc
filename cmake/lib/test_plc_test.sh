#!/bin/sh
export VOLK_GENERIC=1
export GR_DONT_LOAD_PREFS=1
export srcdir=/home/roee/Dropbox/Thesis/gr-plc/gr-plc/lib
export PATH=/home/roee/Dropbox/Thesis/gr-plc/gr-plc/cmake/lib:$PATH
export LD_LIBRARY_PATH=/home/roee/Dropbox/Thesis/gr-plc/gr-plc/cmake/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$PYTHONPATH
test-plc 
