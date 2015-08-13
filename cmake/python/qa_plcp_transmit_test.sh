#!/bin/sh
export VOLK_GENERIC=1
export GR_DONT_LOAD_PREFS=1
export srcdir=/home/roee/Dropbox/Thesis/gr-plc/gr-plc/python
export PATH=/home/roee/Dropbox/Thesis/gr-plc/gr-plc/cmake/python:$PATH
export LD_LIBRARY_PATH=/home/roee/Dropbox/Thesis/gr-plc/gr-plc/cmake/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/home/roee/Dropbox/Thesis/gr-plc/gr-plc/cmake/swig:$PYTHONPATH
/usr/bin/python2 /home/roee/Dropbox/Thesis/gr-plc/gr-plc/python/qa_plcp_transmit.py 
