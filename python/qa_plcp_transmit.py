#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# Copyright 2015 <+YOU OR YOUR COMPANY+>.
# 
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

from gnuradio import gr, gr_unittest
from gnuradio import blocks
import plc_swig as plc
import os
import struct
import time

class qa_plcp_transmit (gr_unittest.TestCase):

    def setUp (self):
        self.tb = gr.top_block ()

    def tearDown (self):
        self.tb = None

    def test_001_t (self):
        dirpath = os.environ["srcdir"]

        # Getting input stream from a file
        filename = dirpath+"/input.bin"
        n = os.path.getsize(filename) / 4
        f = open(filename)
        values = struct.unpack('I'*n,f.read(4*n))
        print "Read " + str(len(values)) + " input values from file"

        pmtvector = gr.pmt.make_u8vector(len(values), 0x00)
        for i in xrange(len(values)) :
            gr.pmt.u8vector_set(pmtvector, i, values[i])

        # Getting expected output stream from a file
        filename = dirpath+"/output.bin"
        n = os.path.getsize(filename) / 4
        f = open(filename)
        expected_result = struct.unpack('f'*n,f.read(4*n))
        print "Read " + str(len(expected_result)) + " expected output values from file"

        # Connecting blocks
        dic = gr.pmt.make_dict()
        mac = plc.mac(1000, 2);
        plcp = plc.plcp_transmit(True)
        dst = blocks.vector_sink_f()
        self.tb.msg_connect(mac, "mac out", plcp, "in")
        self.tb.connect(plcp, dst)

        # Running
        self.tb.start();
        time.sleep(1.5)
        self.tb.stop();

        # check data
        result_data = dst.data()
        self.assertFloatTuplesAlmostEqual(expected_result[0:n], result_data[0:n], n)


if __name__ == '__main__':
    gr_unittest.run(qa_plcp_transmit, "qa_plcp_transmit.xml")
