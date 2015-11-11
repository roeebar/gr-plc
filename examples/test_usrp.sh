#!/bin/sh
/usr/local/lib/uhd/examples/benchmark_rate --duration 30 --tx_rate 25000000 --args "addr=192.168.10.3"
/usr/local/lib/uhd/examples/benchmark_rate --duration 30 --rx_rate 25000000 --args "addr=192.168.10.2"

