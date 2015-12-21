#!/usr/bin/env python

import numpy
from numpy import pi
import math
import cmath
import random
from gnuradio import gr

class impulse_noise(gr.sync_block):
    MIN_IAT = 1/float(20)
    MAX_IAT = 1/float(200e3)
    MAX_FREQ = 19e6
    #PAYLOAD_GAIN = 10^(2.2/20)/(4096**0.5) * 4096/2 * 0.3  # payload carrier gain in frequency domain 
    SAMP_RATE = 0
    dtype = numpy.dtype('complex64')
    impulses = []
#(10**(2.2/10)/2*0.3**2)/10
    def __init__(self, samp_rate):
        gr.sync_block.__init__(self,
            name="impulse_noise",
            in_sig=None,
            out_sig=[self.dtype])
        self.SAMP_RATE = samp_rate

        #for i in range(n_noises):
#            self.impulses.append(self.gen_impulse())
        #self.MIN_SNR_DB = 10
        #self.MIN_SNR = 10**(MIN_SNR_DB/float(10))
        #PAYLOAD_GAIN = 10^(2.2/20)/(4096**0.5) * 4096/2 * 0.3  # payload carrier gain in frequency domain 
        #MAX_GAIN = max_gain
    def add_noise(self, iat, A, l, f, offset):
        print "I" + str(self.impulses.size) + ": iat=" + str(iat) + " A=" + str(A) + " l=" + str(l) + " f=" + str(f) + " offset=" + str(offset)
        self.impulses.append(self.create_impulse(iat, A, l, f, offset))

    def work(self, input_items, output_items):
        out = output_items[0]
        M = out.size
        out[:] = 0
        for impulse in self.impulses:
            s = impulse['signal']
            i = impulse['pos']
            N = s.size

            # copy remainder of previous iteration
            k = min(N-i, M)
            out[0:k] += s[i:i+k]
            j = k

            # keep filling with noise till no more room for full noise signal
            while j+N<=M:
                out[j:j+N] += s[:]
                j+=N

            # fill the remaining space
            out[j:M] += s[0:M-j]

            # update noise position
            impulse['pos'] = (i + M) % N

        return M

    def gen_impulse(self):
        # Randomize noise parameters        
        iat = random.uniform(self.IAT_RANGE[0], self.IAT_RANGE[1]) # inter arrival time 
        f = random.uniform(self.FREQ_RANGE[0], self.FREQ_RANGE[1]) # sine frequency 
        l = random.uniform(math.log(1/0.1)/iat, math.log(1/0.01)/iat) # lambda - attenuation to 1%-0.1% of amplitude within one IAT
        t = numpy.arange(0, self.SAMP_RATE * iat, dtype=self.dtype) / float(self.SAMP_RATE) 
        
        W = self.PSD_WINDOW_LENGTH
        N = int(iat * self.SAMP_RATE)
        m = W / N
        M = W - m * N
        a = cmath.exp(-l / self.SAMP_RATE + 1j * 2 * pi * f / self.SAMP_RATE)
  
        k = numpy.arange(0,W-1)
        exp0 = numpy.exp(-1j*k*2*pi/W)
        fft = 0

        if m:
            expN = numpy.exp(-1j*k*2*pi*N/W)
            fft += m/2*(numpy.abs((1-a**N*expN)/(1-a*exp0) + (1-a.conjugate()**N*expN)/(1-a.conjugate()*exp0)))

        if M:
            expM = numpy.exp(-1j*k*2*pi*M/W)
            fft += 0.5*(numpy.abs((1-a**M*expM)/(1-a*exp0) + (1-a.conjugate()**M*expM)/(1-a.conjugate()*exp0)))

        max_fft = numpy.max(fft)

        #A = random.uniform(0, sqrt(self.MAX_PSD/max_psd)) # sine smplitude based on minimum SNR requirement
        required_max_fft = (self.MAX_PSD*W/2)**0.5
        A =  required_max_fft / max_fft
        o = random.random() # offset (% of iat)

        print "iat=" + str(iat)

        # Generate noise signal
        impulse = {}
        offset = math.floor(o * N)
        t_shifted = numpy.concatenate((t[offset:], t[0:offset]))
        impulse['signal'] = A * numpy.sin(2*pi*f*t_shifted) * numpy.exp(-l*t_shifted);
        impulse['pos'] = 0
        return impulse

    def create_impulse(self, iat, A, l, f, offset):
        N = int(iat * self.SAMP_RATE)
        samples_offset = math.floor(offset * N)
        t = numpy.arange(0, self.SAMP_RATE * iat, dtype=self.dtype) / float(self.SAMP_RATE) 
        t_shifted = numpy.concatenate((t[-samples_offset:], t[:-samples_offset]))
        impulse = {}
        impulse['signal'] = A * numpy.sin(2*pi*f*t_shifted) * numpy.exp(-l*t_shifted);
        impulse['pos'] = 0
        return impulse



