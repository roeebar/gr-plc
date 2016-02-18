w_rise=[0.2/70*[0:69] 0.2+0.6/355*[0:355] 0.8+0.2/70*[1:70]];
w_fall=1-w_rise;
w_rise_down = downsample(w_rise,2);
w_fall_down = downsample(w_fall,2);