m=(0:15)';
fprintf ('complex(%1.15f,%1.15f),\n',[cos(m*2*pi/numel(m)) sin(m*2*pi/numel(m))]');