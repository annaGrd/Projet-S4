function[coef]=interp_poly5(vi,vf,ti,tf)
A=[0 0 2 6*ti 12*ti^2 20*ti^3; 0 0 2 6*tf 12*tf^2 20*tf^3;...
   0 1 2*ti 3*ti^2 4*ti^3 5*ti^4; 0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;...
   1 ti ti^2 ti^3 ti^4 ti^5; 1 tf tf^2 tf^3 tf^4 tf^5];

Y=[0;0;0;0;vi;vf];

coef=(A\Y)';