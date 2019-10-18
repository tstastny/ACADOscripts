const double t2 = -aoa+aoa_m+delta_aoa; 
const double t3 = 1.0/(delta_aoa*delta_aoa*delta_aoa); 
const double t4 = t2*t2; 
const double t5 = t3*t4*3.0; 
jac[0] = t5; 
jac[1] = -t5; 
