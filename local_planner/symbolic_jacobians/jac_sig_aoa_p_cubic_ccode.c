const double t2 = aoa-aoa_p+delta_aoa; 
const double t3 = 1.0/(delta_aoa*delta_aoa*delta_aoa); 
const double t4 = t2*t2; 
jac[0] = t3*t4*-3.0; 
jac[1] = t3*t4*3.0; 
