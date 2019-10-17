const double t2 = 1.0/delta_aoa; 
const double t3 = log_sqrt_w_over_sig1_aoa*sig_aoa_m*t2; 
jac[0] = t3-log_sqrt_w_over_sig1_aoa*sig_aoa_p*t2; 
jac[1] = -t3+log_sqrt_w_over_sig1_aoa*sig_aoa_p*t2; 
