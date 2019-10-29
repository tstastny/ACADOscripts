void jac_sig_r_br_exp(double *jac, 
const double delta_r, const double gamma, const double k_delta_r, const double k_r_offset, const double log_sqrt_w_over_sig1_r, const double p1_h, const double p2_h, const double p3_h, const double r_occ, const double r_offset, const double sig_r, const double terr_dis, const double v, const double v_ray_e, const double v_ray_h, const double v_ray_n, const double v_rel, const double v_rel_sq, const double xi) {
 
/* w.r.t.: 
* r_n 
* r_e 
* r_d 
* v 
* gamma 
* xi 
*/
 
const double t2 = p2_h-p3_h; 
const double t3 = 1.0/delta_r; 
const double t4 = p1_h-p2_h; 
const double t5 = terr_dis*terr_dis; 
const double t6 = t5*v_ray_h; 
const double t7 = t4*terr_dis*v_ray_e; 
const double t8 = t2*terr_dis*v_ray_n; 
const double t9 = t6+t7+t8; 
const double t10 = 1.0/t9; 
const double t11 = cos(gamma); 
const double t12 = sin(gamma); 
const double t13 = t12*v_ray_h; 
const double t14 = cos(xi); 
const double t15 = t11*t14*v_ray_n; 
const double t16 = sin(xi); 
const double t17 = t11*t16*v_ray_e; 
const double t18 = t13+t15+t17; 
const double t19 = 1.0/(delta_r*delta_r); 
const double t20 = t12*t14*v*v_ray_n; 
const double t21 = t12*t16*v*v_ray_e; 
const double t22 = t20+t21-t11*v*v_ray_h; 
const double t23 = k_r_offset*v_rel_sq; 
const double t24 = -r_occ+r_offset+t23; 
const double t25 = t11*t14*v*v_ray_e; 
jac[0] = log_sqrt_w_over_sig1_r*sig_r*t2*t3*t10*terr_dis; 
jac[1] = log_sqrt_w_over_sig1_r*sig_r*t3*t4*t10*terr_dis; 
jac[2] = -log_sqrt_w_over_sig1_r*sig_r*t3*t5*t10; 
jac[3] = sig_r*(k_r_offset*log_sqrt_w_over_sig1_r*t3*t18*v_rel*2.0-k_delta_r*log_sqrt_w_over_sig1_r*t18*t19*t24*v_rel*2.0); 
jac[4] = -sig_r*(k_r_offset*log_sqrt_w_over_sig1_r*t3*t22*v_rel*2.0-k_delta_r*log_sqrt_w_over_sig1_r*t19*t22*t24*v_rel*2.0); 
jac[5] = sig_r*(k_r_offset*log_sqrt_w_over_sig1_r*t3*v_rel*(t25-t11*t16*v*v_ray_n)*2.0-k_delta_r*log_sqrt_w_over_sig1_r*t19*t24*v_rel*(t25-t11*t16*v*v_ray_n)*2.0); 
