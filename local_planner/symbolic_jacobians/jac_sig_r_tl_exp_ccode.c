void jac_sig_r_tl_exp(double *jac, 
const double d_occ, const double delta_r, const double gamma, const double k_r, const double log_sqrt_w_over_sig1_r, const double p1_e, const double p1_h, const double p1_n, const double p2_h, const double p3_h, const double r_d, const double r_e, const double r_n, const double r_offset_1, const double sig_r, const double terr_dis, const double v, const double vG, const double vG_d, const double vG_e, const double vG_n, const double xi) {
 
/* w.r.t.: 
* r_n 
* r_e 
* r_d 
* v 
* gamma 
* xi 
*/
 
const double t2 = p1_h-p2_h; 
const double t3 = 1.0/delta_r; 
const double t4 = p2_h-p3_h; 
const double t5 = terr_dis*terr_dis; 
const double t6 = t4*terr_dis*vG_e; 
const double t7 = t2*terr_dis*vG_n; 
const double t10 = t5*vG_d; 
const double t8 = t6+t7-t10; 
const double t9 = 1.0/t8; 
const double t11 = cos(gamma); 
const double t12 = sin(gamma); 
const double t13 = cos(xi); 
const double t14 = sin(xi); 
const double t15 = t11*t13*vG_n*2.0; 
const double t16 = t11*t14*vG_e*2.0; 
const double t18 = t12*vG_d*2.0; 
const double t17 = t15+t16-t18; 
const double t19 = 1.0/vG; 
const double t20 = t11*v*vG_d*2.0; 
const double t21 = t12*t13*v*vG_n*2.0; 
const double t22 = t12*t14*v*vG_e*2.0; 
const double t23 = t20+t21+t22; 
const double t24 = p1_h+r_d; 
const double t25 = t5*t24; 
const double t26 = p1_h*t5; 
const double t27 = p1_e-r_e; 
const double t28 = t4*t27*terr_dis; 
const double t29 = p1_n-r_n; 
const double t30 = t2*t29*terr_dis; 
const double t31 = t25+t26+t28+t30; 
const double t32 = 1.0/(delta_r*delta_r); 
const double t33 = d_occ-r_offset_1; 
const double t34 = t11*t13*v*vG_e*2.0; 
const double t36 = t11*t14*v*vG_n*2.0; 
const double t35 = t34-t36; 
jac[0] = log_sqrt_w_over_sig1_r*sig_r*t2*t3*t9*terr_dis*vG; 
jac[1] = log_sqrt_w_over_sig1_r*sig_r*t3*t4*t9*terr_dis*vG; 
jac[2] = -log_sqrt_w_over_sig1_r*sig_r*t3*t5*t9*vG; 
jac[3] = sig_r*(log_sqrt_w_over_sig1_r*t3*(k_r*t17+d_occ*t9*(t5*t12+t2*t11*t13*terr_dis+t4*t11*t14*terr_dis)-t9*t17*t19*t31*(1.0/2.0))+k_r*log_sqrt_w_over_sig1_r*t17*t32*t33); 
jac[4] = -sig_r*(log_sqrt_w_over_sig1_r*t3*(k_r*t23+d_occ*t9*(-t5*t11*v+t2*t12*t13*terr_dis*v+t4*t12*t14*terr_dis*v)-t9*t19*t23*t31*(1.0/2.0))+k_r*log_sqrt_w_over_sig1_r*t23*t32*t33); 
jac[5] = -sig_r*(log_sqrt_w_over_sig1_r*t3*(-k_r*t35+d_occ*t9*(t2*t11*t14*terr_dis*v-t4*t11*t13*terr_dis*v)+t9*t19*t31*t35*(1.0/2.0))-k_r*log_sqrt_w_over_sig1_r*t32*t33*t35); 
