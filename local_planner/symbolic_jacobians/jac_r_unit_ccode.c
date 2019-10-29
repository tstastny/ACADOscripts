void jac_r_unit(double *jac, 
const double delta_r, const double gamma, const double k_delta_r, const double k_r_offset, const double n_occ_e, const double n_occ_h, const double n_occ_n, const double r_unit, const double v, const double v_ray_e, const double v_ray_h, const double v_ray_n, const double v_rel, const double xi) {
 
/* w.r.t.: 
* r_n 
* r_e 
* r_d 
* v 
* gamma 
* xi 
*/
 
const double t2 = 1.0/delta_r; 
const double t3 = n_occ_e*v_ray_e; 
const double t4 = n_occ_h*v_ray_h; 
const double t5 = n_occ_n*v_ray_n; 
const double t6 = t3+t4+t5; 
const double t7 = 1.0/t6; 
const double t8 = cos(gamma); 
const double t9 = k_delta_r*r_unit; 
const double t10 = k_r_offset+t9; 
const double t11 = sin(gamma); 
const double t12 = cos(xi); 
const double t13 = sin(xi); 
jac[0] = -n_occ_n*t2*t7; 
jac[1] = -n_occ_e*t2*t7; 
jac[2] = n_occ_h*t2*t7; 
jac[3] = t2*t10*v_rel*(t11*v_ray_h+t8*t13*v_ray_e+t8*t12*v_ray_n)*-2.0; 
jac[4] = t2*t10*v*v_rel*(-t8*v_ray_h+t11*t13*v_ray_e+t11*t12*v_ray_n)*2.0; 
jac[5] = t2*t8*t10*v*v_rel*(t12*v_ray_e-t13*v_ray_n)*-2.0; 
