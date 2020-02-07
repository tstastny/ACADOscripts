void jacobian_Delta_r(double *jac, 
const double gamma, const double k_delta_r, const double v, const double v_ray_e, const double v_ray_h, const double v_ray_n, const double v_rel, const double xi) {
 
/* w.r.t.: 
    r_n 
    r_e 
    r_d 
    v 
    gamma 
    xi 
*/
 
const double t2 = cos(gamma); 
const double t3 = sin(gamma); 
const double t4 = cos(xi); 
const double t5 = sin(xi); 
jac[3] = k_delta_r*v_rel*(t3*v_ray_h+t2*t5*v_ray_e+t2*t4*v_ray_n)*2.0; 
jac[4] = k_delta_r*v*v_rel*(-t2*v_ray_h+t3*t5*v_ray_e+t3*t4*v_ray_n)*-2.0; 
jac[5] = k_delta_r*t2*v*v_rel*(t4*v_ray_e-t5*v_ray_n)*2.0; 
}
