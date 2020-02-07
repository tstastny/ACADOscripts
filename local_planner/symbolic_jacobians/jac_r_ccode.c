void jacobian_r(double *jac, 
const double n_occ_e, const double n_occ_h, const double n_occ_n, const double v_ray_e, const double v_ray_h, const double v_ray_n) {
 
/* w.r.t.: 
    r_n 
    r_e 
    r_d 
    v 
    gamma 
    xi 
*/
 
const double t2 = n_occ_e*v_ray_e; 
const double t3 = n_occ_h*v_ray_h; 
const double t4 = n_occ_n*v_ray_n; 
const double t5 = t2+t3+t4; 
const double t6 = 1.0/t5; 
jac[0] = -n_occ_n*t6; 
jac[1] = -n_occ_e*t6; 
jac[2] = n_occ_h*t6; 
}
