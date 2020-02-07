void jacobian_h(double *jac,
const double de, const double delta_y, const double h1, const double h12, const double h2, const double h3, const double h34, const double h4, const double sgn_e, const double sgn_n, const double terr_dis, const double xi) {

/* w.r.t.:
    r_n
    r_e
    r_d
    xi
*/

const double t2 = 1.0/terr_dis;
const double t3 = de-1.0;
const double t4 = cos(xi);
const double t5 = sin(xi);
jac[0] = de*(h3*t2-h4*t2)-t3*(h1*t2-h2*t2);
jac[1] = t2*(h12-h34);
jac[2] = -1.0;
jac[3] = de*(delta_y*h3*sgn_n*t2*t4-delta_y*h4*sgn_n*t2*t4)-t3*(delta_y*h1*sgn_n*t2*t4-delta_y*h2*sgn_n*t2*t4)-delta_y*h12*sgn_e*t2*t5+delta_y*h34*sgn_e*t2*t5;
}
