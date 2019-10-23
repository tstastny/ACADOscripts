#include "acado_common.h"
#include <math.h>
#include <string.h>

#define EPSILON 0.000001
#define GRAVITY 9.81;

/* lookup table constants */
#define LEN_IDX_N 57
#define LEN_IDX_E 57
#define LEN_IDX_N_1 56
#define LEN_IDX_E_1 56

/* math functions / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/
int constrain_int(int x, int xmin, int xmax) {
    return (x < xmin) ? xmin : ((x > xmax) ? xmax : x);
}

double constrain_double(double x, double xmin, double xmax) {
    return (x < xmin) ? xmin : ((x > xmax) ? xmax : x);
}

void cross(double *v, const double v1[3], const double v2[3]) {
    v[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

double dot(const double v1[3], const double v2[3]) {
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}
/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / */

/* objective helper functions / / / / / / / / / / / / / / / / / / / / / /*/
void lookup_terrain_idx(const double pos_n, const double pos_e, const double pos_n_origin,
        const double pos_e_origin, const double terr_dis, int *idx_q, double *dn, double *de)
{
    /* relative position / indices */
    const double rel_n = pos_n - pos_n_origin;
    const double rel_n_bar = rel_n / terr_dis;
    int idx_n = (int)(floor(rel_n_bar));
    const double rel_e = pos_e - pos_e_origin;
    const double rel_e_bar = rel_e / terr_dis;
    int idx_e = (int)(floor(rel_e_bar));
    
    /* interpolation weights */
    *dn = rel_n_bar-idx_n;
    *de = rel_e_bar-idx_e;
    
    /* cap ends */
    if (idx_n < 0) {
        idx_n = 0;
    }
    else if (idx_n > LEN_IDX_N_1) {
        idx_n = LEN_IDX_N_1;
    }
    if (idx_e < 0) {
        idx_e = 0;
    }
    else if (idx_e > LEN_IDX_E_1) {
        idx_e = LEN_IDX_E_1;
    }
    
    /* neighbors (north) */
    int q_n[4];
    if (idx_n >= LEN_IDX_N_1) {
        q_n[0] = LEN_IDX_N_1;
        q_n[1] = LEN_IDX_N_1;
        q_n[2] = LEN_IDX_N_1;
        q_n[3] = LEN_IDX_N_1;
    }
    else {
        q_n[0] = idx_n;
        q_n[1] = idx_n + 1;
        q_n[2] = idx_n;
        q_n[3] = idx_n + 1;
    }
    /* neighbors (east) */
    int q_e[4];
    if (idx_e >= LEN_IDX_E_1) {
        q_e[0] = LEN_IDX_E_1;
        q_e[1] = LEN_IDX_E_1;
        q_e[2] = LEN_IDX_E_1;
        q_e[3] = LEN_IDX_E_1;
    }
    else {
        q_e[0] = idx_e;
        q_e[1] = idx_e;
        q_e[2] = idx_e + 1;
        q_e[3] = idx_e + 1;
    }
    
    /* neighbors row-major indices */
    idx_q[0] = q_n[0]*LEN_IDX_E + q_e[0];
    idx_q[1] = q_n[1]*LEN_IDX_E + q_e[1];
    idx_q[2] = q_n[2]*LEN_IDX_E + q_e[2];
    idx_q[3] = q_n[3]*LEN_IDX_E + q_e[3];
}

/* unit ground speed jacobian */
void jac_vg_unit(double *jac_vn, double *jac_ve, double *jac_vd,
        const double gamma, const double one_over_vG_norm, const double v,
        const double vG_d, const double vG_d_unit, const double vG_e,
        const double vG_e_unit, const double vG_n, const double vG_n_unit,
        const double v_d, const double v_e, const double v_n, const double xi)
{
    
    /* v_n, w.r.t.:
     * v
     * gamma
     * xi
     *
     * v_e, w.r.t.:
     * v
     * gamma
     * xi
     *
     * v_d, w.r.t.:
     * v
     * gamma
     * xi
     */
    
    const double t2 = cos(gamma); 
    const double t3 = cos(xi); 
    const double t4 = one_over_vG_norm*one_over_vG_norm; 
    const double t5 = sin(gamma); 
    const double t6 = sin(xi); 
    const double t7 = t2*t3*vG_n*2.0; 
    const double t8 = t2*t6*vG_e*2.0; 
    const double t16 = t5*vG_d*2.0; 
    const double t9 = t7+t8-t16; 
    const double t10 = t2*v*vG_d*2.0; 
    const double t11 = t3*t5*v*vG_n*2.0; 
    const double t12 = t5*t6*v*vG_e*2.0; 
    const double t13 = t10+t11+t12; 
    const double t14 = vG_n*v_e*2.0; 
    const double t17 = vG_e*v_n*2.0; 
    const double t15 = t14-t17;
    
    jac_vn[0] = one_over_vG_norm*t2*t3-t4*t9*vG_n_unit*0.5; 
    jac_vn[1] = one_over_vG_norm*t3*v_d+t4*t13*vG_n_unit*0.5; 
    jac_vn[2] = -one_over_vG_norm*v_e+t4*t15*vG_n_unit*0.5;
    
    jac_ve[0] = one_over_vG_norm*t2*t6-t4*t9*vG_e_unit*0.5; 
    jac_ve[1] = one_over_vG_norm*t6*v_d+t4*t13*vG_e_unit*0.5; 
    jac_ve[2] = one_over_vG_norm*v_n+t4*t15*vG_e_unit*0.5;
    
    jac_vd[0] = -one_over_vG_norm*t5-t4*t9*vG_d_unit*0.5; 
    jac_vd[1] = -one_over_vG_norm*t2*v+t4*t13*vG_d_unit*0.5; 
    jac_vd[2] = t4*t15*vG_d_unit*0.5;
}

/* soft angle of attack jacobian (exponential) */
void jac_sig_aoa_exp(double *jac,
        const double delta_aoa, const double log_sqrt_w_over_sig1_aoa,
        const double sig_aoa_m, const double sig_aoa_p) {
    
    /* w.r.t.:
     * gamma
     * theta
     */
    
    const double t2 = 1.0/delta_aoa; 
    const double t3 = log_sqrt_w_over_sig1_aoa*sig_aoa_m*t2; 
    jac[0] = t3-log_sqrt_w_over_sig1_aoa*sig_aoa_p*t2; 
    jac[1] = -t3+log_sqrt_w_over_sig1_aoa*sig_aoa_p*t2; 
}

/* height terrain jacobian (linear) */
void jac_sig_h_lin(double *jac,
        const double de, const double delta_h, const double delta_y,
        const double  h1, const double h12, const double h2,
        const double h3, const double h34, const double h4,
        const double log_sqrt_w_over_sig1_h, const double sgn_e,
        const double sgn_n, const double terr_dis, const double xi) {

    /* w.r.t.:
     * r_n
     * r_e
     * r_d
     * xi
     */
    
    const double t2 = 1.0/terr_dis; 
    const double t3 = 1.0/delta_h; 
    const double t4 = de-1.0; 
    const double t5 = cos(xi); 
    const double t6 = sin(xi);
    const double t7 = delta_y*sgn_n*t2*t5;
    const double t8 = delta_y*sgn_e*t2*t6;
    jac[0] = -log_sqrt_w_over_sig1_h*t3*(de*(h3*t2-h4*t2)-t4*(h1*t2-h2*t2)); 
    jac[1] = -log_sqrt_w_over_sig1_h*t3*(h12*t2-h34*t2); 
    jac[2] = log_sqrt_w_over_sig1_h*t3; 
    jac[3] = -log_sqrt_w_over_sig1_h*t3*(t7*(de*(h3-h4)-t4*(h1-h2))+t8*(-h12+h34)); 
}

/* height terrain jacobian (exponential) */
void jac_sig_h_exp(double *jac,
        const double de, const double delta_h, const double delta_y,
        const double h1, const double h12, const double h2, const double h3,
        const double h34, const double h4, const double log_sqrt_w_over_sig1_h,
        const double sgn_e, const double sgn_n, const double sig_h,
        const double terr_dis, const double xi) {
    
    /* w.r.t.:
     * r_n
     * r_e
     * r_d
     * xi
     */

    const double t2 = 1.0/terr_dis; 
    const double t3 = 1.0/delta_h; 
    const double t4 = de-1.0; 
    const double t5 = cos(xi); 
    const double t6 = sin(xi); 
    jac[0] = -log_sqrt_w_over_sig1_h*sig_h*t3*(de*(h3*t2-h4*t2)-t4*(h1*t2-h2*t2)); 
    jac[1] = -log_sqrt_w_over_sig1_h*sig_h*t3*(h12*t2-h34*t2); 
    jac[2] = log_sqrt_w_over_sig1_h*sig_h*t3; 
    jac[3] = -log_sqrt_w_over_sig1_h*sig_h*t3*(de*(delta_y*h3*sgn_n*t2*t5-delta_y*h4*sgn_n*t2*t5)-t4*(delta_y*h1*sgn_n*t2*t5-delta_y*h2*sgn_n*t2*t5)-delta_y*h12*sgn_e*t2*t6+delta_y*h34*sgn_e*t2*t6);
}

/* radial terrain (bottom-right) jacobian (linear) */
void jac_sig_r_br_lin(double *jac, 
        const double d_occ, const double delta_r, const double gamma,
        const double k_r, const double log_sqrt_w_over_sig1_r,
        const double p1_e, const double p1_h, const double p1_n,
        const double p2_h, const double p3_h, const double r_d,
        const double r_e, const double r_n, const double r_offset_1,
        const double terr_dis, const double v, const double vG,
        const double vG_d, const double vG_e, const double vG_n, const double xi) {
 
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
    jac[0] = log_sqrt_w_over_sig1_r*t2*t3*t9*terr_dis*vG; 
    jac[1] = log_sqrt_w_over_sig1_r*t3*t4*t9*terr_dis*vG; 
    jac[2] = -log_sqrt_w_over_sig1_r*t3*t5*t9*vG; 
    jac[3] = log_sqrt_w_over_sig1_r*t3*(k_r*t17+d_occ*t9*(t5*t12+t2*t11*t13*terr_dis+t4*t11*t14*terr_dis)-t9*t17*t19*t31*(1.0/2.0))+k_r*log_sqrt_w_over_sig1_r*t17*t32*t33; 
    jac[4] = -log_sqrt_w_over_sig1_r*t3*(k_r*t23+d_occ*t9*(-t5*t11*v+t2*t12*t13*terr_dis*v+t4*t12*t14*terr_dis*v)-t9*t19*t23*t31*(1.0/2.0))-k_r*log_sqrt_w_over_sig1_r*t23*t32*t33; 
    jac[5] = -log_sqrt_w_over_sig1_r*t3*(-k_r*t35+d_occ*t9*(t2*t11*t14*terr_dis*v-t4*t11*t13*terr_dis*v)+t9*t19*t31*t35*(1.0/2.0))+k_r*log_sqrt_w_over_sig1_r*t32*t33*(t34-t36); 
}

/* radial terrain (bottom-right) jacobian (exponential) */
void jac_sig_r_br_exp(double *jac, 
        const double d_occ, const double delta_r, const double gamma,
        const double k_r, const double log_sqrt_w_over_sig1_r,
        const double p1_e, const double p1_h, const double p1_n,
        const double p2_h, const double p3_h, const double r_d,
        const double r_e, const double r_n, const double r_offset_1,
        const double sig_r, const double terr_dis, const double v,
        const double vG, const double vG_d, const double vG_e,
        const double vG_n, const double xi) {
    
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
}

/* radial terrain (top-left) jacobian (linear) */
void jac_sig_r_tl_lin(double *jac, 
        const double d_occ, const double delta_r, const double gamma,
        const double k_r, const double log_sqrt_w_over_sig1_r,
        const double p1_e, const double p1_h, const double p1_n,
        const double p2_h, const double p3_h, const double r_d,
        const double r_e, const double r_n, const double r_offset_1,
        const double terr_dis, const double v, const double vG,
        const double vG_d, const double vG_e, const double vG_n, const double xi) {

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
    jac[0] = log_sqrt_w_over_sig1_r*t2*t3*t9*terr_dis*vG; 
    jac[1] = log_sqrt_w_over_sig1_r*t3*t4*t9*terr_dis*vG; 
    jac[2] = -log_sqrt_w_over_sig1_r*t3*t5*t9*vG; 
    jac[3] = log_sqrt_w_over_sig1_r*t3*(k_r*t17+d_occ*t9*(t5*t12+t2*t11*t13*terr_dis+t4*t11*t14*terr_dis)-t9*t17*t19*t31*(1.0/2.0))+k_r*log_sqrt_w_over_sig1_r*t17*t32*t33; 
    jac[4] = -log_sqrt_w_over_sig1_r*t3*(k_r*t23+d_occ*t9*(-t5*t11*v+t2*t12*t13*terr_dis*v+t4*t12*t14*terr_dis*v)-t9*t19*t23*t31*(1.0/2.0))-k_r*log_sqrt_w_over_sig1_r*t23*t32*t33; 
    jac[5] = -log_sqrt_w_over_sig1_r*t3*(-k_r*t35+d_occ*t9*(t2*t11*t14*terr_dis*v-t4*t11*t13*terr_dis*v)+t9*t19*t31*t35*(1.0/2.0))+k_r*log_sqrt_w_over_sig1_r*t32*t33*(t34-t36); 
}

/* radial terrain (top-left) jacobian (exponential) */
void jac_sig_r_tl_exp(double *jac, 
        const double d_occ, const double delta_r, const double gamma,
        const double k_r, const double log_sqrt_w_over_sig1_r,
        const double p1_e, const double p1_h, const double p1_n,
        const double p2_h, const double p3_h, const double r_d,
        const double r_e, const double r_n, const double r_offset_1,
        const double sig_r, const double terr_dis, const double v,
        const double vG, const double vG_d, const double vG_e,
        const double vG_n, const double xi) {

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
}

/* check ray-triangle intersection */
int intersect_triangle(double *d_occ, double *p_occ,
        const double r0[3], const double v_ray[3],
        const double p1[3], const double p2[3], const double p3[3]) {
    /* following: "Fast, Minimum Storage Ray/Triangle Intersection",
     * Moeller et. al., Journal of Graphics Tools, Vol.2(1), 1997
     */

    /* NOTE: all vectors in here are E,N,U */
    
    /* find vectors for two edges sharing p1 */
    const double e1[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
    const double e2[3] = {p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]};

    /* begin calculating determinant - also used to calculate U parameter */
    double pvec[3];
    cross(pvec, v_ray, e2);

    /* we don't test for backwards culling here as, assuming this ray casting
     * algorithm is properly detecting the first occluding triangles, we should
     * not run into a case of a true "backwards" facing triangle (also the
     * current BR and TL definitions have opposite vertex spin definitions..
     * should probably change that in the future.. could maybe avoid a few
     * divisions of the determinant) */

    /* if the determinant is near zero, ray lies in the triangle plane */
    const double det = dot(e1, pvec);
    if (det > -EPSILON && det < EPSILON) {
        return 0;
    }
    
    /* divide the determinant (XXX: could possibly find a way to avoid this until the last minute possible..) */
    const double inv_det = 1.0 / det;

    /* calculate distance from p1 to ray origin */
    const double tvec[3] = {r0[0] - p1[0], r0[1] - p1[1], r0[2] - p1[2]};

    /* calculate u parameter and test bounds */
    const double u = dot(tvec, pvec) * inv_det;
    if (u < 0.0 || u > 1.0) {
        return 0;
    }
    
    /* prepare to test v parameter */
    double qvec[3];
    cross(qvec, tvec, e1);

    /* calculate v parameter and test bounds */
    const double v = dot(v_ray, qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0) {
        return 0;
    }

    /* calculate d_occ, scale parameters, ray intersects triangle */
    *d_occ = dot(e2, qvec) * inv_det;

    /* calculate and return intersection point */
    const double one_u_v = (1 - u - v);
    p_occ[0] = one_u_v * p1[0] + u * p2[0] + v * p3[0];
    p_occ[1] = one_u_v * p1[1] + u * p2[1] + v * p3[1];
    p_occ[2] = one_u_v * p1[2] + u * p2[2] + v * p3[2];
    
    return 1;
}

/* cast ray through terrain map and determine the intersection point on any occluding trianglular surface */
int castray(double *d_occ, double *p_occ, double *p1, double *p2, double *p3,
        const double r0[3], const double r1[3], const double v[3],
        const double pos_n_origin, const double pos_e_origin, const double terr_dis, const double *terr_map) {
    
    /* INPUTS:
     *
     * (double) r0[3]             	start position (e,n,u) [m]
     * (double) r1[3]             	end position (e,n,u) [m]
     * (double) v[3]              	ray unit vector (e,n,u)
     * (double) pos_n_origin        northing origin of terrain map
     * (double) pos_e_origin        easting origin of terrain map
     * (double) terr_dis          	terrain discretization
     * (double) terr_map          	terrain map
     *
     * OUTPUTS:
     *
     * (int)   occ_detected         0=no detection, 1=BR triangle detected, 2=TL triangle detected
     * (double) d_occ           	distance to the ray-triangle intersection [m]
     * (double) p_occ[3]        	coord. of the ray-triangle intersection [m]
     * (double) p1[3]           	coord. of triangle vertex 1 (e,n,u) [m]
     * (double) p2[3]             	coord. of triangle vertex 2 (e,n,u) [m]
     * (double)	p3[3]           	coord. of triangle vertex 3 (e,n,u) [m]
     */ 

    /* initialize */
    int occ_detected = 0;

    /* relative (unit) start position */
    const double x0 = (r0[0] - pos_e_origin)/terr_dis;
    const double y0 = (r0[1] - pos_n_origin)/terr_dis;

    /* initial height */
    const double h0 = r0[2];
    
    /* vector for triangle intersect inputs */
    const double r0_rel[3] = {x0*terr_dis,y0*terr_dis,h0}; /*XXX: this origin subtracting/adding is inefficient.. pick one and go with it for this function

    /* relative end position */
    const double x1 = (r1[0] - pos_e_origin)/terr_dis;
    const double y1 = (r1[1] - pos_n_origin)/terr_dis;

    /* end height */
    const double h1 = r1[2];

    /* line deltas */
    const double dx = fabs(x1 - x0);
    const double dy = fabs(y1 - y0);

    /* initial cell origin */
    int x = (int)(floor(x0));
    int y = (int)(floor(y0));

    /* unit change in line length per x/y (see definition below) */
    double dt_dx;
    double dt_dy;

    /* change in height per unit line length (t) */
    const double dh = h1 - h0;

    /* number of cells we pass through */
    int n = fabs(floor(x1)-x) + fabs(floor(y1)-y) + 1; /*XXX: what is the real difference between this and using dx / dy? */

    /* initialize stepping criteria */
    double t_next_horizontal, t_last_horizontal;
    double t_next_vertical, t_last_vertical;
    double x_inc, y_inc;

    if (dx < 0.00001) {
        x_inc = 0.0;
        dt_dx = INFINITY;
        t_next_horizontal = INFINITY;
        t_last_horizontal = INFINITY;
    }
    else if (x1 > x0) {
        x_inc = 1.0;
        dt_dx = 1.0 / dx;
        t_next_horizontal = (x + 1.0 - x0) * dt_dx; /* remember x is "floor(x0)" here */
        t_last_horizontal = (x0 - x) * dt_dx;
    }
    else {
        x_inc = -1.0;
        dt_dx = 1.0 / dx;
        t_next_horizontal = (x0 - x) * dt_dx; /* remember x is "floor(x0)" here */
        t_last_horizontal = (x + 1.0 - x0) * dt_dx;
    }

    if (dy < 0.00001) {
        y_inc = 0.0;
        dt_dy = INFINITY;
        t_next_vertical = INFINITY;
        t_last_vertical = INFINITY;
    }
    else if (y1 > y0) {
        y_inc = 1.0;
        dt_dy = 1.0 / dy;
        t_next_vertical = (y + 1.0 - y0) * dt_dy; /* remember y is "floor(y0)" here */
        t_last_vertical = (y0 - y) * dt_dy;
    }
    else {
        y_inc = -1.0;
        dt_dy = 1.0 / dy;
        t_next_vertical = (y0 - y) * dt_dy; /* remember y is "floor(y0)" here */
        t_last_vertical = (y + 1.0 - y0) * dt_dy;
    }

    /* find cell intersection in opposite direction to initialize cell entrance
     * condition */
    bool last_step_was_vert = (t_last_vertical < t_last_horizontal);

    /* initialize entrance height */
    double h_entr = h0;

    /* for loop init */
    int ret;
    double t, h_exit, h_check;
    bool take_vert_step, check1, check2, check3, check4;
    
    /* check that start position is not already under the terrain */
    
    /* bound corner coordinates */
    int x_check = constrain_int(x, 0, LEN_IDX_E_1);
    int y_check = constrain_int(y, 0, LEN_IDX_N_1);
    int x_check1 = constrain_int(x_check+1, 0, LEN_IDX_E_1);
    int y_check1 = constrain_int(y_check+1, 0, LEN_IDX_N_1);
    /* convert to row-major indices */
    int idx_corner1 = y_check*LEN_IDX_E + x_check;
    int idx_corner2 = y_check1*LEN_IDX_E + x_check;
    int idx_corner3 = y_check1*LEN_IDX_E + x_check1;
    int idx_corner4 = y_check*LEN_IDX_E + x_check1;
    
    const double x0_unit = x0 - x;
    const double y0_unit = y0 - y;
    if (y0_unit > x0_unit) {
        /* check bottom-right triangle */
        if (x0_unit*(terr_map[idx_corner4]-terr_map[idx_corner1]) + y0_unit*(terr_map[idx_corner3] - terr_map[idx_corner4]) > h0) {
            return occ_detected;
        }
    }
    else {
        /* check top-left triangle */
        if (x0_unit*(terr_map[idx_corner3]-terr_map[idx_corner2]) + y0_unit*(terr_map[idx_corner2] - terr_map[idx_corner1]) > h0) {
            return occ_detected;
        }
    }

    /* cast the ray */
    int i;
    for (i = 0; i < n; i=i+1) {

        /* check the next step we will take and compute the exit height */
        if (t_next_vertical < t_next_horizontal) {
            /* next step is vertical */
            take_vert_step = true;
            t = t_next_vertical; /* current step */
            t_next_vertical = t_next_vertical + dt_dy;
        }
        else {
            /* next step is horizontal */
            take_vert_step = false;
            t = t_next_horizontal; /* current step */
            t_next_horizontal = t_next_horizontal + dt_dx;
        }

        /* take minimum of entrance and exit height for check */
        /* TODO: should be a way to get rid of this if statement by looking at dh outside for loop... */
        h_exit = h0 + dh * t;
        if (dh > 0.0) {
            h_check = h_entr;
        }
        else {
            h_check = h_exit;
        }
        h_entr = h_exit;
        
        /* bound corner coordinates */
        x_check = constrain_int(x, 0, LEN_IDX_E_1);
        y_check = constrain_int(y, 0, LEN_IDX_N_1);
        x_check1 = constrain_int(x_check+1, 0, LEN_IDX_E_1);
        y_check1 = constrain_int(y_check+1, 0, LEN_IDX_N_1);
        /* convert to row-major indices */
        idx_corner1 = y_check*LEN_IDX_E + x_check;
        idx_corner2 = y_check1*LEN_IDX_E + x_check;
        idx_corner3 = y_check1*LEN_IDX_E + x_check1;
        idx_corner4 = y_check*LEN_IDX_E + x_check1;
        /* check the four corners */
        check1 = terr_map[idx_corner1] > h_check; /* corner 1 (bottom left) */
        check2 = terr_map[idx_corner2] > h_check; /* corner 2 (top left) */
        check3 = terr_map[idx_corner3] > h_check; /* corner 3 (top right) */
        check4 = terr_map[idx_corner4] > h_check; /* corner 4 (bottom right) */

        /* check cell triangles */
        if (last_step_was_vert) { /* / / / / / / / / / / / / / / / / / / */
            /* vertical entrance step */

            if (take_vert_step) {
                /* next step is vertical */

                if (y_inc > 0) { /*TODO: should be able to get rid of a few of these ifs by making the decision outside the for loop... */
                    /* BR, TL */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*(x+1);
                        p2[1] = terr_dis*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret; /* =1 if detection */
                    }

                    /* check top-left triangle corners */
                    if ((check1 || check2 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*x;
                        p2[1] = terr_dis*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
                else {
                    /* TL, BR */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*x;
                        p2[1] = terr_dis*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret*2; /* =2 if detection */
                    }

                    /* check bottom-right triangle corners */
                    if ((check1 || check4 || check3) && occ_detected==0) {

                        /* set 3 corners */
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*(x+1);
                        p2[1] = terr_dis*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret; /* =1 if detection */
                    }
                }
            }   
            else  {/* - - - - - - - - - - - - - - - - - - - - - - - - - -*/
                /* next step is horizontal */

                if (y_inc > 0 && x_inc > 0) {
                    /* BR */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*(x+1);
                        p2[1] = terr_dis*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret; /* =1 if detection */
                    }
                }
                else if (y_inc < 0 && x_inc < 0) {
                    /* TL */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*x;
                        p2[1] = terr_dis*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
                else {
                    /* BR, TL */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*(x+1);
                        p2[1] = terr_dis*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret; /* =1 if detection */
                    }

                    /* check top-left triangle corners */
                    if ((check1 || check2 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*x;
                        p2[1] = terr_dis*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
            }
        }
        else { /* last step was horizontal / / / / / / / / / / / / / / / */
            if (take_vert_step) {
                /* next step is vertical */

                if (x_inc > 0) {
                    /* TL */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*x;
                        p2[1] = terr_dis*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret*2; /* =2 if detection */
                    }

                    if ((y_inc < 0) && (occ_detected==0)) {
                        /* BR */

                        /* check bottom-right triangle corners */
                        if (check1 || check4 || check3) {

                            /* set 3 corners */
                            p1[0] = terr_dis*x;
                            p1[1] = terr_dis*y;
                            p1[2] = terr_map[idx_corner1];
                            p2[0] = terr_dis*(x+1);
                            p2[1] = terr_dis*y;
                            p2[2] = terr_map[idx_corner4];
                            p3[0] = terr_dis*(x+1);
                            p3[1] = terr_dis*(y+1);
                            p3[2] = terr_map[idx_corner3];

                            /* check for ray-triangle intersection */
                            ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                            occ_detected += ret; /* =1 if detection */
                        }
                    }
                }
                else {
                    /* BR */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*(x+1);
                        p2[1] = terr_dis*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret; /* =1 if detection */
                    }

                    if ((y > 0) && (occ_detected==0)) {
                        /* TL */

                        /* check top-left triangle corners */
                        if (check1 || check2 || check3) {

                            /* set 3 corners */
                            p1[0] = terr_dis*x;
                            p1[1] = terr_dis*y;
                            p1[2] = terr_map[idx_corner1];
                            p2[0] = terr_dis*x;
                            p2[1] = terr_dis*(y+1);
                            p2[2] = terr_map[idx_corner2];
                            p3[0] = terr_dis*(x+1);
                            p3[1] = terr_dis*(y+1);
                            p3[2] = terr_map[idx_corner3];

                            /* check for ray-triangle intersection */
                            ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                            occ_detected += ret*2; /* =2 if detection */
                        }
                    }
                }
            }
            else { /* - - - - - - - - - - - - - - - - - - - - - - - - - -*/
                /* next step is horizontal */

                if (x_inc > 0) {
                    /* TL, BR */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*x;
                        p2[1] = terr_dis*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret*2; /* =2 if detection */
                    }

                    /* check bottom-right triangle corners */
                    if ((check1 || check4 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*(x+1);
                        p2[1] = terr_dis*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret; /* =1 if detection */
                    }
                }
                else {
                    /* BR, TL */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*(x+1);
                        p2[1] = terr_dis*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret; /* =1 if detection */
                    }

                    /* check top-left triangle corners */
                    if ((check1 || check2 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*x;
                        p2[1] = terr_dis*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
            }
        } /* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / */

        /* return if occlusion detected */
        if (occ_detected > 0) {
            return occ_detected;
        }

        /* actually take the step */
        if (take_vert_step) { /* (t_next_vertical < t_next_horizontal) */
            /* take a vertical step */
            y = y + y_inc;
        }
        else {
            /* take a horizontal step */
            x = x + x_inc;
        }
        last_step_was_vert = take_vert_step;
    }
    return occ_detected;
}

/* calculate speed states */
void calculate_speed_states(double *speed_states,
        const double v, const double gamma, const double xi,
        const double w_n, const double w_e, const double w_d)
{
    const double v_cos_gamma = v*cos(gamma);
    const double cos_xi = cos(xi);
    const double sin_xi = sin(xi);
    
    /* airspeed */
    speed_states[0] = v_cos_gamma*cos_xi;       /* v_n */
    speed_states[1] = v_cos_gamma*sin_xi;       /* v_e */
    speed_states[2] = -v*sin(gamma);            /* v_d */
    
    /* ground speed */
    speed_states[3] = speed_states[0] + w_n;    /* vG_n */
    speed_states[4] = speed_states[1] + w_e;    /* vG_e */
    speed_states[5] = speed_states[2] + w_d;    /* vG_d */
    speed_states[6] = speed_states[3]*speed_states[3] + speed_states[4]*speed_states[4] + speed_states[5]*speed_states[5]; /* vG_sq */
    speed_states[7] = sqrt(speed_states[6]);    /* vG_norm */
    
    /* unit ground speed */
    speed_states[8] = (speed_states[7] < 0.01) ? 100.0 : 1.0 / speed_states[7];
    speed_states[9] = speed_states[3] * speed_states[8];    /* vG_n_unit */
    speed_states[10] = speed_states[4] * speed_states[8];	/* vG_e_unit */
    speed_states[11] = speed_states[5] * speed_states[8];   /* vG_d_unit */
}
/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / */

/* evaluation functions / / / / / / / / / / / / / / / / / / / / / / / / /*/
void lsq_obj_eval( const real_t *in, real_t *out, bool eval_end_term )
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */
    
    /* shift inputs by ACADO_NU if evaluating the end term */
    int idx_shift = 0; 
    if (eval_end_term) idx_shift = ACADO_NU;
    
    /* states */
    const double r_n = in[0];
    const double r_e = in[1];
    const double r_d = in[2];
    const double v = in[3];
    const double gamma = in[4];
    const double xi = in[5];
    const double phi = in[6];
    const double theta = in[7];
    const double n_p = in[8];
    
    /* controls */ /* NOTE: these just dont get used if in end term eval */
    const double u_T = in[9];
    const double phi_ref = in[10];
    const double theta_ref = in[11];
    
    /* online data */
    
    /* environment */
    /*const double rho = in[12-idx_shift]; */
    
    /* disturbances */
    const double w_n = in[13-idx_shift];
    const double w_e = in[14-idx_shift];
    const double w_d = in[15-idx_shift];
    
    /* control augmented attitude time constants and gains */
    /*const double tau_phi = in[16-idx_shift];
    const double tau_theta = in[17-idx_shift];
    const double k_phi = in[18-idx_shift];
    const double k_theta = in[19-idx_shift];*/
    
    /* soft aoa */
    const double sig_aoa = in[20-idx_shift];
    double jac_sig_aoa[2];
    jac_sig_aoa[0] = in[21-idx_shift];
    jac_sig_aoa[1] = in[22-idx_shift];
    
    /* soft height */
    const double sig_h = in[23-idx_shift];
    double jac_sig_h[4];
    jac_sig_h[0] = in[24-idx_shift];
    jac_sig_h[1] = in[25-idx_shift];
    jac_sig_h[2] = in[26-idx_shift];
    jac_sig_h[3] = in[27-idx_shift];
    
    /* soft radial */
    const double sig_r = in[28-idx_shift];
    double jac_sig_r[6];
    jac_sig_r[0] = in[29-idx_shift];
    jac_sig_r[1] = in[30-idx_shift];
    jac_sig_r[2] = in[31-idx_shift];
    jac_sig_r[3] = in[32-idx_shift];
    jac_sig_r[4] = in[33-idx_shift];
    jac_sig_r[5] = in[34-idx_shift];
    
    
    /* INTERMEDIATE CALCULATIONS - - - - - - - - - - - - - - - - - - - - */

    /* speed states */
    double speed_states[12];
    calculate_speed_states(speed_states, v, gamma, xi, w_n, w_e, w_d);
    const double v_n = speed_states[0];
    const double v_e = speed_states[1];
    const double v_d = speed_states[2];
    const double vG_n = speed_states[3];
    const double vG_e = speed_states[4];
    const double vG_d = speed_states[5];
    const double one_over_vG_norm = speed_states[8];
    const double vG_n_unit = speed_states[9];
    const double vG_e_unit = speed_states[10];
    const double vG_d_unit = speed_states[11]; 
    
    
    /* OBJECTIVES - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
    
    /* state output */
    out[0] = vG_n_unit;
    out[1] = vG_e_unit;
    out[2] = vG_d_unit;
    out[3] = v;
    out[4] = phi;
    out[5] = theta;
    out[6] = sig_aoa;
    out[7] = sig_h;
    out[8] = sig_r;
    
    /* control output */
    if (!eval_end_term) {
        out[9] = u_T;
        out[10] = phi_ref;
        out[11] = theta_ref;
    }
    

    /* JACOBIANS - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

    if (eval_end_term) {
        
        /* lsq end term non-zero jacobian evals */
        double jac_v_n[3];
        double jac_v_e[3];
        double jac_v_d[3];
        jac_vg_unit(jac_v_n, jac_v_e, jac_v_d,
            gamma, one_over_vG_norm, v, vG_d, vG_d_unit, vG_e, vG_e_unit, vG_n, vG_n_unit, v_d, v_e, v_n, xi);
        double jac_v = 1.0;
        double jac_phi = 1.0;
        double jac_theta = 1.0;

        /* lsq end term jacobian w.r.t. states */
        out[9] = 0.0;
        out[10] = 0.0;
        out[11] = 0.0;
        out[12] = jac_v_n[0];
        out[13] = jac_v_n[1];
        out[14] = jac_v_n[2];
        out[15] = 0.0;
        out[16] = 0.0;
        out[17] = 0.0;
        out[18] = 0.0;
        out[19] = 0.0;
        out[20] = 0.0;
        out[21] = jac_v_e[0];
        out[22] = jac_v_e[1];
        out[23] = jac_v_e[2];
        out[24] = 0.0;
        out[25] = 0.0;
        out[26] = 0.0;
        out[27] = 0.0;
        out[28] = 0.0;
        out[29] = 0.0;
        out[30] = jac_v_d[0];
        out[31] = jac_v_d[1];
        out[32] = jac_v_d[2];
        out[33] = 0.0;
        out[34] = 0.0;
        out[35] = 0.0;
        out[36] = 0.0;
        out[37] = 0.0;
        out[38] = 0.0;
        out[39] = jac_v;
        out[40] = 0.0;
        out[41] = 0.0;
        out[42] = 0.0;
        out[43] = 0.0;
        out[44] = 0.0;
        out[45] = 0.0;
        out[46] = 0.0;
        out[47] = 0.0;
        out[48] = 0.0;
        out[49] = 0.0;
        out[50] = 0.0;
        out[51] = jac_phi;
        out[52] = 0.0;
        out[53] = 0.0;
        out[54] = 0.0;
        out[55] = 0.0;
        out[56] = 0.0;
        out[57] = 0.0;
        out[58] = 0.0;
        out[59] = 0.0;
        out[60] = 0.0;
        out[61] = jac_theta;
        out[62] = 0.0;
        out[63] = 0.0;
        out[64] = 0.0;
        out[65] = 0.0;
        out[66] = 0.0;
        out[67] = jac_sig_aoa[0];
        out[68] = 0.0;
        out[69] = 0.0;
        out[70] = jac_sig_aoa[1];
        out[71] = 0.0;
        out[72] = jac_sig_h[0];
        out[73] = jac_sig_h[1];
        out[74] = jac_sig_h[2];
        out[75] = 0.0;
        out[76] = 0.0;
        out[77] = jac_sig_h[3];
        out[78] = 0.0;
        out[79] = 0.0;
        out[80] = 0.0;
        out[81] = jac_sig_r[0];
        out[82] = jac_sig_r[1];
        out[83] = jac_sig_r[2];
        out[84] = jac_sig_r[3];
        out[85] = jac_sig_r[4];
        out[86] = jac_sig_r[5];
        out[87] = 0.0;
        out[88] = 0.0;
        out[89] = 0.0;
    }
    else {
        
        /* lsq non-zero jacobian evals */
        double jac_v_n[3];
        double jac_v_e[3];
        double jac_v_d[3];
        jac_vg_unit(jac_v_n, jac_v_e, jac_v_d,
            gamma, one_over_vG_norm, v, vG_d, vG_d_unit, vG_e, vG_e_unit, vG_n, vG_n_unit, v_d, v_e, v_n, xi);
        double jac_v = 1.0;
        double jac_phi = 1.0;
        double jac_theta = 1.0;
        double jac_uT = 1.0;
        double jac_phi_ref = 1.0;
        double jac_theta_ref = 1.0;
    
        /* lsq jacobian w.r.t. states */
        out[12] = 0.0;
        out[13] = 0.0;
        out[14] = 0.0;
        out[15] = jac_v_n[0];
        out[16] = jac_v_n[1];
        out[17] = jac_v_n[2];
        out[18] = 0.0;
        out[19] = 0.0;
        out[20] = 0.0;
        out[21] = 0.0;
        out[22] = 0.0;
        out[23] = 0.0;
        out[24] = jac_v_e[0];
        out[25] = jac_v_e[1];
        out[26] = jac_v_e[2];
        out[27] = 0.0;
        out[28] = 0.0;
        out[29] = 0.0;
        out[30] = 0.0;
        out[31] = 0.0;
        out[32] = 0.0;
        out[33] = jac_v_d[0];
        out[34] = jac_v_d[1];
        out[35] = jac_v_d[2];
        out[36] = 0.0;
        out[37] = 0.0;
        out[38] = 0.0;
        out[39] = 0.0;
        out[40] = 0.0;
        out[41] = 0.0;
        out[42] = jac_v;
        out[43] = 0.0;
        out[44] = 0.0;
        out[45] = 0.0;
        out[46] = 0.0;
        out[47] = 0.0;
        out[48] = 0.0;
        out[49] = 0.0;
        out[50] = 0.0;
        out[51] = 0.0;
        out[52] = 0.0;
        out[53] = 0.0;
        out[54] = jac_phi;
        out[55] = 0.0;
        out[56] = 0.0;
        out[57] = 0.0;
        out[58] = 0.0;
        out[59] = 0.0;
        out[60] = 0.0;
        out[61] = 0.0;
        out[62] = 0.0;
        out[63] = 0.0;
        out[64] = jac_theta;
        out[65] = 0.0;
        out[66] = 0.0;
        out[67] = 0.0;
        out[68] = 0.0;
        out[69] = 0.0;
        out[70] = jac_sig_aoa[0];
        out[71] = 0.0;
        out[72] = 0.0;
        out[73] = jac_sig_aoa[1];
        out[74] = 0.0;
        out[75] = jac_sig_h[0];
        out[76] = jac_sig_h[1];
        out[77] = jac_sig_h[2];
        out[78] = 0.0;
        out[79] = 0.0;
        out[80] = jac_sig_h[3];
        out[81] = 0.0;
        out[82] = 0.0;
        out[83] = 0.0;
        out[84] = jac_sig_r[0];
        out[85] = jac_sig_r[1];
        out[86] = jac_sig_r[2];
        out[87] = jac_sig_r[3];
        out[88] = jac_sig_r[4];
        out[89] = jac_sig_r[5];
        out[90] = 0.0;
        out[91] = 0.0;
        out[92] = 0.0;
        out[93] = 0.0;
        out[94] = 0.0;
        out[95] = 0.0;
        out[96] = 0.0;
        out[97] = 0.0;
        out[98] = 0.0;
        out[99] = 0.0;
        out[100] = 0.0;
        out[101] = 0.0;
        out[102] = 0.0;
        out[103] = 0.0;
        out[104] = 0.0;
        out[105] = 0.0;
        out[106] = 0.0;
        out[107] = 0.0;
        out[108] = 0.0;
        out[109] = 0.0;
        out[110] = 0.0;
        out[111] = 0.0;
        out[112] = 0.0;
        out[113] = 0.0;
        out[114] = 0.0;
        out[115] = 0.0;
        out[116] = 0.0;
        out[117] = 0.0;
        out[118] = 0.0;
        out[119] = 0.0;

        /* lsq jacobian w.r.t. controls */
        out[120] = 0.0;
        out[121] = 0.0;
        out[122] = 0.0;
        out[123] = 0.0;
        out[124] = 0.0;
        out[125] = 0.0;
        out[126] = 0.0;
        out[127] = 0.0;
        out[128] = 0.0;
        out[129] = 0.0;
        out[130] = 0.0;
        out[131] = 0.0;
        out[132] = 0.0;
        out[133] = 0.0;
        out[134] = 0.0;
        out[135] = 0.0;
        out[136] = 0.0;
        out[137] = 0.0;
        out[138] = 0.0;
        out[139] = 0.0;
        out[140] = 0.0;
        out[141] = 0.0;
        out[142] = 0.0;
        out[143] = 0.0;
        out[144] = 0.0;
        out[145] = 0.0;
        out[146] = 0.0;
        out[147] = jac_uT;
        out[148] = 0.0;
        out[149] = 0.0;
        out[150] = 0.0;
        out[151] = jac_phi_ref;
        out[152] = 0.0;
        out[153] = 0.0;
        out[154] = 0.0;
        out[155] = jac_theta_ref;
    }
}

void acado_evaluateLSQ( const real_t *in, real_t *out )
{
    lsq_obj_eval(in, out, false);
}

void acado_evaluateLSQEndTerm( const real_t *in, real_t *out )
{
	lsq_obj_eval(in, out, true);
}
/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / */

/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / */
/* EXTERNAL FUNCTIONS / / / / / / / / / / / / / / / / / / / / / / / / / /*/
/*
 * below are a collection of functions to be called from outside the MPC
 * internal model.
 */

/* GUIDANCE / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

/* calculate ground velocity reference */
void calculate_velocity_reference(double *v_ref, double *e_lat, double *e_lon,
        const double *states,
        const double *path_reference,
        const double *guidance_params,
        const double *speed_states,
        const double *jac_sig_r,
        const double prio_r)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */
    
    /* states */
    const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    const double v = states[3];
    const double gamma = states[4];
    const double xi = states[5];
    /*const double phi = states[6];
    const double theta = states[7];
    const double n_p = states[8];*/
    
    /* path reference */
    const double b_n = path_reference[0];
    const double b_e = path_reference[1];
    const double b_d = path_reference[2];
    const double Gamma_p = path_reference[3];
    const double chi_p = path_reference[4];
    
    /* guidance */
    const double T_b_lat = guidance_params[0];
    const double T_b_lon = guidance_params[1];
    const double gamma_app_max = guidance_params[2];
    bool use_occ_as_guidance = (guidance_params[3] > 0.5);
    
    /* speed states */
    const double vG_n = speed_states[3];
    const double vG_e = speed_states[4];
    const double vG_d = speed_states[5];
    
    /* path following - - - - - - - - - - - - - - - - - - - - - - - - - -*/
    
    /* path tangent unit vector  */
    const double tP_n_bar = cos(chi_p);
    const double tP_e_bar = sin(chi_p);
    
    /* "closest" point on track */
    const double tp_dot_br = tP_n_bar*(r_n-b_n) + tP_e_bar*(r_e-b_e);
    const double tp_dot_br_n = tp_dot_br*tP_n_bar;
    const double tp_dot_br_e = tp_dot_br*tP_e_bar;
    const double p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar;
    const double p_d = b_d - p_lat*tan(Gamma_p);
        
    /* position error */
    *e_lat = (r_n-b_n)*tP_e_bar - (r_e-b_e)*tP_n_bar;
    *e_lon = p_d - r_d;
    
    /* lateral-directional error boundary */
    const double e_b_lat = T_b_lat * sqrt(vG_n*vG_n + vG_e*vG_e);
    
    /* course approach angle */
    const double chi_app = atan(M_PI_2*(*e_lat)/e_b_lat);
    
    /* longitudinal error boundary */
    double e_b_lon;
    if (fabs(vG_d) < 1.0) {
        e_b_lon = T_b_lon * 0.5 * (1.0 + vG_d*vG_d); /* vG_d may be zero */
    }
    else {
        e_b_lon = T_b_lon * fabs(vG_d);
    }
    
    /* flight path approach angle */
    const double Gamma_app = -gamma_app_max * atan(M_PI_2*(*e_lon)/e_b_lon);
    
    /* normalized ground velocity setpoint */
    const double cos_gamma = cos(Gamma_p + Gamma_app);
    const double vP_n_unit = cos_gamma*cos(chi_p + chi_app);
    const double vP_e_unit = cos_gamma*sin(chi_p + chi_app);
    const double vP_d_unit = -sin(Gamma_p + Gamma_app);
    
    if (use_occ_as_guidance) {
        /* terrain avoidance velocity setpoint */
        const double norm_jac_sig_r = sqrt(jac_sig_r[0]*jac_sig_r[0] + jac_sig_r[1]*jac_sig_r[1] + jac_sig_r[2]*jac_sig_r[2]);
        const double one_over_norm_jac_sig_r = (norm_jac_sig_r > 0.0001) ? 1.0/norm_jac_sig_r : 10000.0;
        const double v_occ_n_unit = -jac_sig_r[0] * one_over_norm_jac_sig_r;
        const double v_occ_e_unit = -jac_sig_r[1] * one_over_norm_jac_sig_r;
        const double v_occ_d_unit = -jac_sig_r[2] * one_over_norm_jac_sig_r;

        /* velocity errors */
        v_ref[0] = vP_n_unit * prio_r + (1.0-prio_r) * v_occ_n_unit;
        v_ref[1] = vP_e_unit * prio_r + (1.0-prio_r) * v_occ_e_unit;
        v_ref[2] = vP_d_unit * prio_r + (1.0-prio_r) * v_occ_d_unit;
    }
    else {
        /* velocity errors */
        v_ref[0] = vP_n_unit;
        v_ref[1] = vP_e_unit;
        v_ref[2] = vP_d_unit;
    }
}

/* SOFT CONSTRAINTS / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

/* calculate soft angle of attack objective */
void calculate_aoa_objective(double *sig_aoa, double *jac_sig_aoa, double *prio_aoa,
        const double *states, const double *aoa_params)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */
    
    /* states */
    /*const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    const double v = states[3];*/
    const double gamma = states[4];
    /*const double xi = states[5];
    const double phi = states[6];*/
    const double theta = states[7];
    /*const double n_p = states[8];*/
    
    /* angle of attack soft constraint */
    const double delta_aoa = aoa_params[0];
    const double aoa_m = aoa_params[1];
    const double aoa_p = aoa_params[2];
    const double log_sqrt_w_over_sig1_aoa = aoa_params[3];
    const double one_over_sqrt_w_aoa = aoa_params[4];
    
    /* angle of attack */
    const double aoa = theta - gamma;
    *sig_aoa = 0.0;
    *prio_aoa = 1.0;
    jac_sig_aoa[0] = 0.0;
    jac_sig_aoa[1] = 0.0;

    /* angle of attack objective / jacobian - - - - - - - - - - - - - - -*/

    if (!(one_over_sqrt_w_aoa<0.0)) {

        /* upper bound */
        const double sig_aoa_p = (aoa - aoa_p < 0.0)
            ? exp((aoa - aoa_p)/delta_aoa*log_sqrt_w_over_sig1_aoa)
            : 1.0 + log_sqrt_w_over_sig1_aoa/delta_aoa * (aoa - aoa_p);

        /* lower bound */
        const double sig_aoa_m = (aoa - aoa_m > 0.0)
            ? exp(-(aoa - aoa_m)/delta_aoa*log_sqrt_w_over_sig1_aoa)
            : 1.0 - log_sqrt_w_over_sig1_aoa/delta_aoa * (aoa - aoa_m);

        /* combined */
        *sig_aoa = sig_aoa_p + sig_aoa_m;
        
        /* jacobian */
        if (aoa - aoa_p > 0.0) {
            /* upper linear jacobian */
            jac_sig_aoa[0] = -log_sqrt_w_over_sig1_aoa/delta_aoa; /* gamma */
            jac_sig_aoa[1] = log_sqrt_w_over_sig1_aoa/delta_aoa; /* theta */
        }
        else if (aoa - aoa_m < 0.0) {
            /* lower linear jacobian */
            jac_sig_aoa[0] = log_sqrt_w_over_sig1_aoa/delta_aoa; /* gamma */
            jac_sig_aoa[1] = -log_sqrt_w_over_sig1_aoa/delta_aoa; /* theta */
        }
        else {
            /* exponential jacobian */
            jac_sig_aoa_exp(jac_sig_aoa, delta_aoa, log_sqrt_w_over_sig1_aoa,
                sig_aoa_m, sig_aoa_p);
        }

        /* prioritization */
        *prio_aoa = 1.0; /*1.0 - ((*sig_aoa*one_over_sqrt_w_aoa > 1.0) ? 1.0 : *sig_aoa*one_over_sqrt_w_aoa);*/
    }
}

/* calculate soft height objective */
void calculate_height_objective(double *sig_h, double *jac_sig_h, double *prio_h, double *h_terr,
        const double *states, const double *terr_params, const double *terr_map)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */
    
    /* states */
    const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    /*const double v = states[3];
    const double gamma = states[4];*/
    const double xi = states[5];
    /*const double phi = states[6];
    const double theta = states[7];
    const double n_p = states[8];*/
    
    /* height params */
    const double h_offset = terr_params[0];
    const double delta_h = terr_params[1];
    const double delta_y = terr_params[2];
    const double log_sqrt_w_over_sig1_h = terr_params[3];
    const double one_over_sqrt_w_h = terr_params[4];
    
    /* radial params */
    /*const double r_offset = terr_params[5];
    const double delta_r0 = terr_params[6];
    const double k_r = terr_params[7];
    const double log_sqrt_w_over_sig1_r = terr_params[8];
    const double one_over_sqrt_w_r = terr_params[9];*/
    
    /* terrain params */
    const double terr_local_origin_n = terr_params[10];
    const double terr_local_origin_e = terr_params[11];
    const double terr_dis = terr_params[12];
    
    /* INTERMEDIATE CALCULATIONS - - - - - - - - - - - - - - - - - - - - */
    
    const double sin_xi = sin(xi);
    const double cos_xi = cos(xi);
    
    /* CALCULATE OBJECTIVE - - - - - - - - - - - - - - - - - - - - - - - */
    
    /* init */
    const double h = -r_d;
    *sig_h = 0.0;
    double sig_h_temp = 0.0;
    *prio_h = 1.0;
    jac_sig_h[0] = 0.0;
    jac_sig_h[1] = 0.0;
    jac_sig_h[2] = 0.0;
    jac_sig_h[3] = 0.0;
    double jac_sig_h_temp[4] = {0.0, 0.0, 0.0, 0.0};
    
    /* if not disabled by weight */
    if (!(one_over_sqrt_w_h<0.0)) {
        
        /* lookup 2.5d grid (CENTER) - - - - - - - - - - - - - - - - - - */
        int idx_q[4];
        double dn, de;    
        double sgn_n = 0.0;
        double sgn_e = 0.0;
        lookup_terrain_idx(r_n, r_e, terr_local_origin_n, terr_local_origin_e, terr_dis, idx_q, &dn, &de);
        
        /* bi-linear interpolation */
        double h12 = (1-dn)*terr_map[idx_q[0]] + dn*terr_map[idx_q[1]];
        double h34 = (1-dn)*terr_map[idx_q[2]] + dn*terr_map[idx_q[3]];
        double h_terr_temp = (1-de)*h12 + de*h34;
        *h_terr = h_terr_temp;
        
        /* objective / jacobian */
        if (h - h_terr_temp - h_offset < 0.0) {
            /* linear */
            *sig_h = 1.0 + -log_sqrt_w_over_sig1_h/delta_h * (h - h_terr_temp - h_offset);
            
            jac_sig_h_lin(jac_sig_h,
                de, delta_h, delta_y,
                terr_map[idx_q[0]], h12, terr_map[idx_q[1]],
                terr_map[idx_q[2]], h34, terr_map[idx_q[3]],
                log_sqrt_w_over_sig1_h, sgn_e,
                sgn_n, terr_dis, xi);
        }
        else {
            /* exponential */
            *sig_h = exp(-(h - h_terr_temp - h_offset)/delta_h*log_sqrt_w_over_sig1_h);
            
            jac_sig_h_exp(jac_sig_h,
                de, delta_h, delta_y,
                terr_map[idx_q[0]], h12, terr_map[idx_q[1]],
                terr_map[idx_q[2]], h34, terr_map[idx_q[3]],
                log_sqrt_w_over_sig1_h, sgn_e, sgn_n, *sig_h,
                terr_dis, xi);
        }

        /* lookup 2.5d grid (LEFT) - - - - - - - - - - - - - - - - - - - */
        sgn_n = 1.0;
        sgn_e = -1.0;
        lookup_terrain_idx(r_n + sgn_n*sin_xi * delta_y, r_e + sgn_e*cos_xi * delta_y, terr_local_origin_n, terr_local_origin_e, terr_dis, idx_q, &dn, &de);
        
        /* bi-linear interpolation */
        h12 = (1-dn)*terr_map[idx_q[0]] + dn*terr_map[idx_q[1]];
        h34 = (1-dn)*terr_map[idx_q[2]] + dn*terr_map[idx_q[3]];
        h_terr_temp = (1-de)*h12 + de*h34;
        
        /* objective / jacobian */
        if (h - h_terr_temp - h_offset < 0.0) {
            /* linear */
            sig_h_temp = 1.0 + -log_sqrt_w_over_sig1_h/delta_h * (h - h_terr_temp - h_offset);
            
            jac_sig_h_lin(jac_sig_h_temp,
                de, delta_h, delta_y,
                terr_map[idx_q[0]], h12, terr_map[idx_q[1]],
                terr_map[idx_q[2]], h34, terr_map[idx_q[3]],
                log_sqrt_w_over_sig1_h, sgn_e,
                sgn_n, terr_dis, xi);
        }
        else {
            /* exponential */
            sig_h_temp = exp(-(h - h_terr_temp - h_offset)/delta_h*log_sqrt_w_over_sig1_h);
            
            jac_sig_h_exp(jac_sig_h_temp,
                de, delta_h, delta_y,
                terr_map[idx_q[0]], h12, terr_map[idx_q[1]], terr_map[idx_q[2]],
                h34, terr_map[idx_q[3]], log_sqrt_w_over_sig1_h,
                sgn_e, sgn_n, sig_h_temp,
                terr_dis, xi);
        }
        *sig_h += sig_h_temp;
        jac_sig_h[0] += jac_sig_h_temp[0];
        jac_sig_h[1] += jac_sig_h_temp[1];
        jac_sig_h[2] += jac_sig_h_temp[2];
        jac_sig_h[3] += jac_sig_h_temp[3];

        /* lookup 2.5d grid (RIGHT) - - - - - - - - - - - - - - - - - - -*/
        sgn_n = -1.0;
        sgn_e = 1.0;
        lookup_terrain_idx(r_n + sgn_n*sin_xi * delta_y, r_e + sgn_e*cos_xi * delta_y, terr_local_origin_n, terr_local_origin_e, terr_dis, idx_q, &dn, &de);
        
        /* bi-linear interpolation */
        h12 = (1-dn)*terr_map[idx_q[0]] + dn*terr_map[idx_q[1]];
        h34 = (1-dn)*terr_map[idx_q[2]] + dn*terr_map[idx_q[3]];
        h_terr_temp = (1-de)*h12 + de*h34;
        
        /* objective / jacobian */
        if (h - h_terr_temp - h_offset < 0.0) {
            /* linear */
            sig_h_temp = 1.0 + -log_sqrt_w_over_sig1_h/delta_h * (h - h_terr_temp - h_offset);
            
            jac_sig_h_lin(jac_sig_h_temp,
                de, delta_h, delta_y,
                terr_map[idx_q[0]], h12, terr_map[idx_q[1]],
                terr_map[idx_q[2]], h34, terr_map[idx_q[3]],
                log_sqrt_w_over_sig1_h, sgn_e,
                sgn_n, terr_dis, xi);
        }
        else {
            /* exponential */
            sig_h_temp = exp(-(h - h_terr_temp - h_offset)/delta_h*log_sqrt_w_over_sig1_h);
            
            jac_sig_h_exp(jac_sig_h_temp,
                de, delta_h, delta_y,
                terr_map[idx_q[0]], h12, terr_map[idx_q[1]], terr_map[idx_q[2]],
                h34, terr_map[idx_q[3]], log_sqrt_w_over_sig1_h,
                sgn_e, sgn_n, sig_h_temp,
                terr_dis, xi);
        }
        *sig_h += sig_h_temp;
        jac_sig_h[0] += jac_sig_h_temp[0];
        jac_sig_h[1] += jac_sig_h_temp[1];
        jac_sig_h[2] += jac_sig_h_temp[2];
        jac_sig_h[3] += jac_sig_h_temp[3];

        /* prioritization */
        *prio_h = 1.0 - ((*sig_h*one_over_sqrt_w_h > 1.0) ? 1.0 : *sig_h*one_over_sqrt_w_h);
    }
}

/* calculate soft radial objective */
void calculate_radial_objective(double *sig_r, double *jac_sig_r, double *prio_r, double *r_occ, int *occ_detected,
        const double *states, const double *speed_states,
        const double *terr_params, const double *terr_map)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */
    
    /* states */
    const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    const double v = states[3];
    const double gamma = states[4];
    const double xi = states[5];
    /*const double phi = states[6];
    const double theta = states[7];
    const double n_p = states[8];*/
    
    /* speed states */
    const double vG_n = speed_states[3];
    const double vG_e = speed_states[4];
    const double vG_d = speed_states[5];
    const double vG_sq = speed_states[6];
    const double vG_norm = speed_states[7];
    const double vG_n_unit = speed_states[9];
    const double vG_e_unit = speed_states[10];
    const double vG_d_unit = speed_states[11];

    /* height params */
    /*const double h_offset = terr_params[0];
    const double delta_h = terr_params[1];
    const double delta_y = terr_params[2];
    const double log_sqrt_w_over_sig1_h = terr_params[3];
    const double one_over_sqrt_w_h = terr_params[4];*/
    
    /* radial params */
    const double r_offset = terr_params[5];
    const double delta_r0 = terr_params[6];
    const double k_r = terr_params[7];
    const double log_sqrt_w_over_sig1_r = terr_params[8];
    const double one_over_sqrt_w_r = terr_params[9];
    
    /* terrain params */
    const double terr_local_origin_n = terr_params[10];
    const double terr_local_origin_e = terr_params[11];
    const double terr_dis = terr_params[12];
    
    /* CALCULATE OBJECTIVE - - - - - - - - - - - - - - - - - - - - - - - */
    
    /* init */
    *sig_r = 0.0;
    jac_sig_r[0] = 0.0;
    jac_sig_r[1] = 0.0;
    jac_sig_r[2] = 0.0;
    jac_sig_r[3] = 0.0;
    jac_sig_r[4] = 0.0;
    jac_sig_r[5] = 0.0;
    *prio_r = 1.0;
    
    /* cast ray along ground speed vector to check for occlusions */
    
    /* init */
    double p_occ[3];
    double p1[3];
    double p2[3];
    double p3[3];
    
    /* unit ray vector */
    double v_ray[3] = {vG_e_unit, vG_n_unit, -vG_d_unit}; /* in ENU */
    
    /* minimum turning radius */
    const double Rmin = vG_sq * k_r;
    
    /* radial buffer zone */
    const double delta_r = delta_r0 + Rmin;
    
    /* adjusted radial offset */
    const double r_offset_1 = r_offset + Rmin;
    
    /* ray length */
    const double d_ray = delta_r + r_offset_1 + terr_dis;
    
    /* ray start ENU */
    const double r0[3] = {r_e, r_n, -r_d};
    /* ray end ENU */
    const double r1[3] = {r0[0] + v_ray[0] * d_ray, r0[1] + v_ray[1] * d_ray, r0[2] + v_ray[2] * d_ray};
    
    /* cast the ray */
    *occ_detected = castray(r_occ, p_occ, p1, p2, p3, r0, r1, v_ray,
        terr_local_origin_n, terr_local_origin_e, terr_dis, terr_map);
    
    /* shift occlusion origin */
    p1[0] = p1[0] + terr_local_origin_e;
    p1[1] = p1[1] + terr_local_origin_n;
    p2[0] = p2[0] + terr_local_origin_e;
    p2[1] = p2[1] + terr_local_origin_n;
    p3[0] = p3[0] + terr_local_origin_e;
    p3[1] = p3[1] + terr_local_origin_n;
    p_occ[0] = p_occ[0] + terr_local_origin_e;
    p_occ[1] = p_occ[1] + terr_local_origin_n;
    
    if (!(one_over_sqrt_w_r<0.0) && (*occ_detected>0)) {
        
        /* objective / jacobian */
        if (*r_occ - r_offset_1 < 0.0) {
            /* linear */
            *sig_r = 1.0 + -log_sqrt_w_over_sig1_r/delta_r * (*r_occ - r_offset_1);
            if (*occ_detected==2) {
                jac_sig_r_tl_lin(jac_sig_r,
                    *r_occ, delta_r, gamma,
                    k_r, log_sqrt_w_over_sig1_r,
                    p1[0], p1[2], p1[1],
                    p2[2], p3[2], r_d,
                    r_e, r_n, r_offset_1,
                    terr_dis, v, vG_norm,
                    vG_d, vG_e, vG_n, xi);
            }
            else if (*occ_detected==1) {
                jac_sig_r_br_lin(jac_sig_r,
                    *r_occ, delta_r, gamma,
                    k_r, log_sqrt_w_over_sig1_r,
                    p1[0], p1[2], p1[1],
                    p2[2], p3[2], r_d,
                    r_e, r_n, r_offset_1,
                    terr_dis, v, vG_norm,
                    vG_d, vG_e, vG_n, xi);
            }
        }
        else {
            /* exponential */
            *sig_r = exp(-(*r_occ - r_offset_1)/delta_r*log_sqrt_w_over_sig1_r);
            if (*occ_detected==2) {
                jac_sig_r_tl_exp(jac_sig_r,
                    *r_occ, delta_r, gamma,
                    k_r, log_sqrt_w_over_sig1_r,
                    p1[0], p1[2], p1[1],
                    p2[2], p3[2], r_d,
                    r_e, r_n, r_offset_1,
                    *sig_r, terr_dis, v,
                    vG_norm, vG_d, vG_e,
                    vG_n, xi);
            }
            else if (*occ_detected==1) {
                jac_sig_r_br_exp(jac_sig_r,
                    *r_occ, delta_r, gamma,
                    k_r, log_sqrt_w_over_sig1_r,
                    p1[0], p1[2], p1[1],
                    p2[2], p3[2], r_d,
                    r_e, r_n, r_offset_1,
                    *sig_r, terr_dis, v,
                    vG_norm, vG_d, vG_e,
                    vG_n, xi);
            }
        }
        jac_sig_r[3] = 0.0;
        /*jac_sig_r[4] = 0.0;
        jac_sig_r[5] = 0.0;*/

        /* prioritization */
        *prio_r = 1.0 - ((*sig_r*one_over_sqrt_w_r > 1.0) ? 1.0 : *sig_r*one_over_sqrt_w_r);
    }
}