#include "acado_common.h"
#include <math.h>
#include <string.h>

#define EPSILON 0.000001

// lookup table constants
// #define LEN_IDX_N 141
// #define LEN_IDX_E 141
// #define LEN_IDX_N_1 140
// #define LEN_IDX_E_1 140
#define LEN_IDX_N 57
#define LEN_IDX_E 57
#define LEN_IDX_N_1 56
#define LEN_IDX_E_1 56

// math functions / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
int constrain_int(int x, int xmin, int xmax) {
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
// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 

// objective helper functions / / / / / / / / / / / / / / / / / / / / / / /
void lookup_terrain_idx(const double pos_n, const double pos_e, const double pos_n_origin,
        const double pos_e_origin, const double terr_dis, int *idx_q, double *dh);

void jac_sig_r_br(double *jac,
        const double r_n, const double r_e, const double r_d,
        const double v, const double gamma, const double xi,
        const double w_e, const double w_n, const double w_d,
        const double terr_dis,
        const double p1_n, const double p1_e, const double p1_h,
        const double p2_n, const double p2_e, const double p2_h,
        const double p3_n, const double p3_e, const double p3_h,
        const double phi_max, const double delta_r0, const double g, const double k_r);
        
void jac_sig_r_tl(double *jac,
        const double r_n, const double r_e, const double r_d,
        const double v, const double gamma, const double xi,
        const double w_e, const double w_n, const double w_d,
        const double terr_dis,
        const double p1_n, const double p1_e, const double p1_h,
        const double p2_n, const double p2_e, const double p2_h,
        const double p3_n, const double p3_e, const double p3_h,
        const double phi_max, const double delta_r0, const double g, const double k_r);

int castray(double *d_occ, double *p_occ, double *p1, double *p2, double *p3,
        const double r0[3], const double r1[3], const double v[3],
        const double pos_n_origin, const double pos_e_origin, const double terr_dis, double *terr_map);

int intersect_triangle(double *d_occ, double *p_occ,
        const double r0[3], const double v[3],
        const double p1[3], const double p2[3], const double p3[3]);
// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 

void lsq_obj_eval( real_t *in, real_t *out, double sig_r, double *jac_sig_r )
{
    /* DEFINE INPUTS -- this is just simply easier to read.. */
    
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
    
    /* controls */
    const double u_T = in[9];
    const double phi_ref = in[10];
    const double theta_ref = in[11];
    
    /* online data */
    
    // disturbances
    const double w_n = in[12];
    const double w_e = in[13];
    const double w_d = in[14];
    
    // path reference
    const double b_n = in[15];
    const double b_e = in[16];
    const double b_d = in[17];
    const double Gamma_p = in[18];
    const double chi_p = in[19];
    
    // guidance
    const double T_b_lat = in[20];
    const double T_b_lon = in[21];
    
    //control augmented attitude time constants and gains
    //const double tau_phi = in[22];
    //const double tau_theta = in[23];
    //const double k_phi = in[24];
    //const double k_theta = in[25];
    
    // soft angle of attack constraints
    const double delta_aoa = in[26];
    const double aoa_m = in[27];
    const double aoa_p = in[28];

    // terrain
    const double delta_h = in[29];
    const double terr_local_origin_n = in[30];
    const double terr_local_origin_e = in[31];
    const double terr_dis = in[32];
    //const double terrain_data = in[33];
    int IDX_TERR_DATA = 33;
    
    /* INTERMEDIATE CALCULATIONS */
    
    // ground speed
    double v_cos_gamma = v*cos(gamma);
    const double cos_xi = cos(xi);
    const double sin_xi = sin(xi);
    const double vG_n = v_cos_gamma*cos_xi + w_n;
    const double vG_e = v_cos_gamma*sin_xi + w_e;
    const double vG_d = -v*sin(gamma) + w_d;
    const double vG_norm = sqrt(vG_n*vG_n + vG_e*vG_e + vG_d*vG_d);
    
    /* PATH FOLLOWING */
    
    // path tangent unit vector 
    const double tP_n_bar = cos(chi_p);
    const double tP_e_bar = sin(chi_p);
    
    // "closest" point on track
    const double tp_dot_br = tP_n_bar*(r_n-b_n) + tP_e_bar*(r_e-b_e);
    const double tp_dot_br_n = tp_dot_br*tP_n_bar;
    const double tp_dot_br_e = tp_dot_br*tP_e_bar;
    const double p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar;
    const double p_d = b_d - p_lat*tan(Gamma_p);
        
    // position error
    const double e_lat = (r_n-b_n)*tP_e_bar - (r_e-b_e)*tP_n_bar;
    const double e_lon = p_d - r_d;
    
    // lateral-directional error boundary
    const double e_b_lat = T_b_lat * sqrt(vG_n*vG_n + vG_e*vG_e);
    
    // course approach angle
    const double chi_app = atan(M_PI_2*e_lat/e_b_lat);
    
    // longitudinal error boundary
    double e_b_lon;
    if (fabs(vG_d) < 1.0) {
        e_b_lon = T_b_lon * 0.5 * (1.0 + vG_d*vG_d); // vG_d may be zero
    }
    else {
        e_b_lon = T_b_lon * fabs(vG_d);
    }
    
    // flight path approach angle
    const double Gamma_app = -0.3/M_PI_2 * atan(M_PI_2*e_lon/e_b_lon); // XXX: MAGIC NUMBER
    
    // ground velocity setpoint
    v_cos_gamma = vG_norm*cos(Gamma_p + Gamma_app);
    const double vP_n = v_cos_gamma*cos(chi_p + chi_app);
    const double vP_e = v_cos_gamma*sin(chi_p + chi_app);
    const double vP_d = -vG_norm*sin(Gamma_p + Gamma_app);
    
    // terrain avoidance velocity setpoint
    const double norm_jac_sig_r = sqrt(jac_sig_r[0]*jac_sig_r[0] + jac_sig_r[1]*jac_sig_r[1] + jac_sig_r[2]*jac_sig_r[2]);
    const double one_over_norm_jac_sig_r = (norm_jac_sig_r > 0.0001) ? 1.0/norm_jac_sig_r : 10000.0;
    const double v_occ_n = -jac_sig_r[0] * one_over_norm_jac_sig_r * vG_norm;
    const double v_occ_e = -jac_sig_r[1] * one_over_norm_jac_sig_r * vG_norm;
    const double v_occ_d = -jac_sig_r[2] * one_over_norm_jac_sig_r * vG_norm;
    
    /* SOFT CONSTRAINTS */
    
    // angle of attack
    const double aoa = theta - gamma;
    double sig_aoa = 0.0;
    if (aoa > aoa_p - delta_aoa) {
        sig_aoa = fabs(aoa - aoa_p + delta_aoa) / delta_aoa;
        sig_aoa = sig_aoa * sig_aoa * sig_aoa;
    }
    if (aoa < aoa_m + delta_aoa) {
        sig_aoa = fabs(aoa - aoa_m - delta_aoa) / delta_aoa;
        sig_aoa = sig_aoa * sig_aoa * sig_aoa;
    }
       
    /* TERRAIN */
    
    // lookup 2.5d grid
    int idx_q[4];
    double dh[2];
    lookup_terrain_idx(r_n, r_e, terr_local_origin_n, terr_local_origin_e, terr_dis, idx_q, dh);
    
    // bi-linear interpolation
    double h12 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[0]] + dh[0]*in[IDX_TERR_DATA+idx_q[1]];
    double h34 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[2]] + dh[0]*in[IDX_TERR_DATA+idx_q[3]];
    const double h_terr = (1-dh[1])*h12 + dh[1]*h34;
    
    // soft constraint
    double sig_h = 0.0;
    if (-r_d < h_terr + delta_h) {
        sig_h = fabs(-r_d - h_terr - delta_h) / delta_h;
        sig_h = sig_h * sig_h * sig_h;
    }
    
    // TEST - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    const double r_side = 5.0;
    
    // lookup 2.5d grid (left side)
    lookup_terrain_idx(r_n + sin_xi * r_side, r_e + -cos_xi * r_side, terr_local_origin_n, terr_local_origin_e, terr_dis, idx_q, dh);
    
    // bi-linear interpolation
    h12 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[0]] + dh[0]*in[IDX_TERR_DATA+idx_q[1]];
    h34 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[2]] + dh[0]*in[IDX_TERR_DATA+idx_q[3]];
    const double h_terr_left = (1-dh[1])*h12 + dh[1]*h34;
    
    // soft constraint
    if (-r_d < h_terr_left + delta_h) {
        const double sig_h_left = fabs(-r_d - h_terr_left - delta_h) / delta_h;
        sig_h = sig_h + sig_h_left * sig_h_left * sig_h_left;
    }
    
    // lookup 2.5d grid (right side)
    lookup_terrain_idx(r_n + -sin_xi * r_side, r_e + cos_xi * r_side, terr_local_origin_n, terr_local_origin_e, terr_dis, idx_q, dh);
    
    // bi-linear interpolation
    h12 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[0]] + dh[0]*in[IDX_TERR_DATA+idx_q[1]];
    h34 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[2]] + dh[0]*in[IDX_TERR_DATA+idx_q[3]];
    const double h_terr_right = (1-dh[1])*h12 + dh[1]*h34;
    
    // soft constraint
    if (-r_d < h_terr_right + delta_h) {
        const double sig_h_right = fabs(-r_d - h_terr_right - delta_h) / delta_h;
        sig_h = sig_h + sig_h_right * sig_h_right * sig_h_right;
    }
    
    // TEST - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    // prioritization
    const double prio_aoa = 1.0;//1.0 - ((sig_aoa > 1.0) ? 1.0 : sig_aoa);
    const double prio_h = 1.0 - ((sig_h > 1.0) ? 1.0 : sig_h);
    const double prio_r = 1.0 - ((sig_r > 1.0) ? 1.0 : sig_r);
    const double prio_aoa_h = prio_aoa * prio_h * prio_r;
    
    // velocity error
//     const double e_v_n = vP_n * prio_r + (1.0-prio_r) * v_occ_n - vG_n;
//     const double e_v_e = vP_e * prio_r + (1.0-prio_r) * v_occ_e - vG_e;
//     const double e_v_d = vP_d * prio_r + (1.0-prio_r) * v_occ_d - vG_d;
    const double e_v_n = vP_n - vG_n;
    const double e_v_e = vP_e - vG_e;
    const double e_v_d = vP_d - vG_d;
    
    // state output
    out[0] = e_lat * prio_aoa_h;
    out[1] = e_lon * prio_aoa_h;
    out[2] = e_v_n * prio_aoa_h;
    out[3] = e_v_e * prio_aoa_h;
    out[4] = e_v_d * prio_aoa_h;
    out[5] = v;
    out[6] = sig_aoa;
    out[7] = sig_h * prio_aoa;
    
    // control output
    out[9] = u_T;
    out[10] = phi_ref;
    out[11] = theta_ref;
}

void acado_evaluateLSQ( const real_t *in, real_t *out )
{
    double in_Delta[ACADO_NX+ACADO_NU+ACADO_NOD];
    memcpy(in_Delta, in, sizeof(in_Delta));
    
    // HANDLE OTHER OBJECTIVES / / / / / / / / / / / / / / / / / / / / / /
    
    // copy for easy reading..
    /* states */
    const double r_n = in_Delta[0];
    const double r_e = in_Delta[1];
    const double r_d = in_Delta[2];
    const double v = in_Delta[3];
    const double gamma = in_Delta[4];
    const double xi = in_Delta[5];
    /* online data */
    const double w_n = in_Delta[12];
    const double w_e = in_Delta[13];
    const double w_d = in_Delta[14];
    // soft angle of attack constraints
    const double delta_aoa = in_Delta[26];
    const double aoa_m = in_Delta[27];
    const double aoa_p = in_Delta[28];
    // terrain
    const double terr_local_origin_n = in_Delta[30];
    const double terr_local_origin_e = in_Delta[31];
    const double terr_dis = in_Delta[32];
    int IDX_TERR_DATA = 33;
    // intermediate calculations
    // ground speed
    double v_cos_gamma = v*cos(gamma);
    const double vG_n = v_cos_gamma*cos(xi) + w_n;
    const double vG_e = v_cos_gamma*sin(xi) + w_e;
    const double vG_d = -v*sin(gamma) + w_d;
    const double vG_norm = sqrt(vG_n*vG_n + vG_e*vG_e + vG_d*vG_d);
    
    // cast ray along ground speed vector to check for occlusions
    double d_occ;
    double p_occ[3];
    double p1[3];
    double p2[3];
    double p3[3];
    double v_ray[3] = {vG_e/vG_norm, vG_n/vG_norm, -vG_d/vG_norm}; // in ENU
    const double phi_max = 0.6109;
    const double delta_r0 = 10.0;
    const double g = 9.81;
    const double k_r = 3.0;
    const double delta_r = vG_norm * vG_norm * 0.1456 * k_r + delta_r0; // radial buffer zone
    const double d_ray = delta_r + terr_dis;
    const double r0[3] = {r_e, r_n, -r_d};
    const double r1[3] = {r0[0] + v_ray[0] * d_ray, r0[1] + v_ray[1] * d_ray, r0[2] + v_ray[2] * d_ray};
    int occ_detected = castray(&d_occ, p_occ, p1, p2, p3, r0, r1, v_ray,
        terr_local_origin_n, terr_local_origin_e, terr_dis, in_Delta+IDX_TERR_DATA);
    p1[0] = p1[0] + terr_local_origin_e;
    p1[1] = p1[1] + terr_local_origin_n;
    p2[0] = p2[0] + terr_local_origin_e;
    p2[1] = p2[1] + terr_local_origin_n;
    p3[0] = p3[0] + terr_local_origin_e;
    p3[1] = p3[1] + terr_local_origin_n;
    p_occ[0] = p_occ[0] + terr_local_origin_e;
    p_occ[1] = p_occ[1] + terr_local_origin_n;
    
    // calculate radial cost
    double sig_r = 0.0;
    if ((d_occ < delta_r) && occ_detected>0) {
        sig_r = fabs(delta_r - d_occ)/delta_r;
        sig_r = sig_r*sig_r*sig_r;
    }
    out[8] = sig_r;
    
    // calculate radial cost jacobians
    double jac_sig_r[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (occ_detected==2) {
        jac_sig_r_tl(jac_sig_r,
            r_n, r_e, r_d,
            v, gamma, xi,
            w_e, w_n, w_d,
            terr_dis,
            p1[0], p1[1], p1[2],
            p2[0], p2[1], p2[2],
            p3[0], p3[1], p3[2],
            phi_max, delta_r0, g, k_r);
    }
    else if (occ_detected==1) {
        jac_sig_r_br(jac_sig_r,
            r_n, r_e, r_d,
            v, gamma, xi,
            w_e, w_n, w_d,
            terr_dis,
            p1[0], p1[1], p1[2],
            p2[0], p2[1], p2[2],
            p3[0], p3[1], p3[2],
            phi_max, delta_r0, g, k_r);
    }
    jac_sig_r[3] = 0.0;
    jac_sig_r[4] = 0.0;
    jac_sig_r[5] = 0.0;
    
    // EVALUATE (MOST) OBJECTIVES / / / / / / / / / / / / / / / / / / / / /
    lsq_obj_eval( in_Delta, out, sig_r, jac_sig_r );

    /* lsq_obj jacobians */

    // numerical jacobians / / / / / / / / / / / / / / / / / / / / / / / /
    double f_Delta_m[ACADO_NY];
    double f_Delta_p[ACADO_NY];
    const double Delta = 0.00001;
    const double Delta2 = 2.0 * Delta;

    int i;
    int j;
    for (i = 0; i < ACADO_NX; i=i+1) {

        in_Delta[i] = in[i] - Delta;
        lsq_obj_eval( in_Delta, f_Delta_m, sig_r, jac_sig_r );
        in_Delta[i] = in[i] + Delta;
        lsq_obj_eval( in_Delta, f_Delta_p, sig_r, jac_sig_r );
        in_Delta[i] = in[i];
        
        for (j = 0; j < 8; j=j+1) {
            out[ACADO_NY+j*ACADO_NX+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;
        }
        for (j = 9; j < ACADO_NY; j=j+1) {
            out[ACADO_NY+j*ACADO_NX+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;
        }
    }
    out[ACADO_NY+8*ACADO_NX+0] = jac_sig_r[0];
    out[ACADO_NY+8*ACADO_NX+1] = jac_sig_r[1];
    out[ACADO_NY+8*ACADO_NX+2] = jac_sig_r[2];
    out[ACADO_NY+8*ACADO_NX+3] = jac_sig_r[3];
    out[ACADO_NY+8*ACADO_NX+4] = jac_sig_r[4];
    out[ACADO_NY+8*ACADO_NX+5] = jac_sig_r[5];
    out[ACADO_NY+8*ACADO_NX+6] = 0.0;
    out[ACADO_NY+8*ACADO_NX+7] = 0.0;
    out[ACADO_NY+8*ACADO_NX+8] = 0.0;

    for (i = 0; i < ACADO_NU; i=i+1) {

        in_Delta[i+ACADO_NX] = in[i+ACADO_NX] - Delta;
        lsq_obj_eval( in_Delta, f_Delta_m, sig_r, jac_sig_r );
        in_Delta[i+ACADO_NX] = in[i+ACADO_NX] + Delta;
        lsq_obj_eval( in_Delta, f_Delta_p, sig_r, jac_sig_r );
        in_Delta[i+ACADO_NX] = in[i+ACADO_NX];

        for (j = 0; j < 8; j=j+1) {
            out[ACADO_NY+ACADO_NY*ACADO_NX+j*ACADO_NU+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;
        }
        for (j = 9; j < ACADO_NY-1; j=j+1) {
            out[ACADO_NY+ACADO_NY*ACADO_NX+j*ACADO_NU+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;
        }
    }
    out[ACADO_NY+ACADO_NY*ACADO_NX+8*ACADO_NU+0] = 0.0;
    out[ACADO_NY+ACADO_NY*ACADO_NX+8*ACADO_NU+1] = 0.0;
    out[ACADO_NY+ACADO_NY*ACADO_NX+8*ACADO_NU+2] = 0.0;

}

void lsq_objN_eval( real_t *in, real_t *out, double sig_r, double *jac_sig_r )
{
    /* DEFINE INPUTS -- this is just simply easier to read.. */
    
    /* states */
    const double r_n = in[0];
    const double r_e = in[1];
    const double r_d = in[2];
    const double v = in[3];
    const double gamma = in[4];
    const double xi = in[5];
    const double phi = in[6];
    const double theta = in[7];
    //const double n_p = in[8];
    
    /* online data */
    
    // disturbances
    const double w_n = in[9];
    const double w_e = in[10];
    const double w_d = in[11];
    
    // path reference
    const double b_n = in[12];
    const double b_e = in[13];
    const double b_d = in[14];
    const double Gamma_p = in[15];
    const double chi_p = in[16];
    
    // guidance
    const double T_b_lat = in[17];
    const double T_b_lon = in[18];
    
    //control augmented attitude time constants and gains
    //const double tau_phi = in[19];
    //const double tau_theta = in[20];
    //const double k_phi = in[21];
    //const double k_theta = in[22];
    
    // soft angle of attack constraints
    const double delta_aoa = in[23];
    const double aoa_m = in[24];
    const double aoa_p = in[25];

    // terrain
    const double delta_h = in[26];
    const double terr_local_origin_n = in[27];
    const double terr_local_origin_e = in[28];
    const double terr_dis = in[29];
    //const double terrain_data = in[30];
    int IDX_TERR_DATA = 30;
    
    /* INTERMEDIATE CALCULATIONS */
    
    // ground speed
    double v_cos_gamma = v*cos(gamma);
    const double cos_xi = cos(xi);
    const double sin_xi = sin(xi);
    const double vG_n = v_cos_gamma*cos_xi + w_n;
    const double vG_e = v_cos_gamma*sin_xi + w_e;
    const double vG_d = -v*sin(gamma) + w_d;
    const double vG_norm = sqrt(vG_n*vG_n + vG_e*vG_e + vG_d*vG_d);
    
    /* PATH FOLLOWING */
    
    // path tangent unit vector 
    const double tP_n_bar = cos(chi_p);
    const double tP_e_bar = sin(chi_p);
    
    // "closest" point on track
    const double tp_dot_br = tP_n_bar*(r_n-b_n) + tP_e_bar*(r_e-b_e);
    const double tp_dot_br_n = tp_dot_br*tP_n_bar;
    const double tp_dot_br_e = tp_dot_br*tP_e_bar;
    const double p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar;
    const double p_d = b_d - p_lat*tan(Gamma_p);
        
    // position error
    const double e_lat = (r_n-b_n)*tP_e_bar - (r_e-b_e)*tP_n_bar;
    const double e_lon = p_d - r_d;
    
    // lateral-directional error boundary
    const double e_b_lat = T_b_lat * sqrt(vG_n*vG_n + vG_e*vG_e);
    
    // course approach angle
    const double chi_app = atan(M_PI_2*e_lat/e_b_lat);
    
    // longitudinal error boundary
    double e_b_lon;
    if (fabs(vG_d) < 1.0) {
        e_b_lon = T_b_lon * 0.5 * (1.0 + vG_d*vG_d); // vG_d may be zero
    }
    else {
        e_b_lon = T_b_lon * fabs(vG_d);
    }
    
    // flight path approach angle
    const double Gamma_app = -0.3/M_PI_2 * atan(M_PI_2*e_lon/e_b_lon); // XXX: MAGIC NUMBER
    
    // ground velocity setpoint
    v_cos_gamma = vG_norm*cos(Gamma_p + Gamma_app);
    const double vP_n = v_cos_gamma*cos(chi_p + chi_app);
    const double vP_e = v_cos_gamma*sin(chi_p + chi_app);
    const double vP_d = -vG_norm*sin(Gamma_p + Gamma_app);
    
    // terrain avoidance velocity setpoint
    const double norm_jac_sig_r = sqrt(jac_sig_r[0]*jac_sig_r[0] + jac_sig_r[1]*jac_sig_r[1] + jac_sig_r[2]*jac_sig_r[2]);
    const double one_over_norm_jac_sig_r = (norm_jac_sig_r > 0.0001) ? 1.0/norm_jac_sig_r : 10000.0;
    const double v_occ_n = -jac_sig_r[0] * one_over_norm_jac_sig_r * vG_norm;
    const double v_occ_e = -jac_sig_r[1] * one_over_norm_jac_sig_r * vG_norm;
    const double v_occ_d = -jac_sig_r[2] * one_over_norm_jac_sig_r * vG_norm;
    
    /* SOFT CONSTRAINTS */
    
    // angle of attack
    const double aoa = theta - gamma;
    double sig_aoa = 0.0;
    if (aoa > aoa_p - delta_aoa) {
        sig_aoa = fabs(aoa - aoa_p + delta_aoa) / delta_aoa;
        sig_aoa = sig_aoa * sig_aoa * sig_aoa;
    }
    if (aoa < aoa_m + delta_aoa) {
        sig_aoa = fabs(aoa - aoa_m - delta_aoa) / delta_aoa;
        sig_aoa = sig_aoa * sig_aoa * sig_aoa;
    }
       
    /* TERRAIN */
    
    // lookup 2.5d grid
    int idx_q[4];
    double dh[2];
    lookup_terrain_idx(r_n, r_e, terr_local_origin_n, terr_local_origin_e, terr_dis, idx_q, dh);
    
    // bi-linear interpolation
    double h12 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[0]] + dh[0]*in[IDX_TERR_DATA+idx_q[1]];
    double h34 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[2]] + dh[0]*in[IDX_TERR_DATA+idx_q[3]];
    const double h_terr = (1-dh[1])*h12 + dh[1]*h34;
    
    // soft constraint
    double sig_h = 0.0;
    if (-r_d < h_terr + delta_h) {
        sig_h = fabs(-r_d - h_terr - delta_h) / delta_h;
        sig_h = sig_h * sig_h * sig_h;
    }
    
    // TEST - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    const double r_side = 5.0;
    
    // lookup 2.5d grid (left side)
    lookup_terrain_idx(r_n + sin_xi * r_side, r_e + -cos_xi * r_side, terr_local_origin_n, terr_local_origin_e, terr_dis, idx_q, dh);
    
    // bi-linear interpolation
    h12 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[0]] + dh[0]*in[IDX_TERR_DATA+idx_q[1]];
    h34 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[2]] + dh[0]*in[IDX_TERR_DATA+idx_q[3]];
    const double h_terr_left = (1-dh[1])*h12 + dh[1]*h34;
    
    // soft constraint
    if (-r_d < h_terr_left + delta_h) {
        const double sig_h_left = fabs(-r_d - h_terr_left - delta_h) / delta_h;
        sig_h = sig_h + sig_h_left * sig_h_left * sig_h_left;
    }
    
    // lookup 2.5d grid (right side)
    lookup_terrain_idx(r_n + -sin_xi * r_side, r_e + cos_xi * r_side, terr_local_origin_n, terr_local_origin_e, terr_dis, idx_q, dh);
    
    // bi-linear interpolation
    h12 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[0]] + dh[0]*in[IDX_TERR_DATA+idx_q[1]];
    h34 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[2]] + dh[0]*in[IDX_TERR_DATA+idx_q[3]];
    const double h_terr_right = (1-dh[1])*h12 + dh[1]*h34;
    
    // soft constraint
    if (-r_d < h_terr_right + delta_h) {
        const double sig_h_right = fabs(-r_d - h_terr_right - delta_h) / delta_h;
        sig_h = sig_h + sig_h_right * sig_h_right * sig_h_right;
    }
    
    // TEST - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    // prioritization
    const double prio_aoa = 1.0;//1.0 - ((sig_aoa > 1.0) ? 1.0 : sig_aoa);
    const double prio_h = 1.0 - ((sig_h > 1.0) ? 1.0 : sig_h);
    const double prio_r = 1.0 - ((sig_r > 1.0) ? 1.0 : sig_r);
    const double prio_aoa_h = prio_aoa * prio_h * prio_r;
    
    // velocity error
//     const double e_v_n = vP_n * prio_r + (1.0-prio_r) * v_occ_n - vG_n;
//     const double e_v_e = vP_e * prio_r + (1.0-prio_r) * v_occ_e - vG_e;
//     const double e_v_d = vP_d * prio_r + (1.0-prio_r) * v_occ_d - vG_d;
    const double e_v_n = vP_n - vG_n;
    const double e_v_e = vP_e - vG_e;
    const double e_v_d = vP_d - vG_d;
    
    // state output
    out[0] = e_lat * prio_aoa_h;
    out[1] = e_lon * prio_aoa_h;
    out[2] = e_v_n * prio_aoa_h;
    out[3] = e_v_e * prio_aoa_h;
    out[4] = e_v_d * prio_aoa_h;
    out[5] = v;
    out[6] = sig_aoa;
    out[7] = sig_h * prio_aoa;
}

void acado_evaluateLSQEndTerm( const real_t *in, real_t *out )
{
    double in_Delta[ACADO_NX+ACADO_NOD];
    memcpy(in_Delta, in, sizeof(in_Delta));
    
    // HANDLE OTHER OBJECTIVES / / / / / / / / / / / / / / / / / / / / / /
    
    // copy for easy reading..
    /* states */
    const double r_n = in_Delta[0];
    const double r_e = in_Delta[1];
    const double r_d = in_Delta[2];
    const double v = in_Delta[3];
    const double gamma = in_Delta[4];
    const double xi = in_Delta[5];
    /* online data */
    const double w_n = in_Delta[9];
    const double w_e = in_Delta[10];
    const double w_d = in_Delta[11];
    // soft angle of attack constraints
    const double delta_aoa = in_Delta[23];
    const double aoa_m = in_Delta[24];
    const double aoa_p = in_Delta[25];
    // terrain
    const double terr_local_origin_n = in_Delta[27];
    const double terr_local_origin_e = in_Delta[28];
    const double terr_dis = in_Delta[29];
    int IDX_TERR_DATA = 30;
    // intermediate calculations
    // ground speed
    double v_cos_gamma = v*cos(gamma);
    const double vG_n = v_cos_gamma*cos(xi) + w_n;
    const double vG_e = v_cos_gamma*sin(xi) + w_e;
    const double vG_d = -v*sin(gamma) + w_d;
    const double vG_norm = sqrt(vG_n*vG_n + vG_e*vG_e + vG_d*vG_d);
    
    // cast ray along ground speed vector to check for occlusions
    double d_occ;
    double p_occ[3];
    double p1[3];
    double p2[3];
    double p3[3];
    double v_ray[3] = {vG_e/vG_norm, vG_n/vG_norm, -vG_d/vG_norm}; // in ENU
    const double phi_max = 0.6109;
    const double delta_r0 = 10.0;
    const double g = 9.81;
    const double k_r = 3.0;
    const double delta_r = vG_norm * vG_norm * 0.1456 * k_r + delta_r0; // radial buffer zone
    const double d_ray = delta_r + terr_dis;
    const double r0[3] = {r_e, r_n, -r_d};
    const double r1[3] = {r0[0] + v_ray[0] * d_ray, r0[1] + v_ray[1] * d_ray, r0[2] + v_ray[2] * d_ray};
    int occ_detected = castray(&d_occ, p_occ, p1, p2, p3, r0, r1, v_ray,
        terr_local_origin_n, terr_local_origin_e, terr_dis, in_Delta+IDX_TERR_DATA);
    p1[0] = p1[0] + terr_local_origin_e;
    p1[1] = p1[1] + terr_local_origin_n;
    p2[0] = p2[0] + terr_local_origin_e;
    p2[1] = p2[1] + terr_local_origin_n;
    p3[0] = p3[0] + terr_local_origin_e;
    p3[1] = p3[1] + terr_local_origin_n;
    p_occ[0] = p_occ[0] + terr_local_origin_e;
    p_occ[1] = p_occ[1] + terr_local_origin_n;
    
    // calculate radial cost
    double sig_r = 0.0;
    if ((d_occ < delta_r) && occ_detected>0) {
        sig_r = fabs(delta_r - d_occ)/delta_r;
        sig_r = sig_r*sig_r*sig_r;
    }
    out[8] = sig_r;
    
    // calculate radial cost jacobians
    double jac_sig_r[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (occ_detected==2) {
        jac_sig_r_tl(jac_sig_r, r_n, r_e, r_d,
            v, gamma, xi,
            w_e, w_n, w_d,
            terr_dis,
            p1[0], p1[1], p1[2],
            p2[0], p2[1], p2[2],
            p3[0], p3[1], p3[2],
            phi_max, delta_r0, g, k_r);
    }
    else if (occ_detected==1) {
        jac_sig_r_br(jac_sig_r, r_n, r_e, r_d,
            v, gamma, xi,
            w_e, w_n, w_d,
            terr_dis,
            p1[0], p1[1], p1[2],
            p2[0], p2[1], p2[2],
            p3[0], p3[1], p3[2],
            phi_max, delta_r0, g, k_r);
    }
    jac_sig_r[3] = 0.0;
    jac_sig_r[4] = 0.0;
    jac_sig_r[5] = 0.0;
    
    // EVALUATE (MOST) OBJECTIVES / / / / / / / / / / / / / / / / / / / / /
    lsq_objN_eval( in_Delta, out, sig_r, jac_sig_r );
    
    // numerical jacobians / / / / / / / / / / / / / / / / / / / / / / / /

    double f_Delta_m[ACADO_NYN];
    double f_Delta_p[ACADO_NYN];
    const double Delta = 0.00001;
    const double Delta2 = 2.0 * Delta;

    int i;
    int j;
    for (i = 0; i < ACADO_NX; i=i+1) {

        in_Delta[i] = in[i] - Delta;
        lsq_objN_eval( in_Delta, f_Delta_m, sig_r, jac_sig_r );
        in_Delta[i] = in[i] + Delta;
        lsq_objN_eval( in_Delta, f_Delta_p, sig_r, jac_sig_r );
        in_Delta[i] = in[i];

        for (j = 0; j < ACADO_NYN-1; j=j+1) {
            out[ACADO_NYN+j*ACADO_NX+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;
        }
    }
    out[ACADO_NYN+(ACADO_NYN-1)*ACADO_NX+0] = jac_sig_r[0];
    out[ACADO_NYN+(ACADO_NYN-1)*ACADO_NX+1] = jac_sig_r[1];
    out[ACADO_NYN+(ACADO_NYN-1)*ACADO_NX+2] = jac_sig_r[2];
    out[ACADO_NYN+(ACADO_NYN-1)*ACADO_NX+3] = jac_sig_r[3];
    out[ACADO_NYN+(ACADO_NYN-1)*ACADO_NX+4] = jac_sig_r[4];
    out[ACADO_NYN+(ACADO_NYN-1)*ACADO_NX+5] = jac_sig_r[5];
    out[ACADO_NYN+(ACADO_NYN-1)*ACADO_NX+6] = 0.0;
    out[ACADO_NYN+(ACADO_NYN-1)*ACADO_NX+7] = 0.0;
    out[ACADO_NYN+(ACADO_NYN-1)*ACADO_NX+8] = 0.0;
}

void lookup_terrain_idx(const double pos_n, const double pos_e, const double pos_n_origin,
        const double pos_e_origin, const double terr_dis, int *idx_q, double *dh)
{
    // relative position / indices
    const double rel_n = pos_n - pos_n_origin;
    const double rel_n_bar = rel_n / terr_dis;
    int idx_n = int(floor(rel_n_bar));
    const double rel_e = pos_e - pos_e_origin;
    const double rel_e_bar = rel_e / terr_dis;
    int idx_e = int(floor(rel_e_bar));
    
    // interpolation weights
    const double dh_n = rel_n_bar-idx_n;
    const double dh_e = rel_e_bar-idx_e;
    
    // cap ends
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
    
    // neighbors (north)
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
    // neighbors (east)
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
    
    // neighbors row-major indices
    idx_q[0] = q_n[0]*LEN_IDX_E + q_e[0];
    idx_q[1] = q_n[1]*LEN_IDX_E + q_e[1];
    idx_q[2] = q_n[2]*LEN_IDX_E + q_e[2];
    idx_q[3] = q_n[3]*LEN_IDX_E + q_e[3];
    
    // interpolation weights
    dh[0] = dh_n;
    dh[1] = dh_e;
}

void jac_sig_r_br(double *jac,
        const double r_n, const double r_e, const double r_d,
        const double v, const double gamma, const double xi,
        const double w_e, const double w_n, const double w_d,
        const double terr_dis,
        const double p1_n, const double p1_e, const double p1_h,
        const double p2_n, const double p2_e, const double p2_h,
        const double p3_n, const double p3_e, const double p3_h,
        const double phi_max, const double delta_r0, const double g, const double k_r) {
        
    const double t3 = cos(gamma); 
    const double t8 = cos(xi); 
    const double t9 = t3*t8*v; 
    const double t2 = t9+w_n; 
    const double t11 = sin(xi); 
    const double t12 = t3*t11*v; 
    const double t4 = t12+w_e; 
    const double t6 = sin(gamma); 
    const double t7 = t6*v; 
    const double t5 = -t7+w_d; 
    const double t10 = p2_h-p3_h; 
    const double t13 = t2*t2; 
    const double t14 = t4*t4; 
    const double t15 = t5*t5; 
    const double t16 = t13+t14+t15; 
    const double t17 = terr_dis*terr_dis; 
    const double t18 = p1_h-p2_h; 
    const double t19 = 1.0/g; 
    const double t20 = tan(phi_max); 
    const double t21 = 1.0/t20; 
    const double t22 = k_r*t16*t19*t21; 
    const double t24 = t5*t17; 
    const double t25 = t2*t10*terr_dis; 
    const double t26 = t4*t18*terr_dis; 
    const double t27 = -t24+t25+t26; 
    const double t28 = 1.0/t27; 
    const double t29 = sqrt(t16); 
    const double t32 = p1_h+r_d; 
    const double t33 = t17*t32; 
    const double t34 = p1_h*t17; 
    const double t35 = p1_e-r_e; 
    const double t36 = t18*t35*terr_dis; 
    const double t37 = p1_n-r_n; 
    const double t38 = t10*t37*terr_dis; 
    const double t39 = t33+t34+t36+t38; 
    const double t40 = t28*t29*t39; 
    const double t23 = delta_r0+t22-t40; 
    const double t30 = delta_r0+t22; 
    const double t31 = 1.0/(t30*t30*t30); 
    const double t41 = t23*t23; 
    const double t42 = t2*t3*t8*2.0; 
    const double t43 = t3*t4*t11*2.0; 
    const double t45 = t5*t6*2.0; 
    const double t44 = t42+t43-t45; 
    const double t46 = 1.0/(t27*t27); 
    const double t47 = t3*t5*v*2.0; 
    const double t48 = t2*t6*t8*v*2.0; 
    const double t49 = t4*t6*t11*v*2.0; 
    const double t50 = t47+t48+t49; 
    const double t51 = 1.0/sqrt(t16); 
    const double t52 = 1.0/(t30*t30*t30*t30); 
    const double t53 = t3*t4*t8*v*2.0; 
    const double t55 = t2*t3*t11*v*2.0; 
    const double t54 = t53-t55; 
    jac[0] = t10*t28*t29*t31*t41*terr_dis*3.0; 
    jac[1] = t18*t28*t29*t31*t41*terr_dis*3.0; 
    jac[2] = t17*t28*t29*t31*t41*-3.0; 
    jac[3] = t31*t41*(k_r*t19*t21*t44-t28*t39*t44*t51*(1.0/2.0)+t29*t39*t46*(t6*t17+t3*t8*t10*terr_dis+t3*t11*t18*terr_dis))*3.0-k_r*t19*t21*t23*t41*t44*t52*3.0; 
    jac[4] = t31*t41*(t29*t39*t46*(-t3*t17*v+t6*t8*t10*terr_dis*v+t6*t11*t18*terr_dis*v)+k_r*t19*t21*t50-t28*t39*t50*t51*(1.0/2.0))*-3.0+k_r*t19*t21*t23*t41*t50*t52*3.0; 
    jac[5] = t31*t41*(-k_r*t19*t21*t54+t28*t39*t51*t54*(1.0/2.0)+t29*t39*t46*(t3*t10*t11*terr_dis*v-t3*t8*t18*terr_dis*v))*-3.0-k_r*t19*t21*t23*t41*t52*t54*3.0; 
}

void jac_sig_r_tl(double *jac,
        const double r_n, const double r_e, const double r_d,
        const double v, const double gamma, const double xi,
        const double w_e, const double w_n, const double w_d,
        const double terr_dis,
        const double p1_n, const double p1_e, const double p1_h,
        const double p2_n, const double p2_e, const double p2_h,
        const double p3_n, const double p3_e, const double p3_h,
        const double phi_max, const double delta_r0, const double g, const double k_r) {
        
    const double t3 = cos(gamma); 
    const double t8 = cos(xi); 
    const double t9 = t3*t8*v; 
    const double t2 = t9+w_n; 
    const double t11 = sin(xi); 
    const double t12 = t3*t11*v; 
    const double t4 = t12+w_e; 
    const double t6 = sin(gamma); 
    const double t7 = t6*v; 
    const double t5 = -t7+w_d; 
    const double t10 = p1_h-p2_h; 
    const double t13 = t2*t2; 
    const double t14 = t4*t4; 
    const double t15 = t5*t5; 
    const double t16 = t13+t14+t15; 
    const double t17 = terr_dis*terr_dis; 
    const double t18 = p2_h-p3_h; 
    const double t19 = 1.0/g; 
    const double t20 = tan(phi_max); 
    const double t21 = 1.0/t20; 
    const double t22 = k_r*t16*t19*t21; 
    const double t24 = t5*t17; 
    const double t25 = t2*t10*terr_dis; 
    const double t26 = t4*t18*terr_dis; 
    const double t27 = -t24+t25+t26; 
    const double t28 = 1.0/t27; 
    const double t29 = sqrt(t16); 
    const double t32 = p1_h+r_d; 
    const double t33 = t17*t32; 
    const double t34 = p1_h*t17; 
    const double t35 = p1_e-r_e; 
    const double t36 = t18*t35*terr_dis; 
    const double t37 = p1_n-r_n; 
    const double t38 = t10*t37*terr_dis; 
    const double t39 = t33+t34+t36+t38; 
    const double t40 = t28*t29*t39; 
    const double t23 = delta_r0+t22-t40; 
    const double t30 = delta_r0+t22; 
    const double t31 = 1.0/(t30*t30*t30); 
    const double t41 = t23*t23; 
    const double t42 = t2*t3*t8*2.0; 
    const double t43 = t3*t4*t11*2.0; 
    const double t45 = t5*t6*2.0; 
    const double t44 = t42+t43-t45; 
    const double t46 = 1.0/(t27*t27); 
    const double t47 = t3*t5*v*2.0; 
    const double t48 = t2*t6*t8*v*2.0; 
    const double t49 = t4*t6*t11*v*2.0; 
    const double t50 = t47+t48+t49; 
    const double t51 = 1.0/sqrt(t16); 
    const double t52 = 1.0/(t30*t30*t30*t30); 
    const double t53 = t3*t4*t8*v*2.0; 
    const double t55 = t2*t3*t11*v*2.0; 
    const double t54 = t53-t55; 
    jac[0] = t10*t28*t29*t31*t41*terr_dis*3.0; 
    jac[1] = t18*t28*t29*t31*t41*terr_dis*3.0; 
    jac[2] = t17*t28*t29*t31*t41*-3.0; 
    jac[3] = t31*t41*(k_r*t19*t21*t44-t28*t39*t44*t51*(1.0/2.0)+t29*t39*t46*(t6*t17+t3*t8*t10*terr_dis+t3*t11*t18*terr_dis))*3.0-k_r*t19*t21*t23*t41*t44*t52*3.0; 
    jac[4] = t31*t41*(t29*t39*t46*(-t3*t17*v+t6*t8*t10*terr_dis*v+t6*t11*t18*terr_dis*v)+k_r*t19*t21*t50-t28*t39*t50*t51*(1.0/2.0))*-3.0+k_r*t19*t21*t23*t41*t50*t52*3.0; 
    jac[5] = t31*t41*(-k_r*t19*t21*t54+t28*t39*t51*t54*(1.0/2.0)+t29*t39*t46*(t3*t10*t11*terr_dis*v-t3*t8*t18*terr_dis*v))*-3.0-k_r*t19*t21*t23*t41*t52*t54*3.0;
}

int castray(double *d_occ, double *p_occ, double *p1, double *p2, double *p3,
        const double r0[3], const double r1[3], const double v[3],
        const double pos_n_origin, const double pos_e_origin, const double terr_dis, double *terr_map) {
    
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

    // initialize
    int occ_detected = 0;

    // relative (unit) start position
    const double x0 = (r0[0] - pos_e_origin)/terr_dis;
    const double y0 = (r0[1] - pos_n_origin)/terr_dis;

    // initial height
    const double h0 = r0[2];
    
    // vector for triangle intersect inputs
    const double r0_rel[3] = {x0*terr_dis,y0*terr_dis,h0}; //XXX: this origin subtracting/adding is inefficient.. pick one and go with it for this function

    // relative end position
    const double x1 = (r1[0] - pos_e_origin)/terr_dis;
    const double y1 = (r1[1] - pos_n_origin)/terr_dis;

    // end height
    const double h1 = r1[2];

    // line deltas
    const double dx = fabs(x1 - x0);
    const double dy = fabs(y1 - y0);

    // initial cell origin
    int x = int(floor(x0));
    int y = int(floor(y0));

    // unit change in line length per x/y (see definition below)
    double dt_dx;
    double dt_dy;

    // change in height per unit line length (t)
    const double dh = h1 - h0;

    // number of cells we pass through
    int n = fabs(floor(x1)-x) + fabs(floor(y1)-y) + 1; //XXX: what is the real difference between this and using dx / dy?

    // initialize stepping criteria
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
        t_next_horizontal = (x + 1.0 - x0) * dt_dx; // remember x is "floor(x0)" here
        t_last_horizontal = (x0 - x) * dt_dx;
    }
    else {
        x_inc = -1.0;
        dt_dx = 1.0 / dx;
        t_next_horizontal = (x0 - x) * dt_dx; // remember x is "floor(x0)" here
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
        t_next_vertical = (y + 1.0 - y0) * dt_dy; // remember y is "floor(y0)" here
        t_last_vertical = (y0 - y) * dt_dy;
    }
    else {
        y_inc = -1.0;
        dt_dy = 1.0 / dy;
        t_next_vertical = (y0 - y) * dt_dy; // remember y is "floor(y0)" here
        t_last_vertical = (y + 1.0 - y0) * dt_dy;
    }

    // find cell intersection in opposite direction to initialize cell entrance
    // condition
    bool last_step_was_vert = (t_last_vertical < t_last_horizontal);

    // initialize entrance height
    double h_entr = h0;

    // for loop init
    int x_check, y_check, x_check1, y_check1, idx_corner1, idx_corner2, idx_corner3, idx_corner4, ret;
    double t, h_exit, h_check;
    bool take_vert_step, check1, check2, check3, check4;

    int i;
    for (i = 0; i < n; i=i+1) {

        // check the next step we will take and compute the exit height
        if (t_next_vertical < t_next_horizontal) {
            // next step is vertical
            take_vert_step = true;
            t = t_next_vertical; // current step
            t_next_vertical = t_next_vertical + dt_dy;
        }
        else {
            // next step is horizontal
            take_vert_step = false;
            t = t_next_horizontal; // current step
            t_next_horizontal = t_next_horizontal + dt_dx;
        }

        // take minimum of entrance and exit height for check
        // TODO: should be a way to get rid of this if statement by looking at dh outside for loop...
        h_exit = h0 + dh * t;
        if (dh > 0.0) {
            h_check = h_entr;
        }
        else {
            h_check = h_exit;
        }
        h_entr = h_exit;
        
        // bound corner coordinates
        x_check = constrain_int(x, 0, LEN_IDX_E_1);
        y_check = constrain_int(y, 0, LEN_IDX_N_1);
        x_check1 = constrain_int(x_check+1, 0, LEN_IDX_E_1);
        y_check1 = constrain_int(y_check+1, 0, LEN_IDX_N_1);
        // convert to row-major indices
        idx_corner1 = y_check*LEN_IDX_E + x_check;
        idx_corner2 = y_check1*LEN_IDX_E + x_check;
        idx_corner3 = y_check1*LEN_IDX_E + x_check1;
        idx_corner4 = y_check*LEN_IDX_E + x_check1;
        // check the four corners
        check1 = terr_map[idx_corner1] > h_check; // corner 1 (bottom left)
        check2 = terr_map[idx_corner2] > h_check; // corner 2 (top left)
        check3 = terr_map[idx_corner3] > h_check; // corner 3 (top right)
        check4 = terr_map[idx_corner4] > h_check; // corner 4 (bottom right) 

        // check cell triangles
        if (last_step_was_vert) { // / / / / / / / / / / / / / / / / / / / / /
            // vertical entrance step

            if (take_vert_step) {
                // next step is vertical

                if (y_inc > 0) { //TODO: should be able to get rid of a few of these ifs by making the decision outside the for loop...
                    // BR, TL

                    // check bottom-right triangle corners
                    if (check1 || check4 || check3) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*(x+1);
                        p2[1] = terr_dis*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret; // =1 if detection
                    }

                    // check top-left triangle corners
                    if ((check1 || check2 || check3) && (occ_detected==0)) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*x;
                        p2[1] = terr_dis*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret*2; // =2 if detection
                    }
                }
                else {
                    // TL, BR

                    // check top-left triangle corners
                    if (check1 || check2 || check3) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*x;
                        p2[1] = terr_dis*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret*2; // =2 if detection
                    }

                    // check bottom-right triangle corners
                    if ((check1 || check4 || check3) && occ_detected==0) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*(x+1);
                        p2[1] = terr_dis*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret; // =1 if detection
                    }
                }
            }   
            else  {// - - - - - - - - - - - - - - - - - - - - - - - - - - -
                // next step is horizontal

                if (y_inc > 0 && x_inc > 0) {
                    // BR

                    // check bottom-right triangle corners
                    if (check1 || check4 || check3) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*(x+1);
                        p2[1] = terr_dis*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret; // =1 if detection
                    }
                }
                else if (y_inc < 0 && x_inc < 0) {
                    // TL

                    // check top-left triangle corners
                    if (check1 || check2 || check3) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*x;
                        p2[1] = terr_dis*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret*2; // =2 if detection
                    }
                }
                else {
                    // BR, TL

                    // check bottom-right triangle corners
                    if (check1 || check4 || check3) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*(x+1);
                        p2[1] = terr_dis*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret; // =1 if detection
                    }

                    // check top-left triangle corners
                    if ((check1 || check2 || check3) && (occ_detected==0)) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*x;
                        p2[1] = terr_dis*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret*2; // =2 if detection
                    }
                }
            }
        }
        else { // last step was horizontal / / / / / / / / / / / / / / / / / / 
            if (take_vert_step) {
                // next step is vertical

                if (x_inc > 0) {
                    // TL

                    // check top-left triangle corners
                    if (check1 || check2 || check3) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*x;
                        p2[1] = terr_dis*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret*2; // =2 if detection
                    }

                    if ((y_inc < 0) && (occ_detected==0)) {
                        // BR

                        // check bottom-right triangle corners
                        if (check1 || check4 || check3) {

                            // set 3 corners
                            p1[0] = terr_dis*x;
                            p1[1] = terr_dis*y;
                            p1[2] = terr_map[idx_corner1];
                            p2[0] = terr_dis*(x+1);
                            p2[1] = terr_dis*y;
                            p2[2] = terr_map[idx_corner4];
                            p3[0] = terr_dis*(x+1);
                            p3[1] = terr_dis*(y+1);
                            p3[2] = terr_map[idx_corner3];

                            // check for ray-triangle intersection
                            ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                            occ_detected += ret; // =1 if detection
                        }
                    }
                }
                else {
                    // BR

                    // check bottom-right triangle corners
                    if (check1 || check4 || check3) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*(x+1);
                        p2[1] = terr_dis*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret; // =1 if detection
                    }

                    if ((y > 0) && (occ_detected==0)) {
                        // TL

                        // check top-left triangle corners
                        if (check1 || check2 || check3) {

                            // set 3 corners
                            p1[0] = terr_dis*x;
                            p1[1] = terr_dis*y;
                            p1[2] = terr_map[idx_corner1];
                            p2[0] = terr_dis*x;
                            p2[1] = terr_dis*(y+1);
                            p2[2] = terr_map[idx_corner2];
                            p3[0] = terr_dis*(x+1);
                            p3[1] = terr_dis*(y+1);
                            p3[2] = terr_map[idx_corner3];

                            // check for ray-triangle intersection
                            ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                            occ_detected += ret*2; // =2 if detection
                        }
                    }
                }
            }
            else { // - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
                // next step is horizontal

                if (x_inc > 0) {
                    // TL, BR

                    // check top-left triangle corners
                    if (check1 || check2 || check3) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*x;
                        p2[1] = terr_dis*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret*2; // =2 if detection
                    }

                    // check bottom-right triangle corners
                    if ((check1 || check4 || check3) && (occ_detected==0)) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*(x+1);
                        p2[1] = terr_dis*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret; // =1 if detection
                    }
                }
                else {
                    // BR, TL

                    // check bottom-right triangle corners
                    if (check1 || check4 || check3) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*(x+1);
                        p2[1] = terr_dis*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret; // =1 if detection
                    }

                    // check top-left triangle corners
                    if ((check1 || check2 || check3) && (occ_detected==0)) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*x;
                        p2[1] = terr_dis*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0_rel, v, p1, p2, p3);

                        occ_detected += ret*2; // =2 if detection
                    }
                }
            }
        } // / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

        // return if occlusion detected
        if (occ_detected > 0) {
            return occ_detected;
        }

        // actually take the step
        if (take_vert_step) { // (t_next_vertical < t_next_horizontal)
            // take a vertical step
            y = y + y_inc;
        }
        else {
            // take a horizontal step
            x = x + x_inc;
        }
        last_step_was_vert = take_vert_step;
    }
    return occ_detected;
}

int intersect_triangle(double *d_occ, double *p_occ,
        const double r0[3], const double v_ray[3],
        const double p1[3], const double p2[3], const double p3[3]) {
    /* following: "Fast, Minimum Storage Ray/Triangle Intersection",
     * Moeller et. al., Journal of Graphics Tools, Vol.2(1), 1997
     */

    // NOTE: all vectors in here are E,N,U
    
    // find vectors for two edges sharing p1
    const double e1[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
    const double e2[3] = {p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]};

    // begin calculating determinant - also used to calculate U parameter
    double pvec[3];
    cross(pvec, v_ray, e2);

    // we don't test for backwards culling here as, assuming this ray casting
    // algorithm is properly detecting the first occluding triangles, we should
    // not run into a case of a true "backwards" facing triangle (also the
    // current BR and TL definitions have opposite vertex spin definitions..
    // should probably change that in the future.. could maybe avoid a few
    // divisions of the determinant)

    // if the determinant is near zero, ray lies in the triangle plane
    const double det = dot(e1, pvec);
    if (det > -EPSILON && det < EPSILON) {
        return 0;
    }
    
    // divide the determinant (XXX: could possibly find a way to avoid this until the last minute possible..)
    const double inv_det = 1.0 / det;

    // calculate distance from p1 to ray origin
    const double tvec[3] = {r0[0] - p1[0], r0[1] - p1[1], r0[2] - p1[2]};

    // calculate u parameter and test bounds
    const double u = dot(tvec, pvec) * inv_det;
    if (u < 0.0 || u > 1.0) {
        return 0;
    }
    
    // prepare to test v parameter
    double qvec[3];
    cross(qvec, tvec, e1);

    // calculate v parameter and test bounds
    const double v = dot(v_ray, qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0) {
        return 0;
    }

    // calculate d_occ, scale parameters, ray intersects triangle
    *d_occ = dot(e2, qvec) * inv_det;

    // calculate and return intersection point
    const double one_u_v = (1 - u - v);
    p_occ[0] = one_u_v * p1[0] + u * p2[0] + v * p3[0];
    p_occ[1] = one_u_v * p1[1] + u * p2[1] + v * p3[1];
    p_occ[2] = one_u_v * p1[2] + u * p2[2] + v * p3[2];
    
    return 1;
}
    