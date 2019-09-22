#include "acado_common.h"
#include <math.h>
#include <string.h>

// lookup table constants
// #define LEN_IDX_N 141
// #define LEN_IDX_E 141
// #define LEN_IDX_N_1 140
// #define LEN_IDX_E_1 140
// #define ONE_DIS 1.0
#define LEN_IDX_N 29
#define LEN_IDX_E 29
#define LEN_IDX_N_1 28
#define LEN_IDX_E_1 28
#define ONE_DIS 0.2

void lookup_terrain_idx( const double pos_n, const double pos_e, const double pos_n_origin,
        const double pos_e_origin, int *idx_q, double *dh);

void lsq_obj_eval( real_t *in, real_t *out )
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
    //const double terrain_data = in[32];
    int IDX_TERR_DATA = 32;
    
    /* INTERMEDIATE CALCULATIONS */
    
    // ground speed
    double v_cos_gamma = v*cos(gamma);
    const double vG_n = v_cos_gamma*cos(xi) + w_n;
    const double vG_e = v_cos_gamma*sin(xi) + w_e;
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
    
    // velocity error
    const double e_v_n = vP_n - vG_n;
    const double e_v_e = vP_e - vG_e;
    const double e_v_d = vP_d - vG_d;
    
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
    lookup_terrain_idx(r_n, r_e, terr_local_origin_n, terr_local_origin_e, idx_q, dh);
    
    // bi-linear interpolation
    const double h12 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[0]] + dh[0]*in[IDX_TERR_DATA+idx_q[1]];
    const double h34 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[2]] + dh[0]*in[IDX_TERR_DATA+idx_q[3]];
    const double h_terr = (1-dh[1])*h12 + dh[1]*h34;
    
    // soft constraint
    double sig_h = 0.0;
    if (-r_d < h_terr + delta_h) {
        sig_h = fabs(-r_d - h_terr - delta_h) / delta_h;
        sig_h = sig_h * sig_h * sig_h;
    }
    
    // prioritization
    const double prio_aoa = 1.0;//1.0 - ((sig_aoa > 1.0) ? 1.0 : sig_aoa);
    const double prio_h = 1.0 - ((sig_h > 1.0) ? 1.0 : sig_h);
    const double prio_aoa_h = prio_aoa * prio_h;

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
    out[8] = u_T;
    out[9] = phi_ref;
    out[10] = theta_ref;
}

void acado_evaluateLSQ( const real_t *in, real_t *out )
{
    double in_Delta[ACADO_NX+ACADO_NU+ACADO_NOD];
    memcpy(in_Delta, in, sizeof(in_Delta));
    lsq_obj_eval( in_Delta, out );

    /* lsq_obj jacobians */

    double f_Delta_m[ACADO_NY];
    double f_Delta_p[ACADO_NY];
    const double Delta = 0.00001;
    const double Delta2 = 2.0 * Delta;

    int i;
    int j;
    for (i = 0; i < ACADO_NX; i=i+1) {

        in_Delta[i] = in[i] - Delta;
        lsq_obj_eval( in_Delta, f_Delta_m );
        in_Delta[i] = in[i] + Delta;
        lsq_obj_eval( in_Delta, f_Delta_p );
        in_Delta[i] = in[i];
        
        for (j = 0; j < ACADO_NY; j=j+1) {
            out[ACADO_NY+j*ACADO_NX+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;
        }
    }

    for (i = 0; i < ACADO_NU; i=i+1) {

        in_Delta[i+ACADO_NX] = in[i+ACADO_NX] - Delta;
        lsq_obj_eval( in_Delta, f_Delta_m );
        in_Delta[i+ACADO_NX] = in[i+ACADO_NX] + Delta;
        lsq_obj_eval( in_Delta, f_Delta_p );
        in_Delta[i+ACADO_NX] = in[i+ACADO_NX];

        for (j = 0; j < ACADO_NY; j=j+1) {
            out[ACADO_NY+ACADO_NY*ACADO_NX+j*ACADO_NU+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;
        }
    }

}

void lsq_objN_eval( real_t *in, real_t *out )
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
    //const double terrain_data = in[29];
    int IDX_TERR_DATA = 29;
    
    /* INTERMEDIATE CALCULATIONS */
    
    // ground speed
    double v_cos_gamma = v*cos(gamma);
    const double vG_n = v_cos_gamma*cos(xi) + w_n;
    const double vG_e = v_cos_gamma*sin(xi) + w_e;
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
    
    // velocity error
    const double e_v_n = vP_n - vG_n;
    const double e_v_e = vP_e - vG_e;
    const double e_v_d = vP_d - vG_d;
    
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
    lookup_terrain_idx(r_n, r_e, terr_local_origin_n, terr_local_origin_e, idx_q, dh);
    
    // bi-linear interpolation
    const double h12 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[0]] + dh[0]*in[IDX_TERR_DATA+idx_q[1]];
    const double h34 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[2]] + dh[0]*in[IDX_TERR_DATA+idx_q[3]];
    const double h_terr = (1-dh[1])*h12 + dh[1]*h34;
    
    // soft constraint
    double sig_h = 0.0;
    if (-r_d < h_terr + delta_h) {
        sig_h = fabs(-r_d - h_terr - delta_h) / delta_h;
        sig_h = sig_h * sig_h * sig_h;
    }
    
    // prioritization
    const double prio_aoa = 1.0;//1.0 - ((sig_aoa > 1.0) ? 1.0 : sig_aoa);
    const double prio_h = 1.0 - ((sig_h > 1.0) ? 1.0 : sig_h);
    const double prio_aoa_h = prio_aoa * prio_h;
    
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
    lsq_objN_eval( in_Delta, out );

    /* lsq_objN jacobians */

    double f_Delta_m[ACADO_NYN];
    double f_Delta_p[ACADO_NYN];
    const double Delta = 0.00001;
    const double Delta2 = 2.0 * Delta;

    int i;
    int j;
    for (i = 0; i < ACADO_NX; i=i+1) {

        in_Delta[i] = in[i] - Delta;
        lsq_objN_eval( in_Delta, f_Delta_m );
        in_Delta[i] = in[i] + Delta;
        lsq_objN_eval( in_Delta, f_Delta_p );
        in_Delta[i] = in[i];

        for (j = 0; j < ACADO_NYN; j=j+1) {
            out[ACADO_NYN+j*ACADO_NX+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;
        }
    }
}

void lookup_terrain_idx( const double pos_n, const double pos_e, const double pos_n_origin,
        const double pos_e_origin, int *idx_q, double *dh)
{
    // relative position / indices
    const double rel_n = pos_n - pos_n_origin;
    const double rel_n_bar = rel_n * ONE_DIS;
    int idx_n = rel_n_bar;
    if (idx_n < 0) {
        idx_n = 0;
    }
    else if (idx_n > LEN_IDX_N_1) {
        idx_n = LEN_IDX_N_1;
    }
    const double rel_e = pos_e - pos_e_origin;
    const double rel_e_bar = rel_e * ONE_DIS;
    int idx_e = rel_e_bar;
    if (idx_e < 0) {
        idx_e = 0;
    }
    else if (idx_e > LEN_IDX_E_1) {
        idx_e = LEN_IDX_E_1;
    }
    
    // neighbor orientation / interpolation weights
    const double delta_n = rel_n_bar-idx_n;
    int down;
    double dh_n;
    if (delta_n<0.5) {
        down = -1;
        dh_n = 0.5 + delta_n;
    }
    else {
        down = 0;
        dh_n = delta_n - 0.5;
    }
    const double delta_e = rel_e_bar-idx_e;
    int left;
    double dh_e;
    if (delta_e<0.5) {
        left = -1;
        dh_e = 0.5 + delta_e;
    }
    else {
        left = 0;
        dh_e = delta_e - 0.5;
    }
    
    // neighbor origin (down,left)
    int q1_n = idx_n + down;
    int q1_e = idx_e + left;
    
    // neighbors (north)
    int q_n[4];
    if (q1_n >= LEN_IDX_N_1) {
        q_n[0] = LEN_IDX_N_1;
        q_n[1] = LEN_IDX_N_1;
        q_n[2] = LEN_IDX_N_1;
        q_n[3] = LEN_IDX_N_1;
    }
    else {
        q_n[0] = q1_n;
        q_n[1] = q1_n + 1;
        q_n[2] = q1_n;
        q_n[3] = q1_n + 1;
    }
    // neighbors (east)
    int q_e[4];
    if (q1_e >= LEN_IDX_N_1) {
        q_e[0] = LEN_IDX_E_1;
        q_e[1] = LEN_IDX_E_1;
        q_e[2] = LEN_IDX_E_1;
        q_e[3] = LEN_IDX_E_1;
    }
    else {
        q_e[0] = q1_e;
        q_e[1] = q1_e;
        q_e[2] = q1_e + 1;
        q_e[3] = q1_e + 1;
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
