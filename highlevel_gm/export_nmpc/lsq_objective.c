#include "acado_common.h"
#include <math.h>
#include <string.h>

#define LEN_IDX_N 61
#define LEN_IDX_E 61
#define LEN_IDX_N_1 60
#define LEN_IDX_E_1 60
#define ONE_DIS 0.1
    
void lookup_terrain_idx( const double pos_n, const double pos_e, const double pos_n_origin,
        const double pos_e_origin, int *idx_q, double *dh);

void lsq_obj_eval( real_t *in, real_t *out ){
    
    // path tangent unit vector 
    const double tP_n_bar = cos(in[16]);
    const double tP_e_bar = sin(in[16]);
    
    // "closest" point on track
    const double tp_dot_br = tP_n_bar*(in[0]-in[12]) + tP_e_bar*(in[1]-in[13]);
    const double tp_dot_br_n = tp_dot_br*tP_n_bar;
    const double tp_dot_br_e = tp_dot_br*tP_e_bar;
    const double p_n = in[12] + tp_dot_br_n;
    const double p_e = in[13] + tp_dot_br_e;
    const double p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar;
    const double p_d = in[14] - p_lat*tan(in[15]);
    
    // directional error
    const double v_c_gamma = in[8]*cos(in[3]);
    const double v_n = v_c_gamma*cos(in[4]);
    const double v_e = v_c_gamma*sin(in[4]);
    const double tp_dot_vg = tP_n_bar*(v_n+in[9]) + tP_e_bar*(v_e+in[10]);
       
    // terrain
    
//     // lookup 2.5d grid
//     int idx_q[4];
//     double dh[2];
//     lookup_terrain_idx(in[0], in[1], in[18], in[19], idx_q, dh);
//     // bi-linear interpolation
//     const double h12 = (1-dh[0])*in[20+idx_q[0]] + dh[0]*in[20+idx_q[1]];
//     const double h34 = (1-dh[0])*in[20+idx_q[2]] + dh[0]*in[20+idx_q[3]];
//     const double h_terr = (1-dh[1])*h12 + dh[1]*h34;
    const double h_terr = 10;
    // soft constraint formulation
    double one_minus_h_normalized = 1.0 + (in[2] + h_terr)/in[17];
    if (one_minus_h_normalized <= 0.0) one_minus_h_normalized = 0.0;  
    
    // state output
    out[0] = (in[1]-p_e)*cos(in[16]) - (in[0]-p_n)*sin(in[16]);
    out[1] = p_d - in[2];
    out[2] = tp_dot_vg*0.5+0.5;
    out[3] = one_minus_h_normalized*one_minus_h_normalized;

    // control output
    out[4] = in[6]; // gamma ref
    out[5] = in[7]; // mu ref
    out[6] = (in[6] - in[3])/1; // gamma dot
    out[7] = (in[7] - in[5])/0.7; // mu dot

}

void acado_evaluateLSQ( const real_t *in, real_t *out ){

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

void lsq_objN_eval( real_t *in, real_t *out ){

    // path tangent unit vector 
    const double tP_n_bar = cos(in[14]);
    const double tP_e_bar = sin(in[14]);
    
    // "closest" point on track
    const double tp_dot_br = tP_n_bar*(in[0]-in[10]) + tP_e_bar*(in[1]-in[11]);
    const double tp_dot_br_n = tp_dot_br*tP_n_bar;
    const double tp_dot_br_e = tp_dot_br*tP_e_bar;
    const double p_n = in[10] + tp_dot_br_n;
    const double p_e = in[11] + tp_dot_br_e;
    const double p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar;
    const double p_d = in[12] - p_lat*tan(in[13]);
    
    // directional error
    const double v_c_gamma = in[6]*cos(in[3]);
    const double v_n = v_c_gamma*cos(in[4]);
    const double v_e = v_c_gamma*sin(in[4]);
    const double tp_dot_vg = tP_n_bar*(v_n+in[7]) + tP_e_bar*(v_e+in[8]);
    
    // terrain
    
//     // lookup 2.5d grid
//     int idx_q[4];
//     double dh[2];
//     lookup_terrain_idx(in[0], in[1], in[16], in[17], idx_q, dh);
//     // bi-linear interpolation
//     const double h12 = (1-dh[0])*in[18+idx_q[0]] + dh[0]*in[18+idx_q[1]];
//     const double h34 = (1-dh[0])*in[18+idx_q[2]] + dh[0]*in[18+idx_q[3]];
//     const double h_terr = (1-dh[1])*h12 + dh[1]*h34;
    const double h_terr = 10;
    // soft constraint formulation
    double one_minus_h_normalized = 1.0 + (in[2] + h_terr)/in[15];
    if (one_minus_h_normalized <= 0.0) one_minus_h_normalized = 0.0;  
       
    // state output
    out[0] = (in[1]-p_e)*cos(in[14]) - (in[0]-p_n)*sin(in[14]);
    out[1] = p_d - in[2];
    out[2] = tp_dot_vg*0.5+0.5;
    out[3] = one_minus_h_normalized*one_minus_h_normalized;

}

void acado_evaluateLSQEndTerm( const real_t *in, real_t *out ){

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
        const double pos_e_origin, int *idx_q, double *dh) {

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

