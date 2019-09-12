#include "acado_common.h"
#include <math.h>
#include <string.h>

// #define M_PI_2 1.570796326794897
// #define M_PI 3.141592653589793
// #define M_2_PI 6.283185307179586
#define TWO_OVER_PI 0.636619772367581

// lookup table constants
#define LEN_IDX_N 141
#define LEN_IDX_E 141
#define LEN_IDX_N_1 140
#define LEN_IDX_E_1 140
#define ONE_DIS 1.0

// longitudinal guidance constants
#define VD_SINK 1.5
#define VD_CLMB -3.5
#define VD_EPS 0.01

// // bearing feasibility constants
// #define LAMBDA_CO 0.02                          // linear finite cut-off angle [rad] (= approx. 1 deg)
// #define ONE_OVER_S_LAMBDA_CO 50.003333488895450 // 1/sin(lambda_co)
// #define M_CO 2499.833309998360                  // linear finite cut-off slope = cos(lambda_co)/sin(lambda_co)^2
    
void lookup_terrain_idx( const double pos_n, const double pos_e, const double pos_n_origin,
        const double pos_e_origin, int *idx_q, double *dh);

// double bearing_feas(double lambda, const double wind_ratio, const double wind_ratio_buf);

void lsq_obj_eval( real_t *in, real_t *out )
{
    
    /* PATH CALCULATIONS */
    
    // path tangent unit vector 
    const double tP_n_bar = cos(in[16]);
    const double tP_e_bar = sin(in[16]);
    
    // "closest" point on track
    const double tp_dot_br = tP_n_bar*(in[0]-in[12]) + tP_e_bar*(in[1]-in[13]);
    const double tp_dot_br_n = tp_dot_br*tP_n_bar;
    const double tp_dot_br_e = tp_dot_br*tP_e_bar;
    const double p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar;
    const double p_d = in[14] - p_lat*tan(in[15]);
    
    /* DIRECTIONAL GUIDANCE */
    
    // lateral-directional error
    const double e_lat = ((in[0]-in[12])*tP_e_bar - (in[1]-in[13])*tP_n_bar);
    
    // ground speed
    const double v_c_gamma = in[8]*cos(in[3]);
    const double vG_n = v_c_gamma*cos(in[4]) + in[9];
    const double vG_e = v_c_gamma*sin(in[4]) + in[10];
    const double vG_d = -in[8]*sin(in[3]) + in[11];
    const double norm_vg_lat2 = vG_n*vG_n + vG_e*vG_e;
    const double norm_vg_lat = sqrt(norm_vg_lat2);
    
    // lateral-directional track-error boundary
    double e_b_lat;
    if (norm_vg_lat > 1.0) {
        e_b_lat = norm_vg_lat*in[17];                               
    } else {
        e_b_lat = 0.5*in[17]*(norm_vg_lat2 + 1.0);
    }
    
    // lateral-directional setpoint
    double normalized_e_lat = e_lat/e_b_lat;
    const double chi_sp = in[16] + atan(M_2_PI * normalized_e_lat);
    
    // lateral-directional error
    double chi_err = chi_sp - atan2(vG_e,vG_n);
    if (chi_err > M_PI) chi_err = chi_err - M_2_PI;
    if (chi_err < -M_PI) chi_err = chi_err + M_2_PI;
    
    /* LONGITUDINAL GUIDANCE */

    // longitudinal track-error
    const double e_lon = p_d-in[2];
    
    // normalized lateral-directional track-error
    normalized_e_lat = fabs(normalized_e_lat);
    if (normalized_e_lat > 1.0) normalized_e_lat = 1.0;
    
    // smooth track proximity factor
    double track_prox = cos(M_PI_2 * normalized_e_lat);
    track_prox = track_prox * track_prox;
    
    // path down velocity setpoint
    const double vP_d = -sin(in[15]) * sqrt(norm_vg_lat2 + vG_d*vG_d) * track_prox  - in[11];
    
    // longitudinal velocity increment
    double delta_vd;
    if (e_lon < 0.0) {
        delta_vd = VD_CLMB - VD_EPS - vP_d;
    }
    else {
        delta_vd = VD_SINK + VD_EPS - vP_d;
    }
    
    // longitudinal track-error boundary
    const double e_b_lon = in[18] * delta_vd;
    const double nomralized_e_lon = fabs(e_lon/e_b_lon);
    
    // longitudinal approach velocity
    const double vsp_d_app = TWO_OVER_PI * atan(M_2_PI * nomralized_e_lon) * delta_vd;
    
    // down velocity setpoint (air-mass relative)
    const double vsp_d = vP_d + vsp_d_app;
    
    // flight path angle setpoint
    double vsp_d_over_v = vsp_d/in[8];
    if (vsp_d_over_v > 1.0) vsp_d_over_v = 1.0;
    if (vsp_d_over_v < -1.0) vsp_d_over_v = -1.0;
    const double gamma_sp = -asin(vsp_d_over_v);
       
    /* TERRAIN */
    
    // lookup 2.5d grid
    int idx_q[4];
    double dh[2];
    lookup_terrain_idx(in[0], in[1], in[20], in[21], idx_q, dh);
    
    // bi-linear interpolation
    const double h12 = (1-dh[0])*in[22+idx_q[0]] + dh[0]*in[22+idx_q[1]];
    const double h34 = (1-dh[0])*in[22+idx_q[2]] + dh[0]*in[22+idx_q[3]];
    const double h_terr = (1-dh[1])*h12 + dh[1]*h34;
    
    // soft constraint formulation
    double one_minus_h_normalized = 1.0 + (in[2] + h_terr)/in[19];
    if (one_minus_h_normalized <= 0.0) one_minus_h_normalized = 0.0;
    
    // constraint priority
    const double sig_h = (one_minus_h_normalized > 1.0) ? 1.0 : cos(M_PI*one_minus_h_normalized)*0.5+0.5;
    
    // state output
    out[0] = sig_h*chi_err;
    out[1] = sig_h*(gamma_sp - in[3]);
    out[2] = one_minus_h_normalized*one_minus_h_normalized;
    
    // control output
    out[3] = in[6] - sig_h*gamma_sp;    // gamma ref
    out[4] = in[7];                     // phi ref
    out[5] = (in[6] - in[3])/1;         // gamma dot
    out[6] = (in[7] - in[5])/0.5;       // phi dot
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
    /* PATH CALCULATIONS */
    
    // path tangent unit vector 
    const double tP_n_bar = cos(in[14]);
    const double tP_e_bar = sin(in[14]);
    
    // "closest" point on track
    const double tp_dot_br = tP_n_bar*(in[0]-in[10]) + tP_e_bar*(in[1]-in[11]);
    const double tp_dot_br_n = tp_dot_br*tP_n_bar;
    const double tp_dot_br_e = tp_dot_br*tP_e_bar;
    const double p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar;
    const double p_d = in[12] - p_lat*tan(in[13]);
    
    /* DIRECTIONAL GUIDANCE */
    
    // lateral-directional error
    const double e_lat = ((in[0]-in[10])*tP_e_bar - (in[1]-in[11])*tP_n_bar);
    
    // ground speed
    const double v_c_gamma = in[6]*cos(in[3]);
    const double vG_n = v_c_gamma*cos(in[4]) + in[7];
    const double vG_e = v_c_gamma*sin(in[4]) + in[8];
    const double vG_d = -in[6]*sin(in[3]) + in[9];
    const double norm_vg_lat2 = vG_n*vG_n + vG_e*vG_e;
    const double norm_vg_lat = sqrt(norm_vg_lat2);
    
    // lateral-directional track-error boundary
    double e_b_lat;
    if (norm_vg_lat > 1.0) {
        e_b_lat = norm_vg_lat*in[15];                               
    } else {
        e_b_lat = 0.5*in[15]*(norm_vg_lat2 + 1.0);
    }
    
    // lateral-directional setpoint
    double normalized_e_lat = e_lat/e_b_lat;
    const double chi_sp = in[14] + atan(M_2_PI * normalized_e_lat);
    
    // lateral-directional error
    double chi_err = chi_sp - atan2(vG_e,vG_n);
    if (chi_err > M_PI) chi_err = chi_err - M_2_PI;
    if (chi_err < -M_PI) chi_err = chi_err + M_2_PI;
    
    /* LONGITUDINAL GUIDANCE */

    // longitudinal track-error
    const double e_lon = p_d-in[2];
    
    // normalized lateral-directional track-error
    normalized_e_lat = fabs(normalized_e_lat);
    if (normalized_e_lat > 1.0) normalized_e_lat = 1.0;
    
    // smooth track proximity factor
    double track_prox = cos(M_PI_2 * normalized_e_lat);
    track_prox = track_prox * track_prox;
    
    // path down velocity setpoint
    const double vP_d = -sin(in[13]) * sqrt(norm_vg_lat2 + vG_d*vG_d) * track_prox  - in[9];
    
    // longitudinal velocity increment
    double delta_vd;
    if (e_lon < 0.0) {
        delta_vd = VD_CLMB - VD_EPS - vP_d; //SOMETHING WRONG WITH DELTA_VD
    }
    else {
        delta_vd = VD_SINK + VD_EPS - vP_d;
    }
    
    // longitudinal track-error boundary
    const double e_b_lon = in[16] * delta_vd;
    const double nomralized_e_lon = fabs(e_lon/e_b_lon);
    
    // longitudinal approach velocity
    const double vsp_d_app = TWO_OVER_PI * atan(M_2_PI * nomralized_e_lon) * delta_vd;
    
    // down velocity setpoint (air-mass relative)
    const double vsp_d = vP_d + vsp_d_app;
    
    // flight path angle setpoint
    double vsp_d_over_v = vsp_d/in[6];
    if (vsp_d_over_v > 1.0) vsp_d_over_v = 1.0;
    if (vsp_d_over_v < -1.0) vsp_d_over_v = -1.0;
    const double gamma_sp = -asin(vsp_d_over_v);
       
    /* TERRAIN */
    
    // lookup 2.5d grid
    int idx_q[4];
    double dh[2];
    lookup_terrain_idx(in[0], in[1], in[18], in[19], idx_q, dh);
    
    // bi-linear interpolation
    const double h12 = (1-dh[0])*in[20+idx_q[0]] + dh[0]*in[20+idx_q[1]];
    const double h34 = (1-dh[0])*in[20+idx_q[2]] + dh[0]*in[20+idx_q[3]];
    const double h_terr = (1-dh[1])*h12 + dh[1]*h34;
    
    // soft constraint formulation
    double one_minus_h_normalized = 1.0 + (in[2] + h_terr)/in[17];
    if (one_minus_h_normalized <= 0.0) one_minus_h_normalized = 0.0;
    
    // constraint priority
    const double sig_h = (one_minus_h_normalized > 1.0) ? 1.0 : cos(M_PI*one_minus_h_normalized)*0.5+0.5;
    
    // state output
    out[0] = sig_h*chi_err;
    out[1] = sig_h*(gamma_sp - in[3]);
    out[2] = one_minus_h_normalized*one_minus_h_normalized;
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

// double bearing_feas(double lambda, const double wind_ratio, const double wind_ratio_buf)
// {
//     /* bound lambda -- angle between wind and bearing */
// 	lambda = fabs(lambda);
//     if (lambda > M_PI_2) lambda = M_PI_2;
// 
// 	/* upper and lower feasibility barriers */
// 	float wind_ratio_ub;
// 	float wind_ratio_lb;
// 	if (lambda < LAMBDA_CO) {
// 		/* linear finite cut-off */
// 		const double mx = M_CO * (LAMBDA_CO - lambda);
// 		const double wind_ratio_ub_co = ONE_OVER_S_LAMBDA_CO;
// 		wind_ratio_ub = wind_ratio_ub_co + mx;
// 		const double wind_ratio_lb_co = (ONE_OVER_S_LAMBDA_CO - 2.0) * wind_ratio_buf + 1.0;
// 		wind_ratio_lb = wind_ratio_lb_co + wind_ratio_buf * mx;
// 	}
//     else {
// 		const double one_over_s_lambda = 1.0 / sin(lambda);
// 		wind_ratio_ub = one_over_s_lambda;
// 		wind_ratio_lb = (one_over_s_lambda - 2.0) * wind_ratio_buf + 1.0;
// 	}
// 
// 	/* calculate bearing feasibility */
// 	float feas;
//     if (wind_ratio > wind_ratio_ub) {
// 		// infeasible
// 		feas = 0.0;
// 	}
//     else if (wind_ratio > wind_ratio_lb) {
// 		// partially feasible
// 		// smoothly transition from fully feasible to infeasible
//         double normalized_wind_ratio = (wind_ratio - wind_ratio_lb) / (wind_ratio_ub - wind_ratio_lb);
//         if (normalized_wind_ratio > 1.0) normalized_wind_ratio = 1.0;
// 		feas = cos(M_PI_2 * normalized_wind_ratio);
//         feas = feas * feas;
// 	}
//     else {
// 		// feasible
// 		feas = 1.0;
// 	}
// 
//     return feas;
// }

