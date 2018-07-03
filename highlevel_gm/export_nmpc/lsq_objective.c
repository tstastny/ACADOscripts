#include "acado_common.h"
#include <math.h>
#include <string.h>

void lsq_obj_eval( real_t *in, real_t *out ){
    
    // path tangent unit vector 
    const double tP_n_bar = cos(in[16]);
    const double tP_e_bar = sin(in[16]);
    
    // "closest" point on track
    const double tp_dot_br = tP_n_bar*(in[0]-in[17]) + tP_e_bar*(in[1]-in[21]);
    const double tp_dot_br_n = tp_dot_br*tP_n_bar;
    const double tp_dot_br_e = tp_dot_br*tP_e_bar;
    const double p_n = in[17] + tp_dot_br_n;
    const double p_e = in[21] + tp_dot_br_e;
    const double p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar;
    const double p_d = in[41] - p_lat*tan(in[15]);
    
    // lateral-directional guidance
    const double e_lat = (in[1]-p_e)*cos(in[16]) - (in[0]-p_n)*sin(in[16]);
    
    // longitudinal guidance
    const double e_lon = p_d - in[2];
    
    // state output
    out[0] = e_lat;
    out[1] = e_lon;

    // control output
    out[2] = in[6]; // gamma ref
    out[3] = in[7]; // mu ref
    out[4] = (in[6] - in[3])/1; // gamma dot
    out[5] = (in[7] - in[5])/0.7; // mu dot

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
    const double tP_n_bar = cos(in[16]);
    const double tP_e_bar = sin(in[16]);
    
    // "closest" point on track
    const double tp_dot_br = tP_n_bar*(in[0]-in[17]) + tP_e_bar*(in[1]-in[21]);
    const double tp_dot_br_n = tp_dot_br*tP_n_bar;
    const double tp_dot_br_e = tp_dot_br*tP_e_bar;
    const double p_n = in[17] + tp_dot_br_n;
    const double p_e = in[21] + tp_dot_br_e;
    const double p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar;
    const double p_d = in[41] - p_lat*tan(in[15]);
    
    // lateral-directional guidance
    const double e_lat = (in[1]-p_e)*cos(in[16]) - (in[0]-p_n)*sin(in[16]);
    
    // longitudinal guidance
    const double e_lon = p_d - in[2];
    
    // state output
    out[0] = e_lat;
    out[1] = e_lon;

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
