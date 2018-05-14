#include "acado_common.h"
#include <math.h>
#include <string.h>

void lsq_obj_eval( real_t *in, real_t *out ){
    
    // closest point on track
    double dot_vp_br = cos(in[16])*cos(in[15])*(in[0]-in[12]) + sin(in[16])*cos(in[15])*(in[1]-in[13]) - sin(in[15])*(in[2]-in[14]);
    double pn = in[12] + dot_vp_br*cos(in[16])*cos(in[15]);
    double pe = in[13] + dot_vp_br*sin(in[16])*cos(in[15]);
    double pd = in[14] - dot_vp_br*sin(in[15]);
 
    // state output
    out[0] = (in[0]-pn)*sin(in[16]) - (in[1]-pe)*cos(in[16]); // e_lat
    out[1] = -(pd-in[2]); // e_lon

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

    // closest point on track
    double dot_vp_br = cos(in[16])*cos(in[15])*(in[0]-in[12]) + sin(in[16])*cos(in[15])*(in[1]-in[13]) - sin(in[15])*(in[2]-in[14]);
    double pn = in[12] + dot_vp_br*cos(in[16])*cos(in[15]);
    double pe = in[13] + dot_vp_br*sin(in[16])*cos(in[15]);
    double pd = in[14] - dot_vp_br*sin(in[15]);

    // state output
    out[0] = (in[0]-pn)*sin(in[16]) - (in[1]-pe)*cos(in[16]); // e_lat
    out[1] = -(pd-in[2]); // e_lon

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
