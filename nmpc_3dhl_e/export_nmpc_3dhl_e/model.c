#include "acado_common.h"

void lsq_obj_eval( real_t *in, real_t *out ){

/* optimized intermediate calculations */

const double alpha = -in[4]+in[7];

const double t2 = alpha-in[18]+in[20];
const double t3 = 1.0/(in[20]*in[20]);
const double t4 = -alpha+in[19]+in[20];
const double t5 = in[0]-in[21];
const double t6 = in[1]-in[22];

double a_soft;
if (alpha>(in[18]-in[20])) {
    a_soft=(t2*t2)*t3;
}
else if (alpha>(in[19]+in[20])) {
    a_soft=0.0;
}
else {
    a_soft=t3*(t4*t4);
}

/* outputs */

out[0] = sqrt(t5*t5+t6*t6);
out[1] = -in[2]+in[23];
out[2] = in[3];
out[3] = in[8];
out[4] = in[9];
out[5] = in[10];
out[6] = a_soft;
out[7] = in[11]*(-4.143016944939305)+in[12]*4.143016944939305;
out[8] = in[12];
out[9] = in[13];
out[10] = in[14];

}

void evaluateLSQ( const real_t *in, real_t *out ){

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

/* optimized intermediate calculations */

const double alpha = -in[4]+in[7];

const double t2 = alpha-in[15]+in[17];
const double t3 = 1.0/(in[17]*in[17]);
const double t4 = -alpha+in[16]+in[17];
const double t5 = in[0]-in[18];
const double t6 = in[1]-in[19];

double a_soft;
if (alpha>(in[15]-in[17])) {
    a_soft=(t2*t2)*t3;
}
else if (alpha>(in[16]+in[17])) {
    a_soft=0.0;
}
else {
    a_soft=t3*(t4*t4);
}

/* outputs */

out[0] = sqrt(t5*t5+t6*t6);
out[1] = -in[2]+in[20];
out[2] = in[3];
out[3] = in[8];
out[4] = in[9];
out[5] = in[10];
out[6] = a_soft;

}

void evaluateLSQEndTerm( const real_t *in, real_t *out ){

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

