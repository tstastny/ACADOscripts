#include "acado_common.h"
// #include <math.h>
// #include <string.h>
// #include <stdbool.h>

void rhs( const real_t *in, real_t *out ){

/* for manual input indexing ... */

const int minus_NU = 0;

/* optimized intermediate calculations */

const double alpha = -in[4]+in[7];
double Vsafe = in[3];
if (Vsafe<1.0) Vsafe=1.0;

const double t2 = alpha-in[18]+in[20];
const double t3 = 1.0/(in[20]*in[20]);
const double t4 = -alpha+in[19]+in[20];
const double t5 = cos(in[4]);
const double t6 = sin(in[4]);
const double t7 = in[11]*in[11];
const double t8 = 1.0/Vsafe;
const double t9 = Vsafe*Vsafe;
const double t10 = alpha*alpha;
const double t11 = in[11]*8.61861E1;
const double t12 = in[11]*t7*2.501023E2;
const double t20 = t7*3.05322E1;
const double t13 = t11+t12-t20;
const double t14 = alpha*5.0996;
const double t15 = t10*(-5.343)+t14+1.7E1/3.2E1;
const double t16 = t9*t15*2.38875E-1;
const double t17 = cos(alpha);
const double t18 = 1.0/t17;
const double t19 = sin(alpha);
const double t21 = t8*t13*t18*t19;
const double t22 = t16+t21;
const double t23 = cos(in[6]);
const double t24 = sin(in[6]);

/* rhs */

out[0] = in[15]+Vsafe*t5*cos(in[5]);
out[1] = in[16]+Vsafe*t5*sin(in[5]);
out[2] = in[17]-Vsafe*t6;
out[3] = t6*(-9.81E2/1.0E2)+t8*t13*(2.0E1/5.3E1)-t9*(alpha*2.5491E-1+t10*2.7337+6.4105E-2)*9.014150943396226E-2;
out[4] = -t8*(t5*(9.81E2/1.0E2)-t22*t23*(2.0E1/5.3E1));
out[5] = (t8*t22*t24*(2.0E1/5.3E1))/t5;
out[6] = in[8];
out[7] = in[9]*t23-in[10]*t24;
out[8] = in[6]*(-1.24716E1)-in[8]*7.4252+in[10]*1.0069+in[13]*1.24716E1;
out[9] = -t9*(alpha*1.9303E-1+in[7]*1.8359E-1+in[9]*4.6239E-2-in[14]*1.8359E-1-9.4955E-4);
out[10] = in[6]*5.7996-in[10]*9.5153+in[13]*1.5967;
out[11] = in[11]*(-4.143016944939305)+in[12]*4.143016944939305;

}

void rhs_eval( real_t *in, real_t *out ){

/* for manual input indexing ... */

const int minus_NU = 0;

/* optimized intermediate calculations */

const double alpha = -in[4]+in[7];
double Vsafe = in[3];
if (Vsafe<1.0) Vsafe=1.0;

const double t2 = alpha-in[18]+in[20];
const double t3 = 1.0/(in[20]*in[20]);
const double t4 = -alpha+in[19]+in[20];
const double t5 = cos(in[4]);
const double t6 = sin(in[4]);
const double t7 = in[11]*in[11];
const double t8 = 1.0/Vsafe;
const double t9 = Vsafe*Vsafe;
const double t10 = alpha*alpha;
const double t11 = in[11]*8.61861E1;
const double t12 = in[11]*t7*2.501023E2;
const double t20 = t7*3.05322E1;
const double t13 = t11+t12-t20;
const double t14 = alpha*5.0996;
const double t15 = t10*(-5.343)+t14+1.7E1/3.2E1;
const double t16 = t9*t15*2.38875E-1;
const double t17 = cos(alpha);
const double t18 = 1.0/t17;
const double t19 = sin(alpha);
const double t21 = t8*t13*t18*t19;
const double t22 = t16+t21;
const double t23 = cos(in[6]);
const double t24 = sin(in[6]);

/* rhs */

out[0] = in[15]+Vsafe*t5*cos(in[5]);
out[1] = in[16]+Vsafe*t5*sin(in[5]);
out[2] = in[17]-Vsafe*t6;
out[3] = t6*(-9.81E2/1.0E2)+t8*t13*(2.0E1/5.3E1)-t9*(alpha*2.5491E-1+t10*2.7337+6.4105E-2)*9.014150943396226E-2;
out[4] = -t8*(t5*(9.81E2/1.0E2)-t22*t23*(2.0E1/5.3E1));
out[5] = (t8*t22*t24*(2.0E1/5.3E1))/t5;
out[6] = in[8];
out[7] = in[9]*t23-in[10]*t24;
out[8] = in[6]*(-1.24716E1)-in[8]*7.4252+in[10]*1.0069+in[13]*1.24716E1;
out[9] = -t9*(alpha*1.9303E-1+in[7]*1.8359E-1+in[9]*4.6239E-2-in[14]*1.8359E-1-9.4955E-4);
out[10] = in[6]*5.7996-in[10]*9.5153+in[13]*1.5967;
out[11] = in[11]*(-4.143016944939305)+in[12]*4.143016944939305;

}

void rhs_jac( const real_t *in, real_t *out ){

/* rhs_jac */
 
double f_Delta_m[ACADO_NX];
double f_Delta_p[ACADO_NX];
double in_Delta[ACADO_NX+ACADO_NU+ACADO_NOD];
memcpy(in_Delta, in, sizeof(in_Delta));
const double Delta = 0.00001;
const double Delta2 = 2.0 * Delta;
 
int i;
int j;
for (i = 0; i < (ACADO_NX+ACADO_NU); i=i+1) {
 
    in_Delta[i] = in[i] - Delta;
    rhs_eval( in_Delta, f_Delta_m );
    in_Delta[i] = in[i] + Delta;
    rhs_eval( in_Delta, f_Delta_p );
    in_Delta[i] = in[i];
 
    for (j = 0; j < ACADO_NX; j=j+1) {
        out[j*(ACADO_NX+ACADO_NU)+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;
    }
 
}

}

void lsq_obj_eval( real_t *in, real_t *out ){

/* for manual input indexing ... */

const int minus_NU = 0;

/* optimized intermediate calculations */

const double alpha = -in[4]+in[7];
double Vsafe = in[3];
if (Vsafe<1.0) Vsafe=1.0;

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
out[2] = Vsafe;
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

/* for manual input indexing ... */

const int minus_NU = ACADO_NU;

/* optimized intermediate calculations */

const double alpha = -in[4]+in[7];
double Vsafe = in[3];
if (Vsafe<1.0) Vsafe=1.0;

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
out[2] = Vsafe;
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

