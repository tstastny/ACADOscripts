#include "acado_common.h"
#include <math.h>
#include <string.h>

void rhs( const real_t *in, real_t *out ){

/* optimized intermediate calculations */

const double t2 = in[0]-in[12];
const double t3 = in[1]-in[13];
const double t4 = t2*t2;
const double t5 = t3*t3;
const double t6 = t4+t5;
const double t7 = sqrt(t6);
const double t8 = t7+1.0/1.0E2;
const double t9 = 1.0/t8;
const double t11 = cos(in[4]);
const double t17 = cos(in[5]);
const double t18 = in[10]*t11*t17;
const double t10 = in[20]+t18;
const double t15 = sin(in[5]);
const double t16 = in[10]*t11*t15;
const double t12 = in[21]+t16;
const double t13 = sin(in[4]);
const double t19 = in[10]*t13;
const double t14 = in[22]-t19;

/* rhs */

out[0] = t10;
out[1] = t12;
out[2] = t14;
out[3] = in[6];
out[4] = in[7];
out[5] = (tan(in[3])*(9.81E2/1.0E2))/in[10];
out[6] = -(in[6]-in[8])/in[23];
out[7] = -(in[7]-in[9])/in[24];

}

void rhs_eval( real_t *in, real_t *out ){

/* optimized intermediate calculations */

const double t2 = in[0]-in[12];
const double t3 = in[1]-in[13];
const double t4 = t2*t2;
const double t5 = t3*t3;
const double t6 = t4+t5;
const double t7 = sqrt(t6);
const double t8 = t7+1.0/1.0E2;
const double t9 = 1.0/t8;
const double t11 = cos(in[4]);
const double t17 = cos(in[5]);
const double t18 = in[10]*t11*t17;
const double t10 = in[20]+t18;
const double t15 = sin(in[5]);
const double t16 = in[10]*t11*t15;
const double t12 = in[21]+t16;
const double t13 = sin(in[4]);
const double t19 = in[10]*t13;
const double t14 = in[22]-t19;

/* rhs */

out[0] = t10;
out[1] = t12;
out[2] = t14;
out[3] = in[6];
out[4] = in[7];
out[5] = (tan(in[3])*(9.81E2/1.0E2))/in[10];
out[6] = -(in[6]-in[8])/in[23];
out[7] = -(in[7]-in[9])/in[24];

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

/* optimized intermediate calculations */

const double t2 = in[0]-in[12];
const double t3 = in[1]-in[13];
const double t4 = t2*t2;
const double t5 = t3*t3;
const double t6 = t4+t5;
const double t7 = sqrt(t6);
const double t8 = t7+1.0/1.0E2;
const double t9 = 1.0/t8;
const double t11 = cos(in[4]);
const double t17 = cos(in[5]);
const double t18 = in[10]*t11*t17;
const double t10 = in[20]+t18;
const double t15 = sin(in[5]);
const double t16 = in[10]*t11*t15;
const double t12 = in[21]+t16;
const double t13 = sin(in[4]);
const double t14 = in[22]-in[10]*t13;

const double cp_n_unit = t2*t9;
const double cp_e_unit = t3*t9;

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// variable definitions
const double pparam_cc_d = in[14];
const double pparam_R = in[15];
const double pparam_ldir = in[16];
const double pparam_gam_sp = in[17];
const double pparam_xi0 = in[18];
const double pparam_dxi = in[19];

// spiral angular position: [0,2*pi)
const double xi_sp = atan2(cp_e_unit, cp_n_unit);
double delta_xi = xi_sp-pparam_xi0;

if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {
    
    delta_xi = delta_xi + 6.28318530718;

} else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {
    
    delta_xi = delta_xi - 6.28318530718;

}

// closest point on nearest spiral leg and tangent down component
double d_d;
double Td_d;
double Gamma_d;

if (fabs(pparam_gam_sp) < 0.001) {

    d_d = pparam_cc_d;
    Td_d = 0.0;
    Gamma_d = 0.0;

} else {

    const double Rtangam = pparam_R * tan(pparam_gam_sp);

    // spiral height delta for current angle
    const double delta_d_xi = -delta_xi * pparam_ldir * Rtangam;

    // end spiral altitude change
    const double delta_d_sp_end = -pparam_dxi * Rtangam;

    // nearest spiral leg
    double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
    const double delta_d_end_k  = round( (delta_d_sp_end - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

    // check
    if (delta_d_k * pparam_gam_sp > 0.0) { //NOTE: gam is actually being used for its sign, but writing a sign operator doesnt make a difference here

        delta_d_k = 0.0;
    
    } else if (fabs(delta_d_k) > fabs(delta_d_end_k) ) {
    
        delta_d_k = (delta_d_k < 0.0) ? -fabs(delta_d_end_k) : fabs(delta_d_end_k);
    
    }

    // closest point on nearest spiral leg
    const double delta_d_sp = delta_d_k + delta_d_xi;
    d_d = pparam_cc_d + delta_d_sp;
    Td_d = -asin(pparam_gam_sp);
    Gamma_d = pparam_gam_sp;
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double Td_n = -cp_e_unit*in[16];
const double Td_e = cp_n_unit*in[16];

double V_g = sqrt(t10*t10+t12*t12+t14*t14);
if (V_g < 0.01) V_g = 0.01;

double sin_d_dot_V_g = t14*1.0/V_g;
if (sin_d_dot_V_g > 1.0) {
    sin_d_dot_V_g = 1.0;
} else if (sin_d_dot_V_g < -1.0) {
    sin_d_dot_V_g = -1.0;
}

const double atan2_01 = atan2(Td_e, Td_n);
const double atan2_02 = atan2(t12, t10);
double e_chi = atan2_01-atan2_02;
if (e_chi>3.14159265359) e_chi = e_chi - 6.28318530718;
if (e_chi<-3.14159265359) e_chi = e_chi + 6.28318530718;

const double t20 = d_d-in[2];
const double t24 = cp_e_unit*in[15];
const double t25 = -in[1]+in[13]+t24;
const double t19 = Td_e*t20-Td_d*t25;
const double t22 = cp_n_unit*in[15];
const double t23 = -in[0]+in[12]+t22;
const double t21 = -Td_d*t23+Td_n*t20;
const double t26 = Td_e*t23-Td_n*t25;

/* outputs */

out[0] = sqrt(t19*t19+t21*t21+t26*t26);
out[1] = Gamma_d+asin(sin_d_dot_V_g);
out[2] = e_chi;
out[3] = in[6];
out[4] = in[7];
out[5] = in[8];
out[6] = in[9];

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

const double t2 = in[0]-in[12];
const double t3 = in[1]-in[13];
const double t4 = t2*t2;
const double t5 = t3*t3;
const double t6 = t4+t5;
const double t7 = sqrt(t6);
const double t8 = t7+1.0/1.0E2;
const double t9 = 1.0/t8;
const double t11 = cos(in[4]);
const double t17 = cos(in[5]);
const double t18 = in[10]*t11*t17;
const double t10 = in[20]+t18;
const double t15 = sin(in[5]);
const double t16 = in[10]*t11*t15;
const double t12 = in[21]+t16;
const double t13 = sin(in[4]);
const double t14 = in[22]-in[10]*t13;

const double cp_n_unit = t2*t9;
const double cp_e_unit = t3*t9;

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// variable definitions
const double pparam_cc_d = in[14];
const double pparam_R = in[15];
const double pparam_ldir = in[16];
const double pparam_gam_sp = in[17];
const double pparam_xi0 = in[18];
const double pparam_dxi = in[19];

// spiral angular position: [0,2*pi)
const double xi_sp = atan2(cp_e_unit, cp_n_unit);
double delta_xi = xi_sp-pparam_xi0;

if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {
    
    delta_xi = delta_xi + 6.28318530718;

} else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {
    
    delta_xi = delta_xi - 6.28318530718;

}

// closest point on nearest spiral leg and tangent down component
double d_d;
double Td_d;

if (fabs(pparam_gam_sp) < 0.001) {

    d_d = pparam_cc_d;
    Td_d = 0.0;

} else {

    const double Rtangam = pparam_R * tan(pparam_gam_sp);

    // spiral height delta for current angle
    const double delta_d_xi = -delta_xi * pparam_ldir * Rtangam;

    // end spiral altitude change
    const double delta_d_sp_end = -pparam_dxi * Rtangam;

    // nearest spiral leg
    double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
    const double delta_d_end_k  = round( (delta_d_sp_end - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

    // check
    if (delta_d_k * pparam_gam_sp > 0.0) { //NOTE: gam is actually being used for its sign, but writing a sign operator doesnt make a difference here

        delta_d_k = 0.0;
    
    } else if (fabs(delta_d_k) > fabs(delta_d_end_k) ) {
    
        delta_d_k = (delta_d_k < 0.0) ? -fabs(delta_d_end_k) : fabs(delta_d_end_k);
    
    }

    // closest point on nearest spiral leg
    const double delta_d_sp = delta_d_k + delta_d_xi;
    d_d = pparam_cc_d + delta_d_sp;
    Td_d = -asin(pparam_gam_sp);
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double Td_n = -cp_e_unit*in[16];
const double Td_e = cp_n_unit*in[16];

const double t20 = d_d-in[2];
const double t24 = cp_e_unit*in[15];
const double t25 = -in[1]+in[13]+t24;
const double t19 = Td_e*t20-Td_d*t25;
const double t22 = cp_n_unit*in[15];
const double t23 = -in[0]+in[12]+t22;
const double t21 = -Td_d*t23+Td_n*t20;
const double t26 = Td_e*t23-Td_n*t25;

/* outputs */

out[0] = sqrt(t19*t19+t21*t21+t26*t26);

}

void evaluateLSQEndTerm( const real_t *in, real_t *out ){

double in_Delta[ACADO_NX+ACADO_NU+ACADO_NOD];
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
