#include "acado_common.h"
#include <math.h>
#include <string.h>

void rhs( const real_t *in, real_t *out ){

/* optimized intermediate calculations */

const double t3 = cos(in[4]);
const double t9 = cos(in[5]);
const double t10 = in[10]*t3*t9;
const double t2 = in[20]+t10;
const double t7 = sin(in[5]);
const double t8 = in[10]*t3*t7;
const double t4 = in[21]+t8;
const double t5 = sin(in[4]);
const double t11 = in[10]*t5;
const double t6 = in[22]-t11;

/* rhs */

out[0] = t2;
out[1] = t4;
out[2] = t6;
out[3] = in[6];
out[4] = in[7];
out[5] = (tan(in[3])*(9.81E2/1.0E2))/in[10];
out[6] = -in[25]*(in[6]+in[23]*(in[3]-in[8]));
out[7] = -in[26]*(in[7]+in[24]*(in[4]-in[9]));

}

void rhs_eval( real_t *in, real_t *out ){

/* optimized intermediate calculations */

const double t3 = cos(in[4]);
const double t9 = cos(in[5]);
const double t10 = in[10]*t3*t9;
const double t2 = in[20]+t10;
const double t7 = sin(in[5]);
const double t8 = in[10]*t3*t7;
const double t4 = in[21]+t8;
const double t5 = sin(in[4]);
const double t11 = in[10]*t5;
const double t6 = in[22]-t11;

/* rhs */

out[0] = t2;
out[1] = t4;
out[2] = t6;
out[3] = in[6];
out[4] = in[7];
out[5] = (tan(in[3])*(9.81E2/1.0E2))/in[10];
out[6] = -in[25]*(in[6]+in[23]*(in[3]-in[8]));
out[7] = -in[26]*(in[7]+in[24]*(in[4]-in[9]));

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

const double t3 = cos(in[4]);
const double t9 = cos(in[5]);
const double t10 = in[10]*t3*t9;
const double t2 = in[20]+t10;
const double t7 = sin(in[5]);
const double t8 = in[10]*t3*t7;
const double t4 = in[21]+t8;
const double t5 = sin(in[4]);
const double t6 = in[22]-in[10]*t5;

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

double d_n;
double d_e;
double d_d;
double Td_n;
double Td_e;
double Td_d;
double Gamma_d;

const double pparam_type = in[11];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[12];
    const double pparam_aa_e = in[13];
    const double pparam_aa_d = in[14];
    const double pparam_bb_n = in[15];
    const double pparam_bb_e = in[16];
    const double pparam_bb_d = in[17];

    // calculate vector from waypoint a to b
    const double abn = pparam_bb_n - pparam_aa_n;
    const double abe = pparam_bb_e - pparam_aa_e;
    const double abd = pparam_bb_d - pparam_aa_d;
    const double norm_ab = sqrt(abn*abn + abe*abe + abd*abd);

    // calculate tangent
    Td_n = abn / (norm_ab + 0.01);
    Td_e = abe / (norm_ab + 0.01);
    Td_d = abd / (norm_ab + 0.01);

    // point on track
    d_n = pparam_aa_n;
    d_e = pparam_aa_e;
    d_d = pparam_aa_d;
    
    // desired ground-relative flight path angle
    if ( Td_d > 1.0 ) Td_d = 1.0;
    if ( Td_d < -1.0 ) Td_d = -1.0;
    Gamma_d = -asin(Td_d);

// CURVE SEGMENT
} else if ( pparam_type < 1.5 ) {

    // variable definitions
    const double pparam_cc_n = in[12];
    const double pparam_cc_e = in[13];
    const double pparam_cc_d = in[14];
    const double pparam_R = in[15];
    const double pparam_ldir = in[16];
    const double pparam_gam_sp = in[17];
    const double pparam_xi0 = in[18];
    const double pparam_dxi = in[19];

    // calculate closest point on loiter circle
    const double cp_n = in[0] - pparam_cc_n;
    const double cp_e = in[1] - pparam_cc_e;
    const double norm_cp = sqrt( cp_n*cp_n + cp_e*cp_e );
    const double cp_n_unit = cp_n / (norm_cp + 0.01);
    const double cp_e_unit = cp_e / (norm_cp + 0.01);
    d_n = pparam_R * cp_n_unit + pparam_cc_n;
    d_e = pparam_R * cp_e_unit + pparam_cc_e;

    // calculate tangent
    Td_n = pparam_ldir * -cp_e_unit;
    Td_e = pparam_ldir * cp_n_unit;

    // spiral angular position: [0,2*pi)
    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
    double delta_xi = xi_sp-pparam_xi0;

    // closest point on nearest spiral leg and tangent down component
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

        delta_xi = delta_xi + 6.28318530718;

    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

        delta_xi = delta_xi - 6.28318530718;

    }

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
        Td_d = -sin(pparam_gam_sp);
        Gamma_d = pparam_gam_sp;
    }
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

double V_g = sqrt(t2*t2+t4*t4+t6*t6);
if (V_g < 0.01) V_g = 0.01;

double sin_d_dot_V_g = t6*1.0/V_g;
if (sin_d_dot_V_g > 1.0) sin_d_dot_V_g = 1.0;
if (sin_d_dot_V_g < -1.0) sin_d_dot_V_g = -1.0;

const double atan2_01 = atan2(Td_e, Td_n);
const double atan2_02 = atan2(t4, t2);
double e_chi = atan2_01-atan2_02;
if (e_chi>3.14159265359) e_chi = e_chi - 6.28318530718;
if (e_chi<-3.14159265359) e_chi = e_chi + 6.28318530718;

const double t12 = d_d-in[2];
const double t14 = d_e-in[1];
const double t11 = Td_e*t12-Td_d*t14;
const double t15 = d_n-in[0];
const double t13 = -Td_d*t15+Td_n*t12;
const double t16 = Td_e*t15-Td_n*t14;

/* outputs */

out[0] = sqrt(t11*t11+t13*t13+t16*t16);
out[1] = in[6];
out[2] = in[7];
out[3] = Gamma_d+asin(sin_d_dot_V_g);
out[4] = e_chi;
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

const double t3 = cos(in[4]);
const double t9 = cos(in[5]);
const double t10 = in[10]*t3*t9;
const double t2 = in[20]+t10;
const double t7 = sin(in[5]);
const double t8 = in[10]*t3*t7;
const double t4 = in[21]+t8;
const double t5 = sin(in[4]);
const double t6 = in[22]-in[10]*t5;

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

double d_n;
double d_e;
double d_d;
double Td_n;
double Td_e;
double Td_d;

const double pparam_type = in[11];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[12];
    const double pparam_aa_e = in[13];
    const double pparam_aa_d = in[14];
    const double pparam_bb_n = in[15];
    const double pparam_bb_e = in[16];
    const double pparam_bb_d = in[17];

    // calculate vector from waypoint a to b
    const double abn = pparam_bb_n - pparam_aa_n;
    const double abe = pparam_bb_e - pparam_aa_e;
    const double abd = pparam_bb_d - pparam_aa_d;
    const double norm_ab = sqrt(abn*abn + abe*abe + abd*abd);

    // calculate tangent
    Td_n = abn / (norm_ab + 0.01);
    Td_e = abe / (norm_ab + 0.01);
    Td_d = abd / (norm_ab + 0.01);

    // point on track
    d_n = pparam_aa_n;
    d_e = pparam_aa_e;
    d_d = pparam_aa_d;
    
// CURVE SEGMENT
} else if ( pparam_type < 1.5 ) {

    // variable definitions
    const double pparam_cc_n = in[12];
    const double pparam_cc_e = in[13];
    const double pparam_cc_d = in[14];
    const double pparam_R = in[15];
    const double pparam_ldir = in[16];
    const double pparam_gam_sp = in[17];
    const double pparam_xi0 = in[18];
    const double pparam_dxi = in[19];

    // calculate closest point on loiter circle
    const double cp_n = in[0] - pparam_cc_n;
    const double cp_e = in[1] - pparam_cc_e;
    const double norm_cp = sqrt( cp_n*cp_n + cp_e*cp_e );
    const double cp_n_unit = cp_n / (norm_cp + 0.01);
    const double cp_e_unit = cp_e / (norm_cp + 0.01);
    d_n = pparam_R * cp_n_unit + pparam_cc_n;
    d_e = pparam_R * cp_e_unit + pparam_cc_e;

    // calculate tangent
    Td_n = pparam_ldir * -cp_e_unit;
    Td_e = pparam_ldir * cp_n_unit;

    // spiral angular position: [0,2*pi)
    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
    double delta_xi = xi_sp-pparam_xi0;

    // closest point on nearest spiral leg and tangent down component
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

        delta_xi = delta_xi + 6.28318530718;

    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

        delta_xi = delta_xi - 6.28318530718;

    }

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
        Td_d = -sin(pparam_gam_sp);
    }
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double t12 = d_d-in[2];
const double t14 = d_e-in[1];
const double t11 = Td_e*t12-Td_d*t14;
const double t15 = d_n-in[0];
const double t13 = -Td_d*t15+Td_n*t12;
const double t16 = Td_e*t15-Td_n*t14;

/* outputs */

out[0] = sqrt(t11*t11+t13*t13+t16*t16);
out[1] = in[6];
out[2] = in[7];

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

