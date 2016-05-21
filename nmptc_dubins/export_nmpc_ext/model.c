#include "acado_common.h"
#include <math.h>
#include <string.h>

bool check_line_seg( const double *pos, const double *pparams );
bool check_curve_seg( const double *pos, const double *pparams );

void rhs( const real_t *in, real_t *out ){

/* optimized intermediate calculations */

const double t3 = cos(in[8]);
const double t9 = cos(in[3]);
const double t10 = in[9]*t3*t9;
const double t2 = in[28]+t10;
const double t7 = sin(in[3]);
const double t8 = in[9]*t3*t7;
const double t4 = in[29]+t8;
const double t5 = sin(in[8]);
const double t11 = in[9]*t5;
const double t6 = in[30]-t11;

double V_g = sqrt(t2*t2+t4*t4+t6*t6);
if (V_g < 0.01) V_g = 0.01;

double sin_d_dot_V_g = t6*1.0/V_g;
if (sin_d_dot_V_g > 1.0) sin_d_dot_V_g = 1.0;
if (sin_d_dot_V_g < -1.0) sin_d_dot_V_g = -1.0;

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
bool b_switch_segment = false; 
int pparam_sel = 0;
if ( in[10] < 0.5 ) {
    b_switch_segment = check_line_seg( &in[0], &in[9] );
} else if (in[10] < 1.5 ) {
    b_switch_segment = check_curve_seg( &in[0], &in[11] );
}
if (b_switch_segment) pparam_sel = 9;

double d_n = 0.0;
double d_e = 0.0;
double d_d = 0.0;
double Td_n = 1.0;
double Td_e = 0.0;
double Td_d = 0.0;
double Gamma_d = 0.0;

const double pparam_type = in[10+pparam_sel];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[11+pparam_sel];
    const double pparam_aa_e = in[12+pparam_sel];
    const double pparam_aa_d = in[13+pparam_sel];
    const double pparam_bb_n = in[14+pparam_sel];
    const double pparam_bb_e = in[15+pparam_sel];
    const double pparam_bb_d = in[16+pparam_sel];

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
    const double pparam_cc_n = in[11+pparam_sel];
    const double pparam_cc_e = in[12+pparam_sel];
    const double pparam_cc_d = in[13+pparam_sel];
    const double pparam_R = in[14+pparam_sel];
    const double pparam_ldir = in[15+pparam_sel];
    const double pparam_gam_sp = in[16+pparam_sel];
    const double pparam_xi0 = in[17+pparam_sel];
    const double pparam_dxi = in[18+pparam_sel];

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

const double t13 = d_d-in[2];
const double t15 = d_e-in[1];
const double t12 = Td_e*t13-Td_d*t15;
const double t16 = d_n-in[0];
const double t14 = -Td_d*t16+Td_n*t13;
const double t17 = Td_e*t16-Td_n*t15;

const double atan2_01 = atan2(Td_e, Td_n);
const double atan2_02 = atan2(t4, t2);
double e_chi = atan2_01-atan2_02;
if (e_chi>3.14159265359) e_chi = e_chi - 6.28318530718;
if (e_chi<-3.14159265359) e_chi = e_chi + 6.28318530718;

/* rhs */

out[0] = t2;
out[1] = t4;
out[2] = t6;
out[3] = (tan(in[7])*(9.81E2/1.0E2))/in[9];
out[4] = sqrt(t12*t12+t14*t14+t17*t17);
out[5] = Gamma_d+asin(sin_d_dot_V_g);
out[6] = e_chi;

}

void rhs_eval( real_t *in, real_t *out ){

/* optimized intermediate calculations */

const double t3 = cos(in[8]);
const double t9 = cos(in[3]);
const double t10 = in[9]*t3*t9;
const double t2 = in[28]+t10;
const double t7 = sin(in[3]);
const double t8 = in[9]*t3*t7;
const double t4 = in[29]+t8;
const double t5 = sin(in[8]);
const double t11 = in[9]*t5;
const double t6 = in[30]-t11;

double V_g = sqrt(t2*t2+t4*t4+t6*t6);
if (V_g < 0.01) V_g = 0.01;

double sin_d_dot_V_g = t6*1.0/V_g;
if (sin_d_dot_V_g > 1.0) sin_d_dot_V_g = 1.0;
if (sin_d_dot_V_g < -1.0) sin_d_dot_V_g = -1.0;

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
bool b_switch_segment = false; 
int pparam_sel = 0;
if ( in[10] < 0.5 ) {
    b_switch_segment = check_line_seg( &in[0], &in[9] );
} else if (in[10] < 1.5 ) {
    b_switch_segment = check_curve_seg( &in[0], &in[11] );
}
if (b_switch_segment) pparam_sel = 9;

double d_n = 0.0;
double d_e = 0.0;
double d_d = 0.0;
double Td_n = 1.0;
double Td_e = 0.0;
double Td_d = 0.0;
double Gamma_d = 0.0;

const double pparam_type = in[10+pparam_sel];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[11+pparam_sel];
    const double pparam_aa_e = in[12+pparam_sel];
    const double pparam_aa_d = in[13+pparam_sel];
    const double pparam_bb_n = in[14+pparam_sel];
    const double pparam_bb_e = in[15+pparam_sel];
    const double pparam_bb_d = in[16+pparam_sel];

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
    const double pparam_cc_n = in[11+pparam_sel];
    const double pparam_cc_e = in[12+pparam_sel];
    const double pparam_cc_d = in[13+pparam_sel];
    const double pparam_R = in[14+pparam_sel];
    const double pparam_ldir = in[15+pparam_sel];
    const double pparam_gam_sp = in[16+pparam_sel];
    const double pparam_xi0 = in[17+pparam_sel];
    const double pparam_dxi = in[18+pparam_sel];

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

const double t13 = d_d-in[2];
const double t15 = d_e-in[1];
const double t12 = Td_e*t13-Td_d*t15;
const double t16 = d_n-in[0];
const double t14 = -Td_d*t16+Td_n*t13;
const double t17 = Td_e*t16-Td_n*t15;

const double atan2_01 = atan2(Td_e, Td_n);
const double atan2_02 = atan2(t4, t2);
double e_chi = atan2_01-atan2_02;
if (e_chi>3.14159265359) e_chi = e_chi - 6.28318530718;
if (e_chi<-3.14159265359) e_chi = e_chi + 6.28318530718;

/* rhs */

out[0] = t2;
out[1] = t4;
out[2] = t6;
out[3] = (tan(in[7])*(9.81E2/1.0E2))/in[9];
out[4] = sqrt(t12*t12+t14*t14+t17*t17);
out[5] = Gamma_d+asin(sin_d_dot_V_g);
out[6] = e_chi;

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

const double t3 = cos(in[8]);
const double t9 = cos(in[3]);
const double t10 = in[9]*t3*t9;
const double t2 = in[28]+t10;
const double t7 = sin(in[3]);
const double t8 = in[9]*t3*t7;
const double t4 = in[29]+t8;
const double t5 = sin(in[8]);
const double t6 = in[30]-in[9]*t5;

double V_g = sqrt(t2*t2+t4*t4+t6*t6);
if (V_g < 0.01) V_g = 0.01;

double sin_d_dot_V_g = t6*1.0/V_g;
if (sin_d_dot_V_g > 1.0) sin_d_dot_V_g = 1.0;
if (sin_d_dot_V_g < -1.0) sin_d_dot_V_g = -1.0;

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
bool b_switch_segment = false; 
int pparam_sel = 0;
if ( in[10] < 0.5 ) {
    b_switch_segment = check_line_seg( &in[0], &in[9] );
} else if (in[10] < 1.5 ) {
    b_switch_segment = check_curve_seg( &in[0], &in[11] );
}
if (b_switch_segment) pparam_sel = 9;

double d_n = 0.0;
double d_e = 0.0;
double d_d = 0.0;
double Td_n = 1.0;
double Td_e = 0.0;
double Td_d = 0.0;
double Gamma_d = 0.0;

const double pparam_type = in[10+pparam_sel];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[11+pparam_sel];
    const double pparam_aa_e = in[12+pparam_sel];
    const double pparam_aa_d = in[13+pparam_sel];
    const double pparam_bb_n = in[14+pparam_sel];
    const double pparam_bb_e = in[15+pparam_sel];
    const double pparam_bb_d = in[16+pparam_sel];

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
    const double pparam_cc_n = in[11+pparam_sel];
    const double pparam_cc_e = in[12+pparam_sel];
    const double pparam_cc_d = in[13+pparam_sel];
    const double pparam_R = in[14+pparam_sel];
    const double pparam_ldir = in[15+pparam_sel];
    const double pparam_gam_sp = in[16+pparam_sel];
    const double pparam_xi0 = in[17+pparam_sel];
    const double pparam_dxi = in[18+pparam_sel];

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

const double t12 = d_d-in[2];
const double t14 = d_e-in[1];
const double t11 = Td_e*t12-Td_d*t14;
const double t15 = d_n-in[0];
const double t13 = -Td_d*t15+Td_n*t12;
const double t16 = Td_e*t15-Td_n*t14;

const double atan2_01 = atan2(Td_e, Td_n);
const double atan2_02 = atan2(t4, t2);
double e_chi = atan2_01-atan2_02;
if (e_chi>3.14159265359) e_chi = e_chi - 6.28318530718;
if (e_chi<-3.14159265359) e_chi = e_chi + 6.28318530718;

/* outputs */

out[0] = sqrt(t11*t11+t13*t13+t16*t16);
out[1] = Gamma_d+asin(sin_d_dot_V_g);
out[2] = e_chi;
out[3] = in[4];
out[4] = in[5];
out[5] = in[6];
out[6] = in[7];
out[7] = in[8];
out[8] = in[7]-in[31];
out[9] = in[8]-in[32];

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

const double t3 = cos(in[8]);
const double t9 = cos(in[3]);
const double t10 = in[9]*t3*t9;
const double t2 = in[28]+t10;
const double t7 = sin(in[3]);
const double t8 = in[9]*t3*t7;
const double t4 = in[29]+t8;
const double t5 = sin(in[8]);
const double t6 = in[30]-in[9]*t5;

double V_g = sqrt(t2*t2+t4*t4+t6*t6);
if (V_g < 0.01) V_g = 0.01;

double sin_d_dot_V_g = t6*1.0/V_g;
if (sin_d_dot_V_g > 1.0) sin_d_dot_V_g = 1.0;
if (sin_d_dot_V_g < -1.0) sin_d_dot_V_g = -1.0;

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
bool b_switch_segment = false; 
int pparam_sel = 0;
if ( in[10] < 0.5 ) {
    b_switch_segment = check_line_seg( &in[0], &in[9] );
} else if (in[10] < 1.5 ) {
    b_switch_segment = check_curve_seg( &in[0], &in[11] );
}
if (b_switch_segment) pparam_sel = 9;

double d_n = 0.0;
double d_e = 0.0;
double d_d = 0.0;
double Td_n = 1.0;
double Td_e = 0.0;
double Td_d = 0.0;
double Gamma_d = 0.0;

const double pparam_type = in[10+pparam_sel];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[11+pparam_sel];
    const double pparam_aa_e = in[12+pparam_sel];
    const double pparam_aa_d = in[13+pparam_sel];
    const double pparam_bb_n = in[14+pparam_sel];
    const double pparam_bb_e = in[15+pparam_sel];
    const double pparam_bb_d = in[16+pparam_sel];

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
    const double pparam_cc_n = in[11+pparam_sel];
    const double pparam_cc_e = in[12+pparam_sel];
    const double pparam_cc_d = in[13+pparam_sel];
    const double pparam_R = in[14+pparam_sel];
    const double pparam_ldir = in[15+pparam_sel];
    const double pparam_gam_sp = in[16+pparam_sel];
    const double pparam_xi0 = in[17+pparam_sel];
    const double pparam_dxi = in[18+pparam_sel];

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

const double t12 = d_d-in[2];
const double t14 = d_e-in[1];
const double t11 = Td_e*t12-Td_d*t14;
const double t15 = d_n-in[0];
const double t13 = -Td_d*t15+Td_n*t12;
const double t16 = Td_e*t15-Td_n*t14;

const double atan2_01 = atan2(Td_e, Td_n);
const double atan2_02 = atan2(t4, t2);
double e_chi = atan2_01-atan2_02;
if (e_chi>3.14159265359) e_chi = e_chi - 6.28318530718;
if (e_chi<-3.14159265359) e_chi = e_chi + 6.28318530718;

/* outputs */

out[0] = sqrt(t11*t11+t13*t13+t16*t16);
out[1] = Gamma_d+asin(sin_d_dot_V_g);
out[2] = e_chi;
out[3] = in[4];
out[4] = in[5];
out[5] = in[6];
out[6] = in[7];
out[7] = in[8];
out[8] = in[7]-in[31];
out[9] = in[8]-in[32];

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

/* begin inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

bool check_line_seg( const double *pos, const double *pparams ) {
    
    // calculate vector from waypoint a to b
    const double ab_n = pparams[5] - pparams[2];
    const double ab_e = pparams[6] - pparams[3];
    const double ab_d = pparams[7] - pparams[4];
    
    const double norm_ab = sqrt( ab_n*ab_n + ab_e*ab_e + ab_d*ab_d );
    
    // 1-D track position
    const double pb_t = norm_ab - ( (ab_n*(pos[0]-pparams[2]) + ab_e*(pos[1]-pparams[3]) + ab_d*(pos[2]-pparams[4])) ) / norm_ab;
    
    // check
    return ( pb_t < 0.0 );
}

bool check_curve_seg( const double *pos, const double *pparams ) {
    
    const double b_n = pparams[0] + pparams[3] * ( cos(pparams[6] + pparams[4] * pparams[7]) );
    const double b_e = pparams[1] + pparams[3] * ( sin(pparams[6] + pparams[4] * pparams[7]) );
    const double b_d = pparams[2] - pparams[3] * tan(pparams[5]) * pparams[7];
        
    const double Gamma_b = pparams[6] + pparams[4] * (pparams[7] + 1.570796326794897);
        
    const double Tb_n = cos(Gamma_b);
    const double Tb_e = sin(Gamma_b);
    const double Tb_d = -sin(pparams[5]);

    // 1-D track position //TODO: not hard-coded acceptance radius
    const double pb_t = 1.0 - ( Tb_n*(pos[0]-b_n+Tb_n) + Tb_e*(pos[1]-b_e+Tb_e) + Tb_d*(pos[2]-b_d+Tb_d) );
        
    // check
    return ( pb_t < 0.0 && fabs(b_d - pos[2]) < 10.0 );
}

/* end inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */