#include "acado_common.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

bool check_line_seg( const double *pos, const double *vel, const double *params );
bool check_curve_seg( const double *pos, const double *vel, const double *params );

void rhs( const real_t *in, real_t *out ){

/* for manual input indexing ... */

const int minus_NU = 0;

/* optimized intermediate calculations */

const double t2 = cos(in[4]);

const double alpha = -in[4]+in[7];

const double t16 = alpha-in[41]+in[43];
const double t17 = 1.0/(in[43]*in[43]);
const double t18 = -alpha+in[42]+in[43];
const double t19 = sin(in[4]);

double Vsafe = in[3];
if (Vsafe<1.0) Vsafe = 1.0;

const double n_dot = in[38]+Vsafe*t2*cos(in[5]);
const double e_dot = in[39]+Vsafe*t2*sin(in[5]);
const double d_dot = in[40]-Vsafe*t19;

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
const int idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
bool b_switch_segment = false;
int pparam_sel = 0;
double sw_dot = 0.0;
if ( in[ACADO_NX-1] < 0.05 ) { // check x_sw
    const double vel[3] = {n_dot,e_dot,d_dot};
    if ( in[idx_OD_0] < 0.5 ) { // path type
        b_switch_segment = check_line_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    } else if (in[ACADO_NX+ACADO_NU-minus_NU] < 1.5 ) {
        b_switch_segment = check_curve_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    }
} else {
    b_switch_segment = true;
}
if (b_switch_segment) {
    pparam_sel = 9;
    sw_dot = 1.0;
} 

double d_n = 0.0;
double d_e = 0.0;
double d_d = 0.0;
double Td_n = 1.0;
double Td_e = 0.0;
double Td_d = 0.0;

const double pparam_type = in[idx_OD_0+pparam_sel];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[idx_OD_0+pparam_sel+1];
    const double pparam_aa_e = in[idx_OD_0+pparam_sel+2];
    const double pparam_aa_d = in[idx_OD_0+pparam_sel+3];
    const double pparam_bb_n = in[idx_OD_0+pparam_sel+4];
    const double pparam_bb_e = in[idx_OD_0+pparam_sel+5];
    const double pparam_bb_d = in[idx_OD_0+pparam_sel+6];

    // calculate vector from waypoint a to b
    const double abn = pparam_bb_n - pparam_aa_n;
    const double abe = pparam_bb_e - pparam_aa_e;
    const double abd = pparam_bb_d - pparam_aa_d;
    const double norm_ab = sqrt(abn*abn + abe*abe + abd*abd);

    // calculate tangent
    if (norm_ab>0.1) {
        Td_n = abn / norm_ab;
        Td_e = abe / norm_ab;
        Td_d = abd / norm_ab;
    }
    
    // dot product
    const double dot_abunit_ap = Td_n*(in[0] - pparam_aa_n) + Td_e*(in[1] - pparam_aa_e) + Td_d*(in[2] - pparam_aa_d);
    
    // point on track
    d_n = pparam_aa_n + dot_abunit_ap * Td_n;
    d_e = pparam_aa_e + dot_abunit_ap * Td_e;
    d_d = pparam_aa_d + dot_abunit_ap * Td_d;
    
// CURVE SEGMENT
} else if ( pparam_type < 1.5 ) {

    // variable definitions
    const double pparam_cc_n = in[idx_OD_0+pparam_sel+1];
    const double pparam_cc_e = in[idx_OD_0+pparam_sel+2];
    const double pparam_cc_d = in[idx_OD_0+pparam_sel+3];
    const double pparam_R = in[idx_OD_0+pparam_sel+4];
    const double pparam_ldir = in[idx_OD_0+pparam_sel+5];
    const double pparam_gam_sp = in[idx_OD_0+pparam_sel+6];
    const double pparam_xi0 = in[idx_OD_0+pparam_sel+7];
    const double pparam_dxi = in[idx_OD_0+pparam_sel+8];

    // calculate closest point on loiter circle
    const double cp_n = in[0] - pparam_cc_n;
    const double cp_e = in[1] - pparam_cc_e;
    const double norm_cp = sqrt( cp_n*cp_n + cp_e*cp_e );
    double cp_n_unit;
    double cp_e_unit;
    if (norm_cp>0.1) {
        cp_n_unit = cp_n / norm_cp;
        cp_e_unit = cp_e / norm_cp;
    }
    else {
        cp_n_unit = 0.0;
        cp_e_unit = 0.0;
    }
    d_n = pparam_R * cp_n_unit + pparam_cc_n;
    d_e = pparam_R * cp_e_unit + pparam_cc_e;

    // calculate tangent
    Td_n = pparam_ldir * -cp_e_unit;
    Td_e = pparam_ldir * cp_n_unit;
    
    // spiral angular position: [0,2*pi)
    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
    double delta_xi_p = xi_sp-pparam_xi0;
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

        delta_xi_p = delta_xi_p + 6.28318530718;

    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

        delta_xi_p = delta_xi_p - 6.28318530718;

    }

    // closest point on nearest spiral leg and tangent down component
    if (fabs(pparam_gam_sp) < 0.001) {

        d_d = pparam_cc_d;
        Td_d = 0.0;

    } else {

        const double Rtangam = pparam_R * tan(pparam_gam_sp);

        // spiral height delta for current angle
        const double delta_d_xi = -delta_xi_p * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end = -pparam_dxi * Rtangam;

        // nearest spiral leg
        const double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        // closest point on nearest spiral leg
        d_d = pparam_cc_d + delta_d_k + delta_d_xi;

        /* d (on spiral) = (start height) + (revolution height increment) +
         * (lateral-direcitonal angular position increment)
         */
        
        // cap end point
        if ((d_d - (delta_d_sp_end + pparam_cc_d)) * pparam_gam_sp < 0.0) {
            // we (or more correctly, the closest point on the nearest spiral leg) are beyond the end point
            d_d = pparam_cc_d + delta_d_sp_end;
            Td_d = 0.0;
        }
        else {
            Td_d = -sin(pparam_gam_sp);
        }
        
    }
    
    if (fabs(Td_n)<0.01 && fabs(Td_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        Td_n=1.0;
    }
    
    // Renormalize Td
    const double normTd = sqrt(Td_n*Td_n+Td_e*Td_e+Td_d*Td_d);
    Td_n = Td_n / normTd;
    Td_e = Td_e / normTd;
    Td_d = Td_d / normTd;
        
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double t3 = d_e-in[1];
const double t4 = d_n-in[0];
const double t6 = d_d-in[2];

const double norm_pd_ne = sqrt(t3*t3+t4*t4);
const double e_t_ne = -Td_e*t4+Td_n*t3;
const double e_t_d = t6;

double pd_n_unit = 0.0; 
double pd_e_unit = 0.0;
if (norm_pd_ne>0.001) {
    pd_n_unit = t4/norm_pd_ne;
    pd_e_unit = t3/norm_pd_ne;
}

// take sign of height difference
double sgn_pd = 0.0;
if (t6<0.0) {
    sgn_pd = -1.0;
}
else if (t6>0.0) {
    sgn_pd = 1.0;
}

// saturated track error ratios
double sat_e_ne = fabs(e_t_ne)/in[44];
if ( sat_e_ne>1.0 ) sat_e_ne = 1.0;

double sat_e_d = fabs(e_t_d)/in[45];
if ( sat_e_d>1.0 ) sat_e_d = 1.0;

const double t7 = sat_e_ne-1.0;
const double t8 = t7*t7;
// const double t9 = t8*3.141592653589793*(1.0/2.0);
// const double t10 = sin(t9);
// const double t11 = cos(t9);
const double t9 = t8;
const double t10 = t9;
const double t11 = 1-t9;
const double t12 = sat_e_d-1.0;
const double t13 = t12*t12;
// const double t14 = t13*3.141592653589793*(1.0/2.0);
// const double t15 = sin(t14);
const double t14 = t13;
const double t15 = t14;

const double t20 = in[11]*in[11];
const double t21 = 1.0/Vsafe;
const double t22 = Vsafe*Vsafe;
const double t23 = alpha*alpha;
const double t24 = in[11]*8.61861E1;
const double t25 = in[11]*t20*2.501023E2;
const double t33 = t20*3.05322E1;
const double t26 = t24+t25-t33;
const double t27 = alpha*5.0996;
const double t28 = t23*(-5.343)+t27+1.7E1/3.2E1;
const double t29 = t22*t28*2.38875E-1;
const double t30 = cos(alpha);
const double t31 = 1.0/t30;
const double t32 = sin(alpha);
const double t34 = t21*t26*t31*t32;
const double t35 = t29+t34;
const double t36 = cos(in[6]);
const double t37 = sin(in[6]);

// error angles
const double atan2_01 = atan2(Td_e*t10+pd_e_unit*t11, Td_n*t10+pd_n_unit*t11);
const double atan2_02 = atan2(e_dot, n_dot);

double eta_lat = atan2_01-atan2_02;
if (eta_lat>3.141592653589793) {
    eta_lat = eta_lat - 6.283185307179586;
}
else if (eta_lat<-3.141592653589793) {
    eta_lat = eta_lat + 6.283185307179586;
}

// const double atan2_03 = atan2(-Td_d*t15-sgn_pd*cos(t14), t15*sqrt(Td_e*Td_e+Td_n*Td_n));
const double atan2_03 = atan2(-Td_d*t15-sgn_pd*(1-t14), t15*sqrt(Td_e*Td_e+Td_n*Td_n));
const double atan2_04 = atan2(-d_dot, sqrt(e_dot*e_dot+n_dot*n_dot));

double eta_lon = atan2_03-atan2_04;
if (eta_lon>3.141592653589793) {
    eta_lon = eta_lon - 6.283185307179586;
}
else if (eta_lon<-3.141592653589793) {
    eta_lon = eta_lon + 6.283185307179586;
}

// integrator differentials
double i_eta_lat_dot = 0.0;
if (fabs(eta_lat) < in[46]) i_eta_lat_dot = eta_lat/in[46];

double i_eta_lon_dot = 0.0;
if (fabs(eta_lon) < in[47]) i_eta_lon_dot = eta_lon/in[47];

/* rhs */

out[0] = n_dot;
out[1] = e_dot;
out[2] = d_dot;
out[3] = t19*(-9.81E2/1.0E2)+t21*t26*(2.0E1/5.3E1)-t22*(alpha*2.5491E-1+t23*2.7337+6.4105E-2)*9.014150943396226E-2;
out[4] = -t21*(t2*(9.81E2/1.0E2)-t35*t36*(2.0E1/5.3E1));
out[5] = (t21*t35*t37*(2.0E1/5.3E1))/t2;
out[6] = in[8];
out[7] = in[9]*t36-in[10]*t37;
out[8] = in[6]*(-1.24716E1)-in[8]*7.4252+in[10]*1.0069+in[16]*1.24716E1;
out[9] = -t22*(alpha*1.9303E-1+in[7]*1.8359E-1+in[9]*4.6239E-2-in[17]*1.8359E-1-9.4955E-4);
out[10] = in[6]*5.7996-in[10]*9.5153+in[16]*1.5967;
out[11] = in[11]*(-4.143016944939305)+in[15]*4.143016944939305;
out[12] = i_eta_lat_dot;
out[13] = i_eta_lon_dot;
out[14] = sw_dot;

}

void rhs_eval( real_t *in, real_t *out ){

/* for manual input indexing ... */

const int minus_NU = 0;

/* optimized intermediate calculations */

const double t2 = cos(in[4]);

const double alpha = -in[4]+in[7];

const double t16 = alpha-in[41]+in[43];
const double t17 = 1.0/(in[43]*in[43]);
const double t18 = -alpha+in[42]+in[43];
const double t19 = sin(in[4]);

double Vsafe = in[3];
if (Vsafe<1.0) Vsafe = 1.0;

const double n_dot = in[38]+Vsafe*t2*cos(in[5]);
const double e_dot = in[39]+Vsafe*t2*sin(in[5]);
const double d_dot = in[40]-Vsafe*t19;

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
const int idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
bool b_switch_segment = false;
int pparam_sel = 0;
double sw_dot = 0.0;
if ( in[ACADO_NX-1] < 0.05 ) { // check x_sw
    const double vel[3] = {n_dot,e_dot,d_dot};
    if ( in[idx_OD_0] < 0.5 ) { // path type
        b_switch_segment = check_line_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    } else if (in[ACADO_NX+ACADO_NU-minus_NU] < 1.5 ) {
        b_switch_segment = check_curve_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    }
} else {
    b_switch_segment = true;
}
if (b_switch_segment) {
    pparam_sel = 9;
    sw_dot = 1.0;
} 

double d_n = 0.0;
double d_e = 0.0;
double d_d = 0.0;
double Td_n = 1.0;
double Td_e = 0.0;
double Td_d = 0.0;

const double pparam_type = in[idx_OD_0+pparam_sel];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[idx_OD_0+pparam_sel+1];
    const double pparam_aa_e = in[idx_OD_0+pparam_sel+2];
    const double pparam_aa_d = in[idx_OD_0+pparam_sel+3];
    const double pparam_bb_n = in[idx_OD_0+pparam_sel+4];
    const double pparam_bb_e = in[idx_OD_0+pparam_sel+5];
    const double pparam_bb_d = in[idx_OD_0+pparam_sel+6];

    // calculate vector from waypoint a to b
    const double abn = pparam_bb_n - pparam_aa_n;
    const double abe = pparam_bb_e - pparam_aa_e;
    const double abd = pparam_bb_d - pparam_aa_d;
    const double norm_ab = sqrt(abn*abn + abe*abe + abd*abd);

    // calculate tangent
    if (norm_ab>0.1) {
        Td_n = abn / norm_ab;
        Td_e = abe / norm_ab;
        Td_d = abd / norm_ab;
    }
    
    // dot product
    const double dot_abunit_ap = Td_n*(in[0] - pparam_aa_n) + Td_e*(in[1] - pparam_aa_e) + Td_d*(in[2] - pparam_aa_d);
    
    // point on track
    d_n = pparam_aa_n + dot_abunit_ap * Td_n;
    d_e = pparam_aa_e + dot_abunit_ap * Td_e;
    d_d = pparam_aa_d + dot_abunit_ap * Td_d;
    
// CURVE SEGMENT
} else if ( pparam_type < 1.5 ) {

    // variable definitions
    const double pparam_cc_n = in[idx_OD_0+pparam_sel+1];
    const double pparam_cc_e = in[idx_OD_0+pparam_sel+2];
    const double pparam_cc_d = in[idx_OD_0+pparam_sel+3];
    const double pparam_R = in[idx_OD_0+pparam_sel+4];
    const double pparam_ldir = in[idx_OD_0+pparam_sel+5];
    const double pparam_gam_sp = in[idx_OD_0+pparam_sel+6];
    const double pparam_xi0 = in[idx_OD_0+pparam_sel+7];
    const double pparam_dxi = in[idx_OD_0+pparam_sel+8];

    // calculate closest point on loiter circle
    const double cp_n = in[0] - pparam_cc_n;
    const double cp_e = in[1] - pparam_cc_e;
    const double norm_cp = sqrt( cp_n*cp_n + cp_e*cp_e );
    double cp_n_unit;
    double cp_e_unit;
    if (norm_cp>0.1) {
        cp_n_unit = cp_n / norm_cp;
        cp_e_unit = cp_e / norm_cp;
    }
    else {
        cp_n_unit = 0.0;
        cp_e_unit = 0.0;
    }
    d_n = pparam_R * cp_n_unit + pparam_cc_n;
    d_e = pparam_R * cp_e_unit + pparam_cc_e;

    // calculate tangent
    Td_n = pparam_ldir * -cp_e_unit;
    Td_e = pparam_ldir * cp_n_unit;
    
    // spiral angular position: [0,2*pi)
    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
    double delta_xi_p = xi_sp-pparam_xi0;
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

        delta_xi_p = delta_xi_p + 6.28318530718;

    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

        delta_xi_p = delta_xi_p - 6.28318530718;

    }

    // closest point on nearest spiral leg and tangent down component
    if (fabs(pparam_gam_sp) < 0.001) {

        d_d = pparam_cc_d;
        Td_d = 0.0;

    } else {

        const double Rtangam = pparam_R * tan(pparam_gam_sp);

        // spiral height delta for current angle
        const double delta_d_xi = -delta_xi_p * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end = -pparam_dxi * Rtangam;

        // nearest spiral leg
        const double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        // closest point on nearest spiral leg
        d_d = pparam_cc_d + delta_d_k + delta_d_xi;

        /* d (on spiral) = (start height) + (revolution height increment) +
         * (lateral-direcitonal angular position increment)
         */
        
        // cap end point
        if ((d_d - (delta_d_sp_end + pparam_cc_d)) * pparam_gam_sp < 0.0) {
            // we (or more correctly, the closest point on the nearest spiral leg) are beyond the end point
            d_d = pparam_cc_d + delta_d_sp_end;
            Td_d = 0.0;
        }
        else {
            Td_d = -sin(pparam_gam_sp);
        }
        
    }
    
    if (fabs(Td_n)<0.01 && fabs(Td_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        Td_n=1.0;
    }
    
    // Renormalize Td
    const double normTd = sqrt(Td_n*Td_n+Td_e*Td_e+Td_d*Td_d);
    Td_n = Td_n / normTd;
    Td_e = Td_e / normTd;
    Td_d = Td_d / normTd;
        
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double t3 = d_e-in[1];
const double t4 = d_n-in[0];
const double t6 = d_d-in[2];

const double norm_pd_ne = sqrt(t3*t3+t4*t4);
const double e_t_ne = -Td_e*t4+Td_n*t3;
const double e_t_d = t6;

double pd_n_unit = 0.0; 
double pd_e_unit = 0.0;
if (norm_pd_ne>0.001) {
    pd_n_unit = t4/norm_pd_ne;
    pd_e_unit = t3/norm_pd_ne;
}

// take sign of height difference
double sgn_pd = 0.0;
if (t6<0.0) {
    sgn_pd = -1.0;
}
else if (t6>0.0) {
    sgn_pd = 1.0;
}

// saturated track error ratios
double sat_e_ne = fabs(e_t_ne)/in[44];
if ( sat_e_ne>1.0 ) sat_e_ne = 1.0;

double sat_e_d = fabs(e_t_d)/in[45];
if ( sat_e_d>1.0 ) sat_e_d = 1.0;

const double t7 = sat_e_ne-1.0;
const double t8 = t7*t7;
// const double t9 = t8*3.141592653589793*(1.0/2.0);
// const double t10 = sin(t9);
// const double t11 = cos(t9);
const double t9 = t8;
const double t10 = t9;
const double t11 = 1-t9;
const double t12 = sat_e_d-1.0;
const double t13 = t12*t12;
// const double t14 = t13*3.141592653589793*(1.0/2.0);
// const double t15 = sin(t14);
const double t14 = t13;
const double t15 = t14;

const double t20 = in[11]*in[11];
const double t21 = 1.0/Vsafe;
const double t22 = Vsafe*Vsafe;
const double t23 = alpha*alpha;
const double t24 = in[11]*8.61861E1;
const double t25 = in[11]*t20*2.501023E2;
const double t33 = t20*3.05322E1;
const double t26 = t24+t25-t33;
const double t27 = alpha*5.0996;
const double t28 = t23*(-5.343)+t27+1.7E1/3.2E1;
const double t29 = t22*t28*2.38875E-1;
const double t30 = cos(alpha);
const double t31 = 1.0/t30;
const double t32 = sin(alpha);
const double t34 = t21*t26*t31*t32;
const double t35 = t29+t34;
const double t36 = cos(in[6]);
const double t37 = sin(in[6]);

// error angles
const double atan2_01 = atan2(Td_e*t10+pd_e_unit*t11, Td_n*t10+pd_n_unit*t11);
const double atan2_02 = atan2(e_dot, n_dot);

double eta_lat = atan2_01-atan2_02;
if (eta_lat>3.141592653589793) {
    eta_lat = eta_lat - 6.283185307179586;
}
else if (eta_lat<-3.141592653589793) {
    eta_lat = eta_lat + 6.283185307179586;
}

// const double atan2_03 = atan2(-Td_d*t15-sgn_pd*cos(t14), t15*sqrt(Td_e*Td_e+Td_n*Td_n));
const double atan2_03 = atan2(-Td_d*t15-sgn_pd*(1-t14), t15*sqrt(Td_e*Td_e+Td_n*Td_n));
const double atan2_04 = atan2(-d_dot, sqrt(e_dot*e_dot+n_dot*n_dot));

double eta_lon = atan2_03-atan2_04;
if (eta_lon>3.141592653589793) {
    eta_lon = eta_lon - 6.283185307179586;
}
else if (eta_lon<-3.141592653589793) {
    eta_lon = eta_lon + 6.283185307179586;
}

// integrator differentials
double i_eta_lat_dot = 0.0;
if (fabs(eta_lat) < in[46]) i_eta_lat_dot = eta_lat/in[46];

double i_eta_lon_dot = 0.0;
if (fabs(eta_lon) < in[47]) i_eta_lon_dot = eta_lon/in[47];

/* rhs */

out[0] = n_dot;
out[1] = e_dot;
out[2] = d_dot;
out[3] = t19*(-9.81E2/1.0E2)+t21*t26*(2.0E1/5.3E1)-t22*(alpha*2.5491E-1+t23*2.7337+6.4105E-2)*9.014150943396226E-2;
out[4] = -t21*(t2*(9.81E2/1.0E2)-t35*t36*(2.0E1/5.3E1));
out[5] = (t21*t35*t37*(2.0E1/5.3E1))/t2;
out[6] = in[8];
out[7] = in[9]*t36-in[10]*t37;
out[8] = in[6]*(-1.24716E1)-in[8]*7.4252+in[10]*1.0069+in[16]*1.24716E1;
out[9] = -t22*(alpha*1.9303E-1+in[7]*1.8359E-1+in[9]*4.6239E-2-in[17]*1.8359E-1-9.4955E-4);
out[10] = in[6]*5.7996-in[10]*9.5153+in[16]*1.5967;
out[11] = in[11]*(-4.143016944939305)+in[15]*4.143016944939305;
out[12] = i_eta_lat_dot;
out[13] = i_eta_lon_dot;
out[14] = sw_dot;

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

const double t2 = cos(in[4]);

const double alpha = -in[4]+in[7];

double Vsafe = in[3];
if (Vsafe<1.0) Vsafe = 1.0;

const double n_dot = in[38]+Vsafe*t2*cos(in[5]);
const double e_dot = in[39]+Vsafe*t2*sin(in[5]);
const double d_dot = in[40]-Vsafe*sin(in[4]);

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
const int idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
bool b_switch_segment = false;
int pparam_sel = 0;
double sw_dot = 0.0;
if ( in[ACADO_NX-1] < 0.05 ) { // check x_sw
    const double vel[3] = {n_dot,e_dot,d_dot};
    if ( in[idx_OD_0] < 0.5 ) { // path type
        b_switch_segment = check_line_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    } else if (in[ACADO_NX+ACADO_NU-minus_NU] < 1.5 ) {
        b_switch_segment = check_curve_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    }
} else {
    b_switch_segment = true;
}
if (b_switch_segment) {
    pparam_sel = 9;
    sw_dot = 1.0;
} 

double d_n = 0.0;
double d_e = 0.0;
double d_d = 0.0;
double Td_n = 1.0;
double Td_e = 0.0;
double Td_d = 0.0;

const double pparam_type = in[idx_OD_0+pparam_sel];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[idx_OD_0+pparam_sel+1];
    const double pparam_aa_e = in[idx_OD_0+pparam_sel+2];
    const double pparam_aa_d = in[idx_OD_0+pparam_sel+3];
    const double pparam_bb_n = in[idx_OD_0+pparam_sel+4];
    const double pparam_bb_e = in[idx_OD_0+pparam_sel+5];
    const double pparam_bb_d = in[idx_OD_0+pparam_sel+6];

    // calculate vector from waypoint a to b
    const double abn = pparam_bb_n - pparam_aa_n;
    const double abe = pparam_bb_e - pparam_aa_e;
    const double abd = pparam_bb_d - pparam_aa_d;
    const double norm_ab = sqrt(abn*abn + abe*abe + abd*abd);

    // calculate tangent
    if (norm_ab>0.1) {
        Td_n = abn / norm_ab;
        Td_e = abe / norm_ab;
        Td_d = abd / norm_ab;
    }
    
    // dot product
    const double dot_abunit_ap = Td_n*(in[0] - pparam_aa_n) + Td_e*(in[1] - pparam_aa_e) + Td_d*(in[2] - pparam_aa_d);
    
    // point on track
    d_n = pparam_aa_n + dot_abunit_ap * Td_n;
    d_e = pparam_aa_e + dot_abunit_ap * Td_e;
    d_d = pparam_aa_d + dot_abunit_ap * Td_d;
    
// CURVE SEGMENT
} else if ( pparam_type < 1.5 ) {

    // variable definitions
    const double pparam_cc_n = in[idx_OD_0+pparam_sel+1];
    const double pparam_cc_e = in[idx_OD_0+pparam_sel+2];
    const double pparam_cc_d = in[idx_OD_0+pparam_sel+3];
    const double pparam_R = in[idx_OD_0+pparam_sel+4];
    const double pparam_ldir = in[idx_OD_0+pparam_sel+5];
    const double pparam_gam_sp = in[idx_OD_0+pparam_sel+6];
    const double pparam_xi0 = in[idx_OD_0+pparam_sel+7];
    const double pparam_dxi = in[idx_OD_0+pparam_sel+8];

    // calculate closest point on loiter circle
    const double cp_n = in[0] - pparam_cc_n;
    const double cp_e = in[1] - pparam_cc_e;
    const double norm_cp = sqrt( cp_n*cp_n + cp_e*cp_e );
    double cp_n_unit;
    double cp_e_unit;
    if (norm_cp>0.1) {
        cp_n_unit = cp_n / norm_cp;
        cp_e_unit = cp_e / norm_cp;
    }
    else {
        cp_n_unit = 0.0;
        cp_e_unit = 0.0;
    }
    d_n = pparam_R * cp_n_unit + pparam_cc_n;
    d_e = pparam_R * cp_e_unit + pparam_cc_e;

    // calculate tangent
    Td_n = pparam_ldir * -cp_e_unit;
    Td_e = pparam_ldir * cp_n_unit;
    
    // spiral angular position: [0,2*pi)
    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
    double delta_xi_p = xi_sp-pparam_xi0;
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

        delta_xi_p = delta_xi_p + 6.28318530718;

    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

        delta_xi_p = delta_xi_p - 6.28318530718;

    }

    // closest point on nearest spiral leg and tangent down component
    if (fabs(pparam_gam_sp) < 0.001) {

        d_d = pparam_cc_d;
        Td_d = 0.0;

    } else {

        const double Rtangam = pparam_R * tan(pparam_gam_sp);

        // spiral height delta for current angle
        const double delta_d_xi = -delta_xi_p * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end = -pparam_dxi * Rtangam;

        // nearest spiral leg
        const double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        // closest point on nearest spiral leg
        d_d = pparam_cc_d + delta_d_k + delta_d_xi;

        /* d (on spiral) = (start height) + (revolution height increment) +
         * (lateral-direcitonal angular position increment)
         */
        
        // cap end point
        if ((d_d - (delta_d_sp_end + pparam_cc_d)) * pparam_gam_sp < 0.0) {
            // we (or more correctly, the closest point on the nearest spiral leg) are beyond the end point
            d_d = pparam_cc_d + delta_d_sp_end;
            Td_d = 0.0;
        }
        else {
            Td_d = -sin(pparam_gam_sp);
        }
        
    }
    
    if (fabs(Td_n)<0.01 && fabs(Td_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        Td_n=1.0;
    }
    
    // Renormalize Td
    const double normTd = sqrt(Td_n*Td_n+Td_e*Td_e+Td_d*Td_d);
    Td_n = Td_n / normTd;
    Td_e = Td_e / normTd;
    Td_d = Td_d / normTd;
        
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double t3 = d_e-in[1];
const double t4 = d_n-in[0];
const double t6 = d_d-in[2];

const double norm_pd_ne = sqrt(t3*t3+t4*t4);
const double e_t_ne = -Td_e*t4+Td_n*t3;
const double e_t_d = t6;

double pd_n_unit = 0.0; 
double pd_e_unit = 0.0;
if (norm_pd_ne>0.001) {
    pd_n_unit = t4/norm_pd_ne;
    pd_e_unit = t3/norm_pd_ne;
}

// take sign of height difference
double sgn_pd = 0.0;
if (t6<0.0) {
    sgn_pd = -1.0;
}
else if (t6>0.0) {
    sgn_pd = 1.0;
}

// saturated track error ratios
double sat_e_ne = fabs(e_t_ne)/in[44];
if ( sat_e_ne>1.0 ) sat_e_ne = 1.0;

double sat_e_d = fabs(e_t_d)/in[45];
if ( sat_e_d>1.0 ) sat_e_d = 1.0;

const double t7 = sat_e_ne-1.0;
const double t8 = t7*t7;
// const double t9 = t8*3.141592653589793*(1.0/2.0);
// const double t10 = sin(t9);
// const double t11 = cos(t9);
const double t9 = t8;
const double t10 = t9;
const double t11 = 1-t9;
const double t12 = sat_e_d-1.0;
const double t13 = t12*t12;
// const double t14 = t13*3.141592653589793*(1.0/2.0);
// const double t15 = sin(t14);
const double t14 = t13;
const double t15 = t14;
const double t16 = alpha-in[41]+in[43];
const double t17 = 1.0/(in[43]*in[43]);
const double t18 = -alpha+in[42]+in[43];

// error angles
const double atan2_01 = atan2(Td_e*t10+pd_e_unit*t11, Td_n*t10+pd_n_unit*t11);
const double atan2_02 = atan2(e_dot, n_dot);

double eta_lat = atan2_01-atan2_02;
if (eta_lat>3.141592653589793) {
    eta_lat = eta_lat - 6.283185307179586;
}
else if (eta_lat<-3.141592653589793) {
    eta_lat = eta_lat + 6.283185307179586;
}

// const double atan2_03 = atan2(-Td_d*t15-sgn_pd*cos(t14), t15*sqrt(Td_e*Td_e+Td_n*Td_n));
const double atan2_03 = atan2(-Td_d*t15-sgn_pd*(1-t14), t15*sqrt(Td_e*Td_e+Td_n*Td_n));
const double atan2_04 = atan2(-d_dot, sqrt(e_dot*e_dot+n_dot*n_dot));

double eta_lon = atan2_03-atan2_04;
if (eta_lon>3.141592653589793) {
    eta_lon = eta_lon - 6.283185307179586;
}
else if (eta_lon<-3.141592653589793) {
    eta_lon = eta_lon + 6.283185307179586;
}

// soft constraints
double a_soft;
if (alpha>(in[41]-in[43])) {
    a_soft=(t16*t16)*t17;
}
else if (alpha>(in[42]+in[43])) {
    a_soft=0.0;
}
else {
    a_soft=t17*(t18*t18);
}

/* outputs */

out[0] = eta_lat;
out[1] = eta_lon;
out[2] = in[12];
out[3] = in[13];
out[4] = Vsafe;
out[5] = in[8];
out[6] = in[9];
out[7] = in[10];
out[8] = a_soft;
out[9] = in[11]*(-4.143016944939305)+in[15]*4.143016944939305;
out[10] = in[15];
out[11] = in[16];
out[12] = in[17];
out[13] = in[15];
out[14] = in[16];
out[15] = in[17];

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

const double t2 = cos(in[4]);

const double alpha = -in[4]+in[7];

double Vsafe = in[3];
if (Vsafe<1.0) Vsafe = 1.0;

const double n_dot = in[35]+Vsafe*t2*cos(in[5]);
const double e_dot = in[36]+Vsafe*t2*sin(in[5]);
const double d_dot = in[37]-Vsafe*sin(in[4]);

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
const int idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
bool b_switch_segment = false;
int pparam_sel = 0;
double sw_dot = 0.0;
if ( in[ACADO_NX-1] < 0.05 ) { // check x_sw
    const double vel[3] = {n_dot,e_dot,d_dot};
    if ( in[idx_OD_0] < 0.5 ) { // path type
        b_switch_segment = check_line_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    } else if (in[ACADO_NX+ACADO_NU-minus_NU] < 1.5 ) {
        b_switch_segment = check_curve_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    }
} else {
    b_switch_segment = true;
}
if (b_switch_segment) {
    pparam_sel = 9;
    sw_dot = 1.0;
} 

double d_n = 0.0;
double d_e = 0.0;
double d_d = 0.0;
double Td_n = 1.0;
double Td_e = 0.0;
double Td_d = 0.0;

const double pparam_type = in[idx_OD_0+pparam_sel];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[idx_OD_0+pparam_sel+1];
    const double pparam_aa_e = in[idx_OD_0+pparam_sel+2];
    const double pparam_aa_d = in[idx_OD_0+pparam_sel+3];
    const double pparam_bb_n = in[idx_OD_0+pparam_sel+4];
    const double pparam_bb_e = in[idx_OD_0+pparam_sel+5];
    const double pparam_bb_d = in[idx_OD_0+pparam_sel+6];

    // calculate vector from waypoint a to b
    const double abn = pparam_bb_n - pparam_aa_n;
    const double abe = pparam_bb_e - pparam_aa_e;
    const double abd = pparam_bb_d - pparam_aa_d;
    const double norm_ab = sqrt(abn*abn + abe*abe + abd*abd);

    // calculate tangent
    if (norm_ab>0.1) {
        Td_n = abn / norm_ab;
        Td_e = abe / norm_ab;
        Td_d = abd / norm_ab;
    }
    
    // dot product
    const double dot_abunit_ap = Td_n*(in[0] - pparam_aa_n) + Td_e*(in[1] - pparam_aa_e) + Td_d*(in[2] - pparam_aa_d);
    
    // point on track
    d_n = pparam_aa_n + dot_abunit_ap * Td_n;
    d_e = pparam_aa_e + dot_abunit_ap * Td_e;
    d_d = pparam_aa_d + dot_abunit_ap * Td_d;
    
// CURVE SEGMENT
} else if ( pparam_type < 1.5 ) {

    // variable definitions
    const double pparam_cc_n = in[idx_OD_0+pparam_sel+1];
    const double pparam_cc_e = in[idx_OD_0+pparam_sel+2];
    const double pparam_cc_d = in[idx_OD_0+pparam_sel+3];
    const double pparam_R = in[idx_OD_0+pparam_sel+4];
    const double pparam_ldir = in[idx_OD_0+pparam_sel+5];
    const double pparam_gam_sp = in[idx_OD_0+pparam_sel+6];
    const double pparam_xi0 = in[idx_OD_0+pparam_sel+7];
    const double pparam_dxi = in[idx_OD_0+pparam_sel+8];

    // calculate closest point on loiter circle
    const double cp_n = in[0] - pparam_cc_n;
    const double cp_e = in[1] - pparam_cc_e;
    const double norm_cp = sqrt( cp_n*cp_n + cp_e*cp_e );
    double cp_n_unit;
    double cp_e_unit;
    if (norm_cp>0.1) {
        cp_n_unit = cp_n / norm_cp;
        cp_e_unit = cp_e / norm_cp;
    }
    else {
        cp_n_unit = 0.0;
        cp_e_unit = 0.0;
    }
    d_n = pparam_R * cp_n_unit + pparam_cc_n;
    d_e = pparam_R * cp_e_unit + pparam_cc_e;

    // calculate tangent
    Td_n = pparam_ldir * -cp_e_unit;
    Td_e = pparam_ldir * cp_n_unit;
    
    // spiral angular position: [0,2*pi)
    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
    double delta_xi_p = xi_sp-pparam_xi0;
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

        delta_xi_p = delta_xi_p + 6.28318530718;

    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

        delta_xi_p = delta_xi_p - 6.28318530718;

    }

    // closest point on nearest spiral leg and tangent down component
    if (fabs(pparam_gam_sp) < 0.001) {

        d_d = pparam_cc_d;
        Td_d = 0.0;

    } else {

        const double Rtangam = pparam_R * tan(pparam_gam_sp);

        // spiral height delta for current angle
        const double delta_d_xi = -delta_xi_p * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end = -pparam_dxi * Rtangam;

        // nearest spiral leg
        const double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        // closest point on nearest spiral leg
        d_d = pparam_cc_d + delta_d_k + delta_d_xi;

        /* d (on spiral) = (start height) + (revolution height increment) +
         * (lateral-direcitonal angular position increment)
         */
        
        // cap end point
        if ((d_d - (delta_d_sp_end + pparam_cc_d)) * pparam_gam_sp < 0.0) {
            // we (or more correctly, the closest point on the nearest spiral leg) are beyond the end point
            d_d = pparam_cc_d + delta_d_sp_end;
            Td_d = 0.0;
        }
        else {
            Td_d = -sin(pparam_gam_sp);
        }
        
    }
    
    if (fabs(Td_n)<0.01 && fabs(Td_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        Td_n=1.0;
    }
    
    // Renormalize Td
    const double normTd = sqrt(Td_n*Td_n+Td_e*Td_e+Td_d*Td_d);
    Td_n = Td_n / normTd;
    Td_e = Td_e / normTd;
    Td_d = Td_d / normTd;
        
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double t3 = d_e-in[1];
const double t4 = d_n-in[0];
const double t6 = d_d-in[2];

const double norm_pd_ne = sqrt(t3*t3+t4*t4);
const double e_t_ne = -Td_e*t4+Td_n*t3;
const double e_t_d = t6;

// saturated track error ratios
double sat_e_ne = fabs(e_t_ne)/in[41];
if ( sat_e_ne>1.0 ) sat_e_ne = 1.0;

double sat_e_d = fabs(e_t_d)/in[42];
if ( sat_e_d>1.0 ) sat_e_d = 1.0;

const double t7 = sat_e_ne-1.0;
const double t8 = t7*t7;
// const double t9 = t8*3.141592653589793*(1.0/2.0);
// const double t10 = sin(t9);
// const double t11 = cos(t9);
const double t9 = t8;
const double t10 = t9;
const double t11 = 1-t9;
const double t12 = sat_e_d-1.0;
const double t13 = t12*t12;
// const double t14 = t13*3.141592653589793*(1.0/2.0);
// const double t15 = sin(t14);
const double t14 = t13;
const double t15 = t14;
const double t16 = alpha-in[38]+in[40];
const double t17 = 1.0/(in[40]*in[40]);
const double t18 = -alpha+in[39]+in[40];

double pd_n_unit = 0.0; 
double pd_e_unit = 0.0;
if (norm_pd_ne>0.001) {
    pd_n_unit = t4/norm_pd_ne;
    pd_e_unit = t3/norm_pd_ne;
}

// take sign of height difference
double sgn_pd = 0.0;
if (t6<0.0) {
    sgn_pd = -1.0;
}
else if (t6>0.0) {
    sgn_pd = 1.0;
}

// error angles
const double atan2_01 = atan2(Td_e*t10+pd_e_unit*t11, Td_n*t10+pd_n_unit*t11);
const double atan2_02 = atan2(e_dot, n_dot);

double eta_lat = atan2_01-atan2_02;
if (eta_lat>3.141592653589793) {
    eta_lat = eta_lat - 6.283185307179586;
}
else if (eta_lat<-3.141592653589793) {
    eta_lat = eta_lat + 6.283185307179586;
}

// const double atan2_03 = atan2(-Td_d*t15-sgn_pd*cos(t14), t15*sqrt(Td_e*Td_e+Td_n*Td_n));
const double atan2_03 = atan2(-Td_d*t15-sgn_pd*(1-t14), t15*sqrt(Td_e*Td_e+Td_n*Td_n));
const double atan2_04 = atan2(-d_dot, sqrt(e_dot*e_dot+n_dot*n_dot));

double eta_lon = atan2_03-atan2_04;
if (eta_lon>3.141592653589793) {
    eta_lon = eta_lon - 6.283185307179586;
}
else if (eta_lon<-3.141592653589793) {
    eta_lon = eta_lon + 6.283185307179586;
}

// soft constraints
double a_soft;
if (alpha>(in[48]-in[40])) {
    a_soft=(t16*t16)*t17;
}
else if (alpha>(in[39]+in[40])) {
    a_soft=0.0;
}
else {
    a_soft=t17*(t18*t18);
}

/* outputs */

out[0] = eta_lat;
out[1] = eta_lon;
out[2] = in[12];
out[3] = in[13];
out[4] = Vsafe;
out[5] = in[8];
out[6] = in[9];
out[7] = in[10];
out[8] = a_soft;

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

/* begin inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

bool check_line_seg( const double *pos, const double *vel, const double *params ) {
    
    // waypoint a to b
    const double ab_n = params[3] - params[0];
    const double ab_e = params[4] - params[1];
    const double ab_d = params[5] - params[2];
    
    const double norm_ab = sqrt( ab_n*ab_n + ab_e*ab_e + ab_d*ab_d );
    
    // Tb
    double Tb_n = 1.0;
    double Tb_e = 0.0;
    double Tb_d = 0.0;
    if (norm_ab > 0.1) {
        Tb_n = ab_n / norm_ab;
        Tb_e = ab_e / norm_ab;
        Tb_d = ab_d / norm_ab;
    }

    // p - b
    const double bp_n = pos[0] - params[3];
    const double bp_e = pos[1] - params[4];
    const double bp_d = pos[2] - params[5];
    
    // dot( v , Tb )
    const double dot_vTb = vel[0] * Tb_n + vel[1] * Tb_e + vel[2] * Tb_d;
    
    // dot( (p-b) , Tb )
    const double dot_bpTb = bp_n * Tb_n + bp_e * Tb_e + bp_d * Tb_d;
    
    // norm( p-b )
    const double norm_bp = sqrt( bp_n*bp_n + bp_e*bp_e + bp_d*bp_d );
    
    // check (1) proximity, (2) bearing, (3) travel 
    return ( (norm_bp < params[17] && dot_vTb > params[18]) || dot_bpTb > 0.0 );
    
}

bool check_curve_seg( const double *pos, const double *vel, const double *params ) {
    
    // chi_b
    const double chi_b = params[6] + params[4] * (params[7] + 1.570796326794897);
        
    // Tb
    const double Tb_n = cos(chi_b)*cos(params[5]);
    const double Tb_e = sin(chi_b)*cos(params[5]);
    const double Tb_d = -sin(params[5]);

    // p - b
    const double bp_n = pos[0] - ( params[0] + params[3] * ( cos(params[6] + params[4] * params[7]) ) );
    const double bp_e = pos[1] - ( params[1] + params[3] * ( sin(params[6] + params[4] * params[7]) ) );
    const double bp_d = pos[2] - ( params[2] - params[3] * tan(params[5]) * params[7] );
    
    // dot( v , Tb )
    const double dot_vTb = vel[0] * Tb_n + vel[1] * Tb_e + vel[2] * Tb_d;
    
    // dot( (p-b) , Tb )
    const double dot_bpTb = bp_n * Tb_n + bp_e * Tb_e + bp_d * Tb_d;
    
    // norm( p-b )
    const double norm_bp = sqrt( bp_n*bp_n + bp_e*bp_e + bp_d*bp_d );
    
    // check (1) proximity, (2) bearing, (3) travel 
    return ( norm_bp < params[17] && dot_vTb > params[18] && dot_bpTb > 0.0 );
}

/* end inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */