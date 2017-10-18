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
const double t8 = sin(in[4]);

const double alpha = -in[4]+in[7];

double Vsafe = in[3];
if (Vsafe<1.0) Vsafe = 1.0;

const double n_dot = in[32]+Vsafe*t2*cos(in[5]);
const double e_dot = in[33]+Vsafe*t2*sin(in[5]);
const double d_dot = in[34]-Vsafe*t8;

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
const int idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
bool b_switch_segment = false;
int pparam_sel = 0;
double sw_dot = 0.0;
if ( in[ACADO_NX-1] < 0.05 ) { // check x_sw
    const double vel[3] = {n_dot,e_dot,d_dot};
    if ( in[idx_OD_0] < 0.5 ) { // path type
        b_switch_segment = check_line_seg( &in[0], &vel[0], &in[idx_OD_0] );
    } else if (in[idx_OD_0] < 1.5 ) {
        b_switch_segment = check_curve_seg( &in[0], &vel[0], &in[idx_OD_0] );
    }
} else {
    b_switch_segment = true;
}
if (b_switch_segment) {
    pparam_sel = 7;
    sw_dot = 1.0;
} 

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double t3 = e_dot*e_dot;
const double t4 = n_dot*n_dot;
const double t5 = alpha-in[35]+in[37];
const double t6 = 1.0/(in[37]*in[37]);
const double t7 = -alpha+in[36]+in[37];

const double t9 = in[11]*in[11];
const double t10 = 1.0/Vsafe;
const double t11 = Vsafe*Vsafe;
const double t12 = alpha*alpha;
const double t13 = in[11]*8.61861E1;
const double t14 = in[11]*t9*2.501023E2;
const double t22 = t9*3.05322E1;
const double t15 = t13+t14-t22;
const double t16 = alpha*5.0996;
const double t17 = t12*(-5.343)+t16+1.7E1/3.2E1;
const double t18 = t11*t17*2.38875E-1;
const double t19 = cos(alpha);
const double t20 = 1.0/t19;
const double t21 = sin(alpha);
const double t23 = t10*t15*t20*t21;
const double t24 = t18+t23;
const double t25 = cos(in[6]);
const double t26 = sin(in[6]);

/* rhs */

out[0] = n_dot;
out[1] = e_dot;
out[2] = d_dot;
out[3] = t8*(-9.81E2/1.0E2)+t10*t15*(2.0E1/5.3E1)-t11*(alpha*2.5491E-1+t12*2.7337+6.4105E-2)*9.014150943396226E-2;
out[4] = -t10*(t2*(9.81E2/1.0E2)-t24*t25*(2.0E1/5.3E1));
out[5] = (t10*t24*t26*(2.0E1/5.3E1))/t2;
out[6] = in[8];
out[7] = in[9]*t25-in[10]*t26;
out[8] = in[6]*(-1.24716E1)-in[8]*7.4252+in[10]*1.0069+in[14]*1.24716E1;
out[9] = -t11*(alpha*1.9303E-1+in[7]*1.8359E-1+in[9]*4.6239E-2-in[15]*1.8359E-1-9.4955E-4);
out[10] = in[6]*5.7996-in[10]*9.5153+in[14]*1.5967;
out[11] = in[11]*(-4.143016944939305)+in[13]*4.143016944939305;
out[12] = sw_dot;

}

void rhs_eval( real_t *in, real_t *out ){

/* for manual input indexing ... */

const int minus_NU = 0;

/* optimized intermediate calculations */

const double t2 = cos(in[4]);
const double t8 = sin(in[4]);

const double alpha = -in[4]+in[7];

double Vsafe = in[3];
if (Vsafe<1.0) Vsafe = 1.0;

const double n_dot = in[32]+Vsafe*t2*cos(in[5]);
const double e_dot = in[33]+Vsafe*t2*sin(in[5]);
const double d_dot = in[34]-Vsafe*t8;

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
const int idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
bool b_switch_segment = false;
int pparam_sel = 0;
double sw_dot = 0.0;
if ( in[ACADO_NX-1] < 0.05 ) { // check x_sw
    const double vel[3] = {n_dot,e_dot,d_dot};
    if ( in[idx_OD_0] < 0.5 ) { // path type
        b_switch_segment = check_line_seg( &in[0], &vel[0], &in[idx_OD_0] );
    } else if (in[idx_OD_0] < 1.5 ) {
        b_switch_segment = check_curve_seg( &in[0], &vel[0], &in[idx_OD_0] );
    }
} else {
    b_switch_segment = true;
}
if (b_switch_segment) {
    pparam_sel = 7;
    sw_dot = 1.0;
} 

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double t3 = e_dot*e_dot;
const double t4 = n_dot*n_dot;
const double t5 = alpha-in[35]+in[37];
const double t6 = 1.0/(in[37]*in[37]);
const double t7 = -alpha+in[36]+in[37];

const double t9 = in[11]*in[11];
const double t10 = 1.0/Vsafe;
const double t11 = Vsafe*Vsafe;
const double t12 = alpha*alpha;
const double t13 = in[11]*8.61861E1;
const double t14 = in[11]*t9*2.501023E2;
const double t22 = t9*3.05322E1;
const double t15 = t13+t14-t22;
const double t16 = alpha*5.0996;
const double t17 = t12*(-5.343)+t16+1.7E1/3.2E1;
const double t18 = t11*t17*2.38875E-1;
const double t19 = cos(alpha);
const double t20 = 1.0/t19;
const double t21 = sin(alpha);
const double t23 = t10*t15*t20*t21;
const double t24 = t18+t23;
const double t25 = cos(in[6]);
const double t26 = sin(in[6]);

/* rhs */

out[0] = n_dot;
out[1] = e_dot;
out[2] = d_dot;
out[3] = t8*(-9.81E2/1.0E2)+t10*t15*(2.0E1/5.3E1)-t11*(alpha*2.5491E-1+t12*2.7337+6.4105E-2)*9.014150943396226E-2;
out[4] = -t10*(t2*(9.81E2/1.0E2)-t24*t25*(2.0E1/5.3E1));
out[5] = (t10*t24*t26*(2.0E1/5.3E1))/t2;
out[6] = in[8];
out[7] = in[9]*t25-in[10]*t26;
out[8] = in[6]*(-1.24716E1)-in[8]*7.4252+in[10]*1.0069+in[14]*1.24716E1;
out[9] = -t11*(alpha*1.9303E-1+in[7]*1.8359E-1+in[9]*4.6239E-2-in[15]*1.8359E-1-9.4955E-4);
out[10] = in[6]*5.7996-in[10]*9.5153+in[14]*1.5967;
out[11] = in[11]*(-4.143016944939305)+in[13]*4.143016944939305;
out[12] = sw_dot;

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

const double n_dot = in[32]+Vsafe*t2*cos(in[5]);
const double e_dot = in[33]+Vsafe*t2*sin(in[5]);
const double d_dot = in[34]-Vsafe*sin(in[4]);

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
const int idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
bool b_switch_segment = false;
int pparam_sel = 0;
double sw_dot = 0.0;
if ( in[ACADO_NX-1] < 0.05 ) { // check x_sw
    const double vel[3] = {n_dot,e_dot,d_dot};
    if ( in[idx_OD_0] < 0.5 ) { // path type
        b_switch_segment = check_line_seg( &in[0], &vel[0], &in[idx_OD_0] );
    } else if (in[idx_OD_0] < 1.5 ) {
        b_switch_segment = check_curve_seg( &in[0], &vel[0], &in[idx_OD_0] );
    }
} else {
    b_switch_segment = true;
}
if (b_switch_segment) {
    pparam_sel = 7;
    sw_dot = 1.0;
} 

double p_n = 0.0;
double p_e = 0.0;
double p_d = 0.0;
double tP_n = 1.0;
double tP_e = 0.0;
double tP_d = 0.0;

const double pparam_type = in[idx_OD_0+pparam_sel];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // calculate tangent
    double tP_n = cos(in[idx_OD_0+pparam_sel+6])*cos(in[idx_OD_0+pparam_sel+5]);
    double tP_e = cos(in[idx_OD_0+pparam_sel+6])*sin(in[idx_OD_0+pparam_sel+5]);
    double tP_d = -sin(in[idx_OD_0+pparam_sel+6]);
    
    // dot product
    const double dot_tP_bp = tP_n*(in[0] - in[idx_OD_0+pparam_sel+1]) + tP_e*(in[1] - in[idx_OD_0+pparam_sel+2]) + tP_d*(in[2] - in[idx_OD_0+pparam_sel+3]);
    
    // point on track
    p_n = in[idx_OD_0+pparam_sel+1] + dot_tP_bp * tP_n;
    p_e = in[idx_OD_0+pparam_sel+2] + dot_tP_bp * tP_e;
    p_d = in[idx_OD_0+pparam_sel+3] + dot_tP_bp * tP_d;
    
// ARC SEGMENT
} else if ( pparam_type < 1.5 ) {

// variable definitions
//     const double pparam_cc_n = in[idx_OD_0+pparam_sel+1];
//     const double pparam_cc_e = in[idx_OD_0+pparam_sel+2];
//     const double pparam_cc_d = in[idx_OD_0+pparam_sel+3];
//     const double pparam_R = fabs(in[idx_OD_0+pparam_sel+4]);
    const double pparam_ldir = (in[idx_OD_0+pparam_sel+4]<0.0) ? -1.0 : 1.0;
//     const double pparam_Chi = in[idx_OD_0+pparam_sel+5];
//     const double pparam_Gam = in[idx_OD_0+pparam_sel+6];
    double Gam_temp = in[idx_OD_0+pparam_sel+6];

    // calculate closest point on loiter circle
    const double cr_n = in[0] - in[idx_OD_0+pparam_sel+1];
    const double cr_e = in[1] - in[idx_OD_0+pparam_sel+2];
    const double norm_cr = sqrt( cr_n*cr_n + cr_e*cr_e );
    double cr_n_unit;
    double cr_e_unit;
    if (norm_cr>0.1) {
        cr_n_unit = cr_n / norm_cr;
        cr_e_unit = cr_e / norm_cr;
    }
    else {
        cr_n_unit = 0.0;
        cr_e_unit = 0.0;
    }
    p_n = fabs(in[idx_OD_0+pparam_sel+4]) * cr_n_unit + in[idx_OD_0+pparam_sel+1];
    p_e = fabs(in[idx_OD_0+pparam_sel+4]) * cr_e_unit + in[idx_OD_0+pparam_sel+2];

    // calculate tangent
    tP_n = pparam_ldir * -cr_e_unit;
    tP_e = pparam_ldir * cr_n_unit;
    
    // angular position
    const double xi_pos = atan2(cr_e_unit, cr_n_unit);
    
    // angular exit
    double xi_exit = in[idx_OD_0+pparam_sel+5] - pparam_ldir * 1.570796326794897;
    if (xi_exit>3.141592653589793) {
        xi_exit = xi_exit - 6.283185307179586;
    }
    else if (xi_exit<-3.141592653589793) {
        xi_exit = xi_exit + 6.283185307179586;
    }
    
    // angular travel (back calculated) from exit [0,2pi)
    double delta_xi = pparam_ldir * (xi_exit - xi_pos);
    if (delta_xi >= 6.28318530718) delta_xi = 0.0;
    if (delta_xi < 0.0) delta_xi = delta_xi + 6.28318530718;

    // closest point on nearest spiral leg and tangent down component
    if (fabs(in[idx_OD_0+pparam_sel+6]) < 0.001) {

        p_d = in[idx_OD_0+pparam_sel+3];
        tP_d = 0.0;

    } else {

        const double RtanGam = fabs(in[idx_OD_0+pparam_sel+4]) * tan(in[idx_OD_0+pparam_sel+6]);

        // height down from exit
        const double delta_d_xi = delta_xi * RtanGam;

        // nearest spiral leg
        const double delta_d_k = round( (in[2] - (in[idx_OD_0+pparam_sel+3] + delta_d_xi)) / (6.28318530718*RtanGam) ) * 6.28318530718*RtanGam;
        
        // closest point on nearest spiral leg
        p_d = in[idx_OD_0+pparam_sel+3] + delta_d_k + delta_d_xi;
        
        // cap end point
        if ((p_d - in[idx_OD_0+pparam_sel+3]) * in[idx_OD_0+pparam_sel+6] < 0.0) {
            p_d = in[idx_OD_0+pparam_sel+3];
            tP_d = 0.0;
            Gam_temp = 0.0;
        }
        else {
            tP_d = -sin(in[idx_OD_0+pparam_sel+6]);
        }
        
    }
    
    if (fabs(tP_n)<0.01 && fabs(tP_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        tP_n=1.0;
        tP_e=0.0;
    }
    
    // Normalize tP
    tP_n = tP_n * cos(Gam_temp);
    tP_e = tP_e * cos(Gam_temp);
    
// LOITER UNLIM
} else if ( pparam_type < 2.5 ) {

    const double pparam_ldir = (in[idx_OD_0+pparam_sel+4]<0.0) ? -1.0 : 1.0;

    // calculate closest point on loiter circle
    const double cr_n = in[0] - in[idx_OD_0+pparam_sel+1];
    const double cr_e = in[1] - in[idx_OD_0+pparam_sel+2];
    const double norm_cr = sqrt( cr_n*cr_n + cr_e*cr_e );
    double cr_n_unit;
    double cr_e_unit;
    if (norm_cr>0.1) {
        cr_n_unit = cr_n / norm_cr;
        cr_e_unit = cr_e / norm_cr;
    }
    else {
        cr_n_unit = 0.0;
        cr_e_unit = 0.0;
    }
    p_n = fabs(in[idx_OD_0+pparam_sel+4]) * cr_n_unit + in[idx_OD_0+pparam_sel+1];
    p_e = fabs(in[idx_OD_0+pparam_sel+4]) * cr_e_unit + in[idx_OD_0+pparam_sel+2];

    // calculate tangent
    tP_n = pparam_ldir * -cr_e_unit;
    tP_e = pparam_ldir * cr_n_unit;
    
    p_d = in[idx_OD_0+pparam_sel+3];
    tP_d = 0.0;
    
    if (fabs(tP_n)<0.01 && fabs(tP_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        tP_n=1.0;
        tP_e=0.0;
    }
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double t3 = e_dot*e_dot;
const double t4 = n_dot*n_dot;
const double t5 = alpha-in[35]+in[37];
const double t6 = 1.0/(in[37]*in[37]);
const double t7 = -alpha+in[36]+in[37];

const double norm_vG_lat = sqrt(t3+t4);
double e_v_lat;
if (norm_vG_lat<1.0) {
    e_v_lat = sqrt(norm_vG_lat-n_dot*tP_n-e_dot*tP_e);
}
else {
    e_v_lat = sqrt((norm_vG_lat*(1.0/2.0)-n_dot*tP_n*(1.0/2.0)-e_dot*tP_e*(1.0/2.0))/norm_vG_lat);
}

const double norm_vG_lon = sqrt(t3+t4+d_dot*d_dot);
double e_v_lon;
if (norm_vG_lon<1.0) {
    e_v_lon = sqrt(norm_vG_lon-sqrt(tP_e*tP_e+tP_n*tP_n)*norm_vG_lat-d_dot*tP_d);
}
else {
    e_v_lon = sqrt((norm_vG_lon*(1.0/2.0)-norm_vG_lat*sqrt(tP_e*tP_e+tP_n*tP_n)*(1.0/2.0)-d_dot*tP_d*(1.0/2.0))/norm_vG_lon);
}

double a_soft;
if (alpha>(in[35]-in[37])) {
    a_soft=(t5*t5)*t6;
}
else if (alpha>(in[36]+in[37])) {
    a_soft=0.0;
}
else {
    a_soft=t6*(t7*t7);
}

/* outputs */

out[0] = -tP_n*(in[1]-p_e)+tP_e*(in[0]-p_n);
out[1] = -in[2]+p_d;
out[2] = e_v_lat;
out[3] = e_v_lon;
out[4] = Vsafe;
out[5] = in[8];
out[6] = in[9];
out[7] = in[10];
out[8] = a_soft;
out[9] = in[11]*(-4.143016944939305)+in[13]*4.143016944939305;
out[10] = in[13];
out[11] = in[14];
out[12] = in[15];

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

const double n_dot = in[29]+Vsafe*t2*cos(in[5]);
const double e_dot = in[30]+Vsafe*t2*sin(in[5]);
const double d_dot = in[31]-Vsafe*sin(in[4]);

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
const int idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
bool b_switch_segment = false;
int pparam_sel = 0;
double sw_dot = 0.0;
if ( in[ACADO_NX-1] < 0.05 ) { // check x_sw
    const double vel[3] = {n_dot,e_dot,d_dot};
    if ( in[idx_OD_0] < 0.5 ) { // path type
        b_switch_segment = check_line_seg( &in[0], &vel[0], &in[idx_OD_0] );
    } else if (in[idx_OD_0] < 1.5 ) {
        b_switch_segment = check_curve_seg( &in[0], &vel[0], &in[idx_OD_0] );
    }
} else {
    b_switch_segment = true;
}
if (b_switch_segment) {
    pparam_sel = 7;
    sw_dot = 1.0;
} 

double p_n = 0.0;
double p_e = 0.0;
double p_d = 0.0;
double tP_n = 1.0;
double tP_e = 0.0;
double tP_d = 0.0;

const double pparam_type = in[idx_OD_0+pparam_sel];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // calculate tangent
    double tP_n = cos(in[idx_OD_0+pparam_sel+6])*cos(in[idx_OD_0+pparam_sel+5]);
    double tP_e = cos(in[idx_OD_0+pparam_sel+6])*sin(in[idx_OD_0+pparam_sel+5]);
    double tP_d = -sin(in[idx_OD_0+pparam_sel+6]);
    
    // dot product
    const double dot_tP_bp = tP_n*(in[0] - in[idx_OD_0+pparam_sel+1]) + tP_e*(in[1] - in[idx_OD_0+pparam_sel+2]) + tP_d*(in[2] - in[idx_OD_0+pparam_sel+3]);
    
    // point on track
    p_n = in[idx_OD_0+pparam_sel+1] + dot_tP_bp * tP_n;
    p_e = in[idx_OD_0+pparam_sel+2] + dot_tP_bp * tP_e;
    p_d = in[idx_OD_0+pparam_sel+3] + dot_tP_bp * tP_d;
    
// ARC SEGMENT
} else if ( pparam_type < 1.5 ) {

// variable definitions
//     const double pparam_cc_n = in[idx_OD_0+pparam_sel+1];
//     const double pparam_cc_e = in[idx_OD_0+pparam_sel+2];
//     const double pparam_cc_d = in[idx_OD_0+pparam_sel+3];
//     const double pparam_R = fabs(in[idx_OD_0+pparam_sel+4]);
    const double pparam_ldir = (in[idx_OD_0+pparam_sel+4]<0.0) ? -1.0 : 1.0;
//     const double pparam_Chi = in[idx_OD_0+pparam_sel+5];
//     const double pparam_Gam = in[idx_OD_0+pparam_sel+6];
    double Gam_temp = in[idx_OD_0+pparam_sel+6];

    // calculate closest point on loiter circle
    const double cr_n = in[0] - in[idx_OD_0+pparam_sel+1];
    const double cr_e = in[1] - in[idx_OD_0+pparam_sel+2];
    const double norm_cr = sqrt( cr_n*cr_n + cr_e*cr_e );
    double cr_n_unit;
    double cr_e_unit;
    if (norm_cr>0.1) {
        cr_n_unit = cr_n / norm_cr;
        cr_e_unit = cr_e / norm_cr;
    }
    else {
        cr_n_unit = 0.0;
        cr_e_unit = 0.0;
    }
    p_n = fabs(in[idx_OD_0+pparam_sel+4]) * cr_n_unit + in[idx_OD_0+pparam_sel+1];
    p_e = fabs(in[idx_OD_0+pparam_sel+4]) * cr_e_unit + in[idx_OD_0+pparam_sel+2];

    // calculate tangent
    tP_n = pparam_ldir * -cr_e_unit;
    tP_e = pparam_ldir * cr_n_unit;
    
    // angular position
    const double xi_pos = atan2(cr_e_unit, cr_n_unit);
    
    // angular exit
    double xi_exit = in[idx_OD_0+pparam_sel+5] - pparam_ldir * 1.570796326794897;
    if (xi_exit>3.141592653589793) {
        xi_exit = xi_exit - 6.283185307179586;
    }
    else if (xi_exit<-3.141592653589793) {
        xi_exit = xi_exit + 6.283185307179586;
    }
    
    // angular travel (back calculated) from exit [0,2pi)
    double delta_xi = pparam_ldir * (xi_exit - xi_pos);
    if (delta_xi >= 6.28318530718) delta_xi = 0.0;
    if (delta_xi < 0.0) delta_xi = delta_xi + 6.28318530718;

    // closest point on nearest spiral leg and tangent down component
    if (fabs(in[idx_OD_0+pparam_sel+6]) < 0.001) {

        p_d = in[idx_OD_0+pparam_sel+3];
        tP_d = 0.0;

    } else {

        const double RtanGam = fabs(in[idx_OD_0+pparam_sel+4]) * tan(in[idx_OD_0+pparam_sel+6]);

        // height down from exit
        const double delta_d_xi = delta_xi * RtanGam;

        // nearest spiral leg
        const double delta_d_k = round( (in[2] - (in[idx_OD_0+pparam_sel+3] + delta_d_xi)) / (6.28318530718*RtanGam) ) * 6.28318530718*RtanGam;
        
        // closest point on nearest spiral leg
        p_d = in[idx_OD_0+pparam_sel+3] + delta_d_k + delta_d_xi;
        
        // cap end point
        if ((p_d - in[idx_OD_0+pparam_sel+3]) * in[idx_OD_0+pparam_sel+6] < 0.0) {
            p_d = in[idx_OD_0+pparam_sel+3];
            tP_d = 0.0;
            Gam_temp = 0.0;
        }
        else {
            tP_d = -sin(in[idx_OD_0+pparam_sel+6]);
        }
        
    }
    
    if (fabs(tP_n)<0.01 && fabs(tP_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        tP_n=1.0;
        tP_e=0.0;
    }
    
    // Normalize tP
    tP_n = tP_n * cos(Gam_temp);
    tP_e = tP_e * cos(Gam_temp);
    
// LOITER UNLIM
} else if ( pparam_type < 2.5 ) {

    const double pparam_ldir = (in[idx_OD_0+pparam_sel+4]<0.0) ? -1.0 : 1.0;

    // calculate closest point on loiter circle
    const double cr_n = in[0] - in[idx_OD_0+pparam_sel+1];
    const double cr_e = in[1] - in[idx_OD_0+pparam_sel+2];
    const double norm_cr = sqrt( cr_n*cr_n + cr_e*cr_e );
    double cr_n_unit;
    double cr_e_unit;
    if (norm_cr>0.1) {
        cr_n_unit = cr_n / norm_cr;
        cr_e_unit = cr_e / norm_cr;
    }
    else {
        cr_n_unit = 0.0;
        cr_e_unit = 0.0;
    }
    p_n = fabs(in[idx_OD_0+pparam_sel+4]) * cr_n_unit + in[idx_OD_0+pparam_sel+1];
    p_e = fabs(in[idx_OD_0+pparam_sel+4]) * cr_e_unit + in[idx_OD_0+pparam_sel+2];

    // calculate tangent
    tP_n = pparam_ldir * -cr_e_unit;
    tP_e = pparam_ldir * cr_n_unit;
    
    p_d = in[idx_OD_0+pparam_sel+3];
    tP_d = 0.0;
    
    if (fabs(tP_n)<0.01 && fabs(tP_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        tP_n=1.0;
        tP_e=0.0;
    }
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double t3 = e_dot*e_dot;
const double t4 = n_dot*n_dot;
const double t5 = alpha-in[32]+in[34];
const double t6 = 1.0/(in[34]*in[34]);
const double t7 = -alpha+in[33]+in[34];

const double norm_vG_lat = sqrt(t3+t4);
double e_v_lat;
if (norm_vG_lat<1.0) {
    e_v_lat = sqrt(norm_vG_lat-n_dot*tP_n-e_dot*tP_e);
}
else {
    e_v_lat = sqrt((norm_vG_lat*(1.0/2.0)-n_dot*tP_n*(1.0/2.0)-e_dot*tP_e*(1.0/2.0))/norm_vG_lat);
}

const double norm_vG_lon = sqrt(t3+t4+d_dot*d_dot);
double e_v_lon;
if (norm_vG_lon<1.0) {
    e_v_lon = sqrt(norm_vG_lon-sqrt(tP_e*tP_e+tP_n*tP_n)*norm_vG_lat-d_dot*tP_d);
}
else {
    e_v_lon = sqrt((norm_vG_lon*(1.0/2.0)-norm_vG_lat*sqrt(tP_e*tP_e+tP_n*tP_n)*(1.0/2.0)-d_dot*tP_d*(1.0/2.0))/norm_vG_lon);
}

double a_soft;
if (alpha>(in[32]-in[34])) {
    a_soft=(t5*t5)*t6;
}
else if (alpha>(in[33]+in[34])) {
    a_soft=0.0;
}
else {
    a_soft=t6*(t7*t7);
}


/* outputs */

out[0] = -tP_n*(in[1]-p_e)+tP_e*(in[0]-p_n);
out[1] = -in[2]+p_d;
out[2] = sqrt((norm_vG_lat*(1.0/2.0)-e_dot*tP_n*(1.0/2.0)-n_dot*tP_e*(1.0/2.0))/norm_vG_lat);
out[3] = sqrt((norm_vG_lon*(1.0/2.0)-d_dot*sqrt(tP_e*tP_e+tP_n*tP_n)*(1.0/2.0)-norm_vG_lat*tP_d*(1.0/2.0))/norm_vG_lon);
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
    
    // tB
    const double tB_n = cos(params[6])*cos(params[5]);
    const double tB_e = cos(params[6])*sin(params[5]);
    const double tB_d = -sin(params[6]);

    // r - b
    const double br_n = pos[0] - params[1];
    const double br_e = pos[1] - params[2];
    const double br_d = pos[2] - params[3];
    
    // dot( (r-b) , tB )
    const double dot_brtB = br_n * tB_n + br_e * tB_e + br_d * tB_d;
    
    // check travel 
    return ( dot_brtB > 0.0 );
    
}

bool check_curve_seg( const double *pos, const double *vel, const double *params ) {
    
    // tB
    const double tB_n = cos(params[6])*cos(params[5]);
    const double tB_e = cos(params[6])*sin(params[5]);
    const double tB_d = -sin(params[6]);

    // r - b
    const double rot90 = (params[4]<0.0) ? -1.570796326794897 : 1.570796326794897;
    const double br_n = pos[0] - (params[1] + fabs(params[4]) * cos(params[5] - rot90));
    const double br_e = pos[1] - (params[2] + fabs(params[4]) * sin(params[5] - rot90));
    const double br_d = pos[2] - params[3];
    
    // bearing : dot( v , tB ) (lat)
    const double dot_vtB = vel[0] * tB_n + vel[1] * tB_e;
    
    // travel : dot( (r-b) , tB ) (lat)
    const double dot_brtB = br_n * tB_n + br_e * tB_e;
    
    // proximity : norm( r-b ) (lat)
    const double norm_br = sqrt( br_n*br_n + br_e*br_e );
    
    // proximity : norm( r-b ) (lon)
    const double norm_br_d = fabs(br_d);
    
    // check (1) proximity, (2) bearing, (3) travel 
    return ( norm_br < params[14] && norm_br_d < 10.0 && dot_vtB > params[15]*sqrt(vel[0]*vel[0]+vel[1]*vel[1]) && dot_brtB > 0.0 );
}

/* end inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */