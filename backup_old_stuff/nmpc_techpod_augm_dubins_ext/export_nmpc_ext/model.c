#include "acado_common.h"
#include <math.h>
#include <string.h>

void rhs( const real_t *in, real_t *out ){

/* optimized intermediate calculations */

const double t2 = sin(in[6]);
const double t3 = sin(in[8]);
const double t4 = cos(in[6]);
const double t5 = cos(in[8]);
const double t6 = sin(in[7]);
const double t8 = cos(in[7]);
const double t11 = t3*t4;
const double t12 = t2*t5*t6;
const double t13 = t11-t12;
const double t14 = in[1]*t13;
const double t15 = t2*t3;
const double t16 = t4*t5*t6;
const double t17 = t15+t16;
const double t18 = in[2]*t17;
const double t19 = in[0]*t5*t8;
const double t7 = in[46]-t14+t18+t19;
const double t21 = t4*t5;
const double t22 = t2*t3*t6;
const double t23 = t21+t22;
const double t24 = in[1]*t23;
const double t25 = t2*t5;
const double t26 = t3*t4*t6;
const double t27 = t25-t26;
const double t28 = in[2]*t27;
const double t29 = in[0]*t3*t8;
const double t9 = in[47]+t24-t28+t29;
const double t31 = in[0]*t6;
const double t32 = in[2]*t4*t8;
const double t33 = in[1]*t2*t8;
const double t10 = in[48]-t31+t32+t33;
const double t20 = t7*t7;
const double t30 = t9*t9;
const double t34 = 1.0/in[0];
const double t35 = in[2]*t34;
const double t36 = atan(t35);
const double t37 = 1.0/(in[0]*in[0]);
const double t38 = in[2]*in[2];
const double t39 = t37*t38;
const double t40 = t39+1.0;
const double t41 = 1.0/sqrt(t40);
const double t42 = t36*t36;
const double t43 = in[0]*in[0];
const double t44 = in[1]*in[1];
const double t45 = t43*2.773E-1;
const double t46 = t44*2.773E-1;
const double t47 = t38*2.773E-1;
const double t48 = t45+t46+t47;
const double t49 = t36*1.0852E1;
const double t50 = t42*3.49278E1;
const double t51 = t36*t42*3.45338E1;
const double t52 = t49-t50+t51-1.1754E-1;
const double t53 = t36*4.6194E-1;
const double t54 = t42*1.1993;
const double t55 = t53+t54+6.2874E-2;
const double t56 = t38+t43+t44;
const double t57 = 1.0/sqrt(t56);
const double t58 = in[1]*t57;
const double t59 = asin(t58);
const double t60 = in[23]*6.0945E-2;
const double t61 = in[3]*t57*1.08589635E-1;
const double t62 = in[5]*t57*1.0706801E-1;
const double t74 = t59*4.2987E-2;
const double t63 = t60+t61+t62-t74;
const double t64 = in[22]*5.7723E-2;
const double t65 = t59*1.5419E-2;
const double t66 = in[3]*t57*2.1327355E-1;
const double t72 = in[5]*t57*1.5132075E-2;
const double t67 = t64+t65+t66-t72;
const double t68 = t43*7.18207E-1;
const double t69 = t44*7.18207E-1;
const double t70 = t38*7.18207E-1;
const double t71 = t68+t69+t70;
const double t73 = t41*t67;
const double t75 = in[2]*t34*t41*t63;
const double t76 = in[2]*t34*t41*t67;
const double t77 = t75+t76;
const double t78 = in[5]*t4;
const double t79 = in[4]*t2;
const double t80 = t78+t79;

/* augmented guidance logic */

const double V_lat      = sqrt(t20+t30);
const double V_lon      = sqrt(t20+t30+t10*t10);

double LL_lon_ne        = 1.0;
double LL_lon_d         = 0.0;
double LL_lat_n         = 1.0;
double LL_lat_e         = 0.0;
double L1R_lon          = 1.0;
double L1R_lat          = 1.0;

if (fabs(in[33]) < 0.5) { // line

    // calculate vector from waypoint a to b
    const double aa_bb_n        = in[37] - in[34];
    const double aa_bb_e        = in[38] - in[35];
    const double aa_bb_d        = in[39] - in[36];

    const double normaa_bb      = sqrt( aa_bb_n*aa_bb_n + aa_bb_e*aa_bb_e + aa_bb_d*aa_bb_d );
    const double aa_bb_unit_n   = aa_bb_n / normaa_bb;
    const double aa_bb_unit_e   = aa_bb_e / normaa_bb;
    const double aa_bb_unit_d   = aa_bb_d / normaa_bb;

    // calculate closest point on line a->b
    const double aa_pp_n        = in[9] - in[34];
    const double aa_pp_e        = in[10] - in[35];
    const double aa_pp_d        = in[11] - in[36];

    const double pp_proj        = aa_bb_unit_n*aa_pp_n + aa_bb_unit_e*aa_pp_e + aa_bb_unit_d*aa_pp_d;

    const double dd_n           = in[34] + pp_proj * aa_bb_unit_n;
    const double dd_e           = in[35] + pp_proj * aa_bb_unit_e;
    const double dd_d           = in[36] + pp_proj * aa_bb_unit_d;

    // longitudinal / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

    // calculate longitudinal track error
    const double aa_bb_lon_unit_ne   = sqrt( aa_bb_unit_n*aa_bb_unit_n + aa_bb_unit_e*aa_bb_unit_e );
    const double aa_bb_lon_unit_d    = aa_bb_unit_d;

    const double pp_dd_d             = dd_d - in[11];
    const double xtrackerr_lon       = pp_dd_d / aa_bb_lon_unit_ne;

    // calculate the L1 length required for the desired period
    L1R_lon                         = in[25] * in[26] / 3.14159265359;
    double L1_lon                   = L1R_lon * V_lon;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lon          = fabs(xtrackerr_lon);
    if (L1_lon < L1min_lon) { 
        L1_lon = L1min_lon;
    }

    // calculate L1 vector
    const double normdd_rr_lon      = sqrt(L1_lon*L1_lon - xtrackerr_lon*xtrackerr_lon);
    const double rr_lon_ne          = aa_bb_lon_unit_ne * normdd_rr_lon;
    const double rr_lon_d           = aa_bb_lon_unit_d * normdd_rr_lon + pp_dd_d;
    LL_lon_ne                       = rr_lon_ne - pp_dd_d * aa_bb_lon_unit_d / aa_bb_lon_unit_ne;
    LL_lon_d                        = rr_lon_d;

    // lateral / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

    const double normaa_bb_lat       = sqrt( aa_bb_n*aa_bb_n + aa_bb_e*aa_bb_e );
    const double aa_bb_lat_unit_n    = aa_bb_n / normaa_bb_lat;
    const double aa_bb_lat_unit_e    = aa_bb_e / normaa_bb_lat;

    // calculate closest point on line a->b
    const double pp_proj_lat        = aa_bb_lat_unit_n*aa_pp_n + aa_bb_lat_unit_e*aa_pp_e;

    const double dd_lat_n           = in[34] + pp_proj_lat * aa_bb_lat_unit_n;
    const double dd_lat_e           = in[35] + pp_proj_lat * aa_bb_lat_unit_e;

    // calculate lateral track error
    const double normdd_pp_lat      = sqrt( (in[9] - dd_lat_n)*(in[9] - dd_lat_n) + (in[10] - dd_lat_e)*(in[10] - dd_lat_e) );

    // calculate the L1 length required for the desired period
    L1R_lat                         = in[27] * in[28] / 3.14159265359;
    double L1_lat                   = L1R_lat * V_lat;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lat          = normdd_pp_lat;
    if (L1_lat < L1min_lat) {
        L1_lat = L1min_lat;
    }

    // calculate L1 vector
    const double normdd_rr_lat      = sqrt(L1_lat*L1_lat - normdd_pp_lat*normdd_pp_lat);
    const double rr_lat_n           = aa_bb_lat_unit_n * normdd_rr_lat + dd_lat_n;
    const double rr_lat_e           = aa_bb_lat_unit_e * normdd_rr_lat + dd_lat_e;
    LL_lat_n                        = rr_lat_n - in[9];
    LL_lat_e                        = rr_lat_e - in[10];

} else if ( fabs(in[33]-1.0) < 0.5 ) { // spiral

	// lateral / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    
    // calculate the distance from the aircraft to the circle
    double cc_pp_n     = in[9] - in[34];
    double cc_pp_e     = in[10] - in[35];
    
    if ( fabs(cc_pp_n) < 0.1 && fabs(cc_pp_e) < 0.1 ) { // cannot be in center of circle
        cc_pp_n = 0.1;
        cc_pp_e = 0.1;
    }

    const double normcc_pp2  = cc_pp_n*cc_pp_n + cc_pp_e*cc_pp_e;
    const double normcc_pp   = sqrt(normcc_pp2);

    const double pp_dd       = normcc_pp - in[37];

    // check circle tracking feasibility, calculate L1 ratio
    const double R08_V              = in[37] / V_lat * 0.5;
    L1R_lat                         = in[28] * in[27] / 3.14159265359;
    L1R_lat                         = (L1R_lat < R08_V) ? R08_V : L1R_lat;

    // calculate the L1 length required for the desired period
    double L1_lat                   = L1R_lat * V_lat;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lat          = fabs(pp_dd);
    const double L1max_lat          = 2.0*in[37] + pp_dd;
    if (L1_lat > L1max_lat) {
        L1_lat  = L1max_lat;
    } else if (L1_lat < L1min_lat) {
        L1_lat  = L1min_lat;
    }

    // calculate lateral components of L1 vector
    double cosgamL1                 = (L1_lat*L1_lat + normcc_pp2 - in[37]*in[37]) / 2.0 / L1_lat / normcc_pp;
    if (cosgamL1 > 1.0) {
        cosgamL1 = 1.0;
    }
    if (cosgamL1 < -1.0) {
        cosgamL1 = -1.0;
    }
    const double gamL1              = acos(cosgamL1);
    const double b_pp               = atan2(-cc_pp_e,-cc_pp_n);
    double b_L1                     = b_pp - in[38] * gamL1;
    if (b_L1 > 3.14159265359) {
        b_L1 = b_L1 - 6.28318530718;
    }
    if (b_L1 < -3.14159265359) {
        b_L1 = b_L1 + 6.28318530718;
    }

    LL_lat_n                        = L1_lat * cos(b_L1);
    LL_lat_e                        = L1_lat * sin(b_L1);

    // longitudinal / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

    // spiral angular position: [0,2*pi)
    const double xi                 = atan2(cc_pp_e, cc_pp_n);
    double delta_xi = xi-in[40];
    if (in[38]>0.0 && in[40]>xi) {
        delta_xi                    = delta_xi + 6.28318530718;
    } else if (in[38]<0.0 && xi>in[40]) {
        delta_xi                    = delta_xi - 6.28318530718;
    }

    // closest point on nearest spiral leg
    double dd_d_sp;
    if (fabs(in[39]) < 0.001) {
        
        dd_d_sp                         = in[36];
    
    } else {
        
        const double Rtangam            = in[37] * tan(in[39]);
        
        // spiral height delta for current angle
        const double delta_d_xi         = -delta_xi * in[38] * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end     = -in[41] * Rtangam;

        // nearest spiral leg
        double delta_d_k            = round( (in[11] - (in[36] + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
        const double delta_d_end_k  = round( (delta_d_sp_end - (in[36] + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        // check
        if (delta_d_k * in[39] > 0.0) { //NOTE: gam is actually being used for its sign, but writing a sign operator doesnt make a difference here
            delta_d_k = 0.0;
        } else if (fabs(delta_d_k) > fabs(delta_d_end_k) ) {
            delta_d_k = (delta_d_k < 0.0) ? -fabs(delta_d_end_k) : fabs(delta_d_end_k);
        }
        
        // closest point on nearest spiral leg
        const double delta_d_sp         = delta_d_k + delta_d_xi;
        dd_d_sp                         = in[36] + delta_d_sp;
    }

    // calculate longitudinal track error
    const double aa_bb_lon_unit_ne      = cos(in[39]);
    const double aa_bb_lon_unit_d       = -sin(in[39]);
    const double pp_dd_d                = dd_d_sp - in[11];
    const double xtrackerr_lon          = pp_dd_d / aa_bb_lon_unit_ne;

    // calculate the L1 length required for the desired period
    L1R_lon                 = in[25] * in[26] / 3.14159265359;
    double L1_lon           = L1R_lon * V_lon;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lon  = fabs(xtrackerr_lon);
    if (L1_lon < L1min_lon) {
        L1_lon = L1min_lon;
    }

    // calculate L1 vector
    const double normdd_rr_lon  = sqrt(L1_lon*L1_lon - xtrackerr_lon*xtrackerr_lon);
    const double rr_lon_ne      = aa_bb_lon_unit_ne * normdd_rr_lon;
    const double rr_lon_d       = aa_bb_lon_unit_d * normdd_rr_lon + pp_dd_d;
    LL_lon_ne                   = rr_lon_ne - pp_dd_d * aa_bb_lon_unit_d / aa_bb_lon_unit_ne;
    LL_lon_d                    = rr_lon_d;
    
}

const double atan2_01 = atan2(t10, V_lat);
const double atan2_02 = atan2(LL_lon_d, LL_lon_ne);
double etalon = atan2_01-atan2_02;
if (etalon>3.14159265359) etalon = etalon - 6.28318530718;
if (etalon<-3.14159265359) etalon = etalon + 6.28318530718;

const double atan2_03 = atan2(LL_lat_e, LL_lat_n);
const double atan2_04 = atan2(t9, t7);
double etalat = atan2_03-atan2_04;
if (etalat>3.14159265359) etalat = etalat - 6.28318530718;
if (etalat<-3.14159265359) etalat = etalat + 6.28318530718;

/* rhs */

out[0] = in[20]*4.713471698113208-t6*(9.81E2/1.0E2)+in[1]*in[5]-in[2]*in[4]-t48*(t41*t55-in[2]*t34*t41*t52)*(2.0E1/5.3E1);
out[1] = -in[0]*in[5]+in[2]*in[3]+t2*t8*(9.81E2/1.0E2)-t48*t59*1.159735849056604E-1;
out[2] = in[0]*in[4]-in[1]*in[3]+t4*t8*(9.81E2/1.0E2)-t48*(t41*t52+in[2]*t34*t41*t55)*(2.0E1/5.3E1);
out[3] = t71*t77*(-9.263652961916226E-1)+in[4]*(in[3]*2.270436E-2-in[5]*7.616616999999998E-2)*1.22697390224056E1-t71*(t73-in[2]*t34*t41*t63)*6.433024169447254;
out[4] = in[3]*in[5]*9.181328545780969E-1-(in[3]*in[3])*1.93639394716594E-1+(in[5]*in[5])*1.93639394716594E-1-(t38*4.9914E-2+t43*4.9914E-2+t44*4.9914E-2)*(in[21]*3.5262+t36*2.0621+in[4]*t57*4.497507-9.8911E-2)*2.564760194921775;
out[5] = t71*t77*(-2.040702994206499)-t71*(t73-t75)*9.263652961916226E-1-in[4]*(in[3]*3.148557560000001E-2+in[5]*2.270436E-2)*1.22697390224056E1;
out[6] = in[3]+t80*tan(in[7]);
out[7] = -in[5]*t2+in[4]*t4;
out[8] = t80/t8;
out[9] = t7;
out[10] = t9;
out[11] = t10;
out[12] = etalon*in[29];
out[13] = etalat*in[30];
out[14] = in[31]*(in[32]-sqrt(t56));
out[15] = in[15]*in[42]+in[20]*in[43];
out[16] = in[16]*in[42]+in[21]*in[43];
out[17] = in[17]*in[42]+in[22]*in[43];
out[18] = in[18]*in[42]+in[23]*in[43];
out[19] = in[19]*in[42]+in[24]*in[43];

}

void rhs_eval( real_t *in, real_t *out ){

/* optimized intermediate calculations */

const double t2 = sin(in[6]);
const double t3 = sin(in[8]);
const double t4 = cos(in[6]);
const double t5 = cos(in[8]);
const double t6 = sin(in[7]);
const double t8 = cos(in[7]);
const double t11 = t3*t4;
const double t12 = t2*t5*t6;
const double t13 = t11-t12;
const double t14 = in[1]*t13;
const double t15 = t2*t3;
const double t16 = t4*t5*t6;
const double t17 = t15+t16;
const double t18 = in[2]*t17;
const double t19 = in[0]*t5*t8;
const double t7 = in[46]-t14+t18+t19;
const double t21 = t4*t5;
const double t22 = t2*t3*t6;
const double t23 = t21+t22;
const double t24 = in[1]*t23;
const double t25 = t2*t5;
const double t26 = t3*t4*t6;
const double t27 = t25-t26;
const double t28 = in[2]*t27;
const double t29 = in[0]*t3*t8;
const double t9 = in[47]+t24-t28+t29;
const double t31 = in[0]*t6;
const double t32 = in[2]*t4*t8;
const double t33 = in[1]*t2*t8;
const double t10 = in[48]-t31+t32+t33;
const double t20 = t7*t7;
const double t30 = t9*t9;
const double t34 = 1.0/in[0];
const double t35 = in[2]*t34;
const double t36 = atan(t35);
const double t37 = 1.0/(in[0]*in[0]);
const double t38 = in[2]*in[2];
const double t39 = t37*t38;
const double t40 = t39+1.0;
const double t41 = 1.0/sqrt(t40);
const double t42 = t36*t36;
const double t43 = in[0]*in[0];
const double t44 = in[1]*in[1];
const double t45 = t43*2.773E-1;
const double t46 = t44*2.773E-1;
const double t47 = t38*2.773E-1;
const double t48 = t45+t46+t47;
const double t49 = t36*1.0852E1;
const double t50 = t42*3.49278E1;
const double t51 = t36*t42*3.45338E1;
const double t52 = t49-t50+t51-1.1754E-1;
const double t53 = t36*4.6194E-1;
const double t54 = t42*1.1993;
const double t55 = t53+t54+6.2874E-2;
const double t56 = t38+t43+t44;
const double t57 = 1.0/sqrt(t56);
const double t58 = in[1]*t57;
const double t59 = asin(t58);
const double t60 = in[23]*6.0945E-2;
const double t61 = in[3]*t57*1.08589635E-1;
const double t62 = in[5]*t57*1.0706801E-1;
const double t74 = t59*4.2987E-2;
const double t63 = t60+t61+t62-t74;
const double t64 = in[22]*5.7723E-2;
const double t65 = t59*1.5419E-2;
const double t66 = in[3]*t57*2.1327355E-1;
const double t72 = in[5]*t57*1.5132075E-2;
const double t67 = t64+t65+t66-t72;
const double t68 = t43*7.18207E-1;
const double t69 = t44*7.18207E-1;
const double t70 = t38*7.18207E-1;
const double t71 = t68+t69+t70;
const double t73 = t41*t67;
const double t75 = in[2]*t34*t41*t63;
const double t76 = in[2]*t34*t41*t67;
const double t77 = t75+t76;
const double t78 = in[5]*t4;
const double t79 = in[4]*t2;
const double t80 = t78+t79;

/* augmented guidance logic */

const double V_lat = sqrt(t20+t30);
const double V_lon = sqrt(t20+t30+t10*t10);

double LL_lon_ne        = 1.0;
double LL_lon_d         = 0.0;
double LL_lat_n         = 1.0;
double LL_lat_e         = 0.0;
double L1R_lon          = 1.0;
double L1R_lat          = 1.0;

if (fabs(in[33]) < 0.5) { // line

    // calculate vector from waypoint a to b
    const double aa_bb_n        = in[37] - in[34];
    const double aa_bb_e        = in[38] - in[35];
    const double aa_bb_d        = in[39] - in[36];

    const double normaa_bb      = sqrt( aa_bb_n*aa_bb_n + aa_bb_e*aa_bb_e + aa_bb_d*aa_bb_d );
    const double aa_bb_unit_n   = aa_bb_n / normaa_bb;
    const double aa_bb_unit_e   = aa_bb_e / normaa_bb;
    const double aa_bb_unit_d   = aa_bb_d / normaa_bb;

    // calculate closest point on line a->b
    const double aa_pp_n        = in[9] - in[34];
    const double aa_pp_e        = in[10] - in[35];
    const double aa_pp_d        = in[11] - in[36];

    const double pp_proj        = aa_bb_unit_n*aa_pp_n + aa_bb_unit_e*aa_pp_e + aa_bb_unit_d*aa_pp_d;

    const double dd_n           = in[34] + pp_proj * aa_bb_unit_n;
    const double dd_e           = in[35] + pp_proj * aa_bb_unit_e;
    const double dd_d           = in[36] + pp_proj * aa_bb_unit_d;

    // longitudinal / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

    // calculate longitudinal track error
    const double aa_bb_lon_unit_ne   = sqrt( aa_bb_unit_n*aa_bb_unit_n + aa_bb_unit_e*aa_bb_unit_e );
    const double aa_bb_lon_unit_d    = aa_bb_unit_d;

    const double pp_dd_d             = dd_d - in[11];
    const double xtrackerr_lon       = pp_dd_d / aa_bb_lon_unit_ne;

    // calculate the L1 length required for the desired period
    L1R_lon                         = in[25] * in[26] / 3.14159265359;
    double L1_lon                   = L1R_lon * V_lon;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lon          = fabs(xtrackerr_lon);
    if (L1_lon < L1min_lon) { 
        L1_lon = L1min_lon;
    }

    // calculate L1 vector
    const double normdd_rr_lon      = sqrt(L1_lon*L1_lon - xtrackerr_lon*xtrackerr_lon);
    const double rr_lon_ne          = aa_bb_lon_unit_ne * normdd_rr_lon;
    const double rr_lon_d           = aa_bb_lon_unit_d * normdd_rr_lon + pp_dd_d;
    LL_lon_ne                       = rr_lon_ne - pp_dd_d * aa_bb_lon_unit_d / aa_bb_lon_unit_ne;
    LL_lon_d                        = rr_lon_d;

    // lateral / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

    const double normaa_bb_lat       = sqrt( aa_bb_n*aa_bb_n + aa_bb_e*aa_bb_e );
    const double aa_bb_lat_unit_n    = aa_bb_n / normaa_bb_lat;
    const double aa_bb_lat_unit_e    = aa_bb_e / normaa_bb_lat;

    // calculate closest point on line a->b
    const double pp_proj_lat        = aa_bb_lat_unit_n*aa_pp_n + aa_bb_lat_unit_e*aa_pp_e;

    const double dd_lat_n           = in[34] + pp_proj_lat * aa_bb_lat_unit_n;
    const double dd_lat_e           = in[35] + pp_proj_lat * aa_bb_lat_unit_e;

    // calculate lateral track error
    const double normdd_pp_lat      = sqrt( (in[9] - dd_lat_n)*(in[9] - dd_lat_n) + (in[10] - dd_lat_e)*(in[10] - dd_lat_e) );

    // calculate the L1 length required for the desired period
    L1R_lat                         = in[27] * in[28] / 3.14159265359;
    double L1_lat                   = L1R_lat * V_lat;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lat          = normdd_pp_lat;
    if (L1_lat < L1min_lat) {
        L1_lat = L1min_lat;
    }

    // calculate L1 vector
    const double normdd_rr_lat      = sqrt(L1_lat*L1_lat - normdd_pp_lat*normdd_pp_lat);
    const double rr_lat_n           = aa_bb_lat_unit_n * normdd_rr_lat + dd_lat_n;
    const double rr_lat_e           = aa_bb_lat_unit_e * normdd_rr_lat + dd_lat_e;
    LL_lat_n                        = rr_lat_n - in[9];
    LL_lat_e                        = rr_lat_e - in[10];

} else if ( fabs(in[33]-1.0) < 0.5 ) { // spiral

	// lateral / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    
    // calculate the distance from the aircraft to the circle
    double cc_pp_n     = in[9] - in[34];
    double cc_pp_e     = in[10] - in[35];
    
    if ( fabs(cc_pp_n) < 0.1 && fabs(cc_pp_e) < 0.1 ) { // cannot be in center of circle
        cc_pp_n = 0.1;
        cc_pp_e = 0.1;
    }

    const double normcc_pp2  = cc_pp_n*cc_pp_n + cc_pp_e*cc_pp_e;
    const double normcc_pp   = sqrt(normcc_pp2);

    const double pp_dd       = normcc_pp - in[37];

    // check circle tracking feasibility, calculate L1 ratio
    const double R08_V              = in[37] / V_lat * 0.5;
    L1R_lat                         = in[28] * in[27] / 3.14159265359;
    L1R_lat                         = (L1R_lat < R08_V) ? R08_V : L1R_lat;

    // calculate the L1 length required for the desired period
    double L1_lat                   = L1R_lat * V_lat;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lat          = fabs(pp_dd);
    const double L1max_lat          = 2.0*in[37] + pp_dd;
    if (L1_lat > L1max_lat) {
        L1_lat  = L1max_lat;
    } else if (L1_lat < L1min_lat) {
        L1_lat  = L1min_lat;
    }

    // calculate lateral components of L1 vector
    double cosgamL1                 = (L1_lat*L1_lat + normcc_pp2 - in[37]*in[37]) / 2.0 / L1_lat / normcc_pp;
    if (cosgamL1 > 1.0) {
        cosgamL1 = 1.0;
    }
    if (cosgamL1 < -1.0) {
        cosgamL1 = -1.0;
    }
    const double gamL1              = acos(cosgamL1);
    const double b_pp               = atan2(-cc_pp_e,-cc_pp_n);
    double b_L1                     = b_pp - in[38] * gamL1;
    if (b_L1 > 3.14159265359) {
        b_L1 = b_L1 - 6.28318530718;
    }
    if (b_L1 < -3.14159265359) {
        b_L1 = b_L1 + 6.28318530718;
    }

    LL_lat_n                        = L1_lat * cos(b_L1);
    LL_lat_e                        = L1_lat * sin(b_L1);

    // longitudinal / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

    // spiral angular position: [0,2*pi)
    const double xi                 = atan2(cc_pp_e, cc_pp_n);
    double delta_xi = xi-in[40];
    if (in[38]>0.0 && in[40]>xi) {
        delta_xi                    = delta_xi + 6.28318530718;
    } else if (in[38]<0.0 && xi>in[40]) {
        delta_xi                    = delta_xi - 6.28318530718;
    }

    // closest point on nearest spiral leg
    double dd_d_sp;
    if (fabs(in[39]) < 0.001) {
        
        dd_d_sp                         = in[36];
    
    } else {
        
        const double Rtangam            = in[37] * tan(in[39]);
        
        // spiral height delta for current angle
        const double delta_d_xi         = -delta_xi * in[38] * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end     = -in[41] * Rtangam;

        // nearest spiral leg
        double delta_d_k            = round( (in[11] - (in[36] + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
        const double delta_d_end_k  = round( (delta_d_sp_end - (in[36] + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
        
        // check
        if (delta_d_k * in[39] > 0.0) { //NOTE: gam is actually being used for its sign, but writing a sign operator doesnt make a difference here
            delta_d_k = 0.0;
        } else if (fabs(delta_d_k)-fabs(delta_d_end_k) > 0 ) {
            delta_d_k = (delta_d_k < 0.0) ? -fabs(delta_d_end_k) : fabs(delta_d_end_k);
        }

        // closest point on nearest spiral leg
        const double delta_d_sp         = delta_d_k + delta_d_xi;
        dd_d_sp                         = in[36] + delta_d_sp;
    }

    // calculate longitudinal track error
    const double aa_bb_lon_unit_ne      = cos(in[39]);
    const double aa_bb_lon_unit_d       = -sin(in[39]);
    const double pp_dd_d                = dd_d_sp - in[11];
    const double xtrackerr_lon          = pp_dd_d / aa_bb_lon_unit_ne;

    // calculate the L1 length required for the desired period
    L1R_lon                 = in[25] * in[26] / 3.14159265359;
    double L1_lon           = L1R_lon * V_lon;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lon  = fabs(xtrackerr_lon);
    if (L1_lon < L1min_lon) {
        L1_lon = L1min_lon;
    }

    // calculate L1 vector
    const double normdd_rr_lon  = sqrt(L1_lon*L1_lon - xtrackerr_lon*xtrackerr_lon);
    const double rr_lon_ne      = aa_bb_lon_unit_ne * normdd_rr_lon;
    const double rr_lon_d       = aa_bb_lon_unit_d * normdd_rr_lon + pp_dd_d;
    LL_lon_ne                   = rr_lon_ne - pp_dd_d * aa_bb_lon_unit_d / aa_bb_lon_unit_ne;
    LL_lon_d                    = rr_lon_d;
    
}

const double atan2_01 = atan2(t10, V_lat);
const double atan2_02 = atan2(LL_lon_d, LL_lon_ne);
double etalon = atan2_01-atan2_02;
if (etalon>3.14159265359) etalon = etalon - 6.28318530718;
if (etalon<-3.14159265359) etalon = etalon + 6.28318530718;

const double atan2_03 = atan2(LL_lat_e, LL_lat_n);
const double atan2_04 = atan2(t9, t7);
double etalat = atan2_03-atan2_04;
if (etalat>3.14159265359) etalat = etalat - 6.28318530718;
if (etalat<-3.14159265359) etalat = etalat + 6.28318530718;

/* rhs */

out[0] = in[20]*4.713471698113208-t6*(9.81E2/1.0E2)+in[1]*in[5]-in[2]*in[4]-t48*(t41*t55-in[2]*t34*t41*t52)*(2.0E1/5.3E1);
out[1] = -in[0]*in[5]+in[2]*in[3]+t2*t8*(9.81E2/1.0E2)-t48*t59*1.159735849056604E-1;
out[2] = in[0]*in[4]-in[1]*in[3]+t4*t8*(9.81E2/1.0E2)-t48*(t41*t52+in[2]*t34*t41*t55)*(2.0E1/5.3E1);
out[3] = t71*t77*(-9.263652961916226E-1)+in[4]*(in[3]*2.270436E-2-in[5]*7.616616999999998E-2)*1.22697390224056E1-t71*(t73-in[2]*t34*t41*t63)*6.433024169447254;
out[4] = in[3]*in[5]*9.181328545780969E-1-(in[3]*in[3])*1.93639394716594E-1+(in[5]*in[5])*1.93639394716594E-1-(t38*4.9914E-2+t43*4.9914E-2+t44*4.9914E-2)*(in[21]*3.5262+t36*2.0621+in[4]*t57*4.497507-9.8911E-2)*2.564760194921775;
out[5] = t71*t77*(-2.040702994206499)-t71*(t73-t75)*9.263652961916226E-1-in[4]*(in[3]*3.148557560000001E-2+in[5]*2.270436E-2)*1.22697390224056E1;
out[6] = in[3]+t80*tan(in[7]);
out[7] = -in[5]*t2+in[4]*t4;
out[8] = t80/t8;
out[9] = t7;
out[10] = t9;
out[11] = t10;
out[12] = etalon*in[29];
out[13] = etalat*in[30];
out[14] = in[31]*(in[32]-sqrt(t56));
out[15] = in[15]*in[42]+in[20]*in[43];
out[16] = in[16]*in[42]+in[21]*in[43];
out[17] = in[17]*in[42]+in[22]*in[43];
out[18] = in[18]*in[42]+in[23]*in[43];
out[19] = in[19]*in[42]+in[24]*in[43];

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

const double t2 = sin(in[6]);
const double t3 = sin(in[8]);
const double t4 = cos(in[6]);
const double t5 = cos(in[8]);
const double t6 = sin(in[7]);
const double t8 = cos(in[7]);
const double t11 = t3*t4;
const double t12 = t2*t5*t6;
const double t13 = t11-t12;
const double t14 = in[1]*t13;
const double t15 = t2*t3;
const double t16 = t4*t5*t6;
const double t17 = t15+t16;
const double t18 = in[2]*t17;
const double t19 = in[0]*t5*t8;
const double t7 = in[46]-t14+t18+t19;
const double t21 = t4*t5;
const double t22 = t2*t3*t6;
const double t23 = t21+t22;
const double t24 = in[1]*t23;
const double t25 = t2*t5;
const double t26 = t3*t4*t6;
const double t27 = t25-t26;
const double t28 = in[2]*t27;
const double t29 = in[0]*t3*t8;
const double t9 = in[47]+t24-t28+t29;
const double t31 = in[0]*t6;
const double t32 = in[2]*t4*t8;
const double t33 = in[1]*t2*t8;
const double t10 = in[48]-t31+t32+t33;
const double t20 = t7*t7;
const double t30 = t9*t9;
const double t34 = in[0]*in[0];
const double t35 = in[1]*in[1];
const double t36 = in[2]*in[2];
const double t37 = t34+t35+t36;

/* augmented guidance logic */

const double V_lat = sqrt(t20+t30);
const double V_lon = sqrt(t20+t30+t10*t10);

double LL_lon_ne        = 1.0;
double LL_lon_d         = 0.0;
double LL_lat_n         = 1.0;
double LL_lat_e         = 0.0;
double L1R_lon          = 1.0;
double L1R_lat          = 1.0;

if (fabs(in[33]) < 0.5) { // line

    // calculate vector from waypoint a to b
    const double aa_bb_n        = in[37] - in[34];
    const double aa_bb_e        = in[38] - in[35];
    const double aa_bb_d        = in[39] - in[36];

    const double normaa_bb      = sqrt( aa_bb_n*aa_bb_n + aa_bb_e*aa_bb_e + aa_bb_d*aa_bb_d );
    const double aa_bb_unit_n   = aa_bb_n / normaa_bb;
    const double aa_bb_unit_e   = aa_bb_e / normaa_bb;
    const double aa_bb_unit_d   = aa_bb_d / normaa_bb;

    // calculate closest point on line a->b
    const double aa_pp_n        = in[9] - in[34];
    const double aa_pp_e        = in[10] - in[35];
    const double aa_pp_d        = in[11] - in[36];

    const double pp_proj        = aa_bb_unit_n*aa_pp_n + aa_bb_unit_e*aa_pp_e + aa_bb_unit_d*aa_pp_d;

    const double dd_n           = in[34] + pp_proj * aa_bb_unit_n;
    const double dd_e           = in[35] + pp_proj * aa_bb_unit_e;
    const double dd_d           = in[36] + pp_proj * aa_bb_unit_d;

    // longitudinal / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

    // calculate longitudinal track error
    const double aa_bb_lon_unit_ne   = sqrt( aa_bb_unit_n*aa_bb_unit_n + aa_bb_unit_e*aa_bb_unit_e );
    const double aa_bb_lon_unit_d    = aa_bb_unit_d;

    const double pp_dd_d             = dd_d - in[11];
    const double xtrackerr_lon       = pp_dd_d / aa_bb_lon_unit_ne;

    // calculate the L1 length required for the desired period
    L1R_lon                         = in[25] * in[26] / 3.14159265359;
    double L1_lon                   = L1R_lon * V_lon;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lon          = fabs(xtrackerr_lon);
    if (L1_lon < L1min_lon) { 
        L1_lon = L1min_lon;
    }

    // calculate L1 vector
    const double normdd_rr_lon      = sqrt(L1_lon*L1_lon - xtrackerr_lon*xtrackerr_lon);
    const double rr_lon_ne          = aa_bb_lon_unit_ne * normdd_rr_lon;
    const double rr_lon_d           = aa_bb_lon_unit_d * normdd_rr_lon + pp_dd_d;
    LL_lon_ne                       = rr_lon_ne - pp_dd_d * aa_bb_lon_unit_d / aa_bb_lon_unit_ne;
    LL_lon_d                        = rr_lon_d;

    // lateral / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

    const double normaa_bb_lat       = sqrt( aa_bb_n*aa_bb_n + aa_bb_e*aa_bb_e );
    const double aa_bb_lat_unit_n    = aa_bb_n / normaa_bb_lat;
    const double aa_bb_lat_unit_e    = aa_bb_e / normaa_bb_lat;

    // calculate closest point on line a->b
    const double pp_proj_lat        = aa_bb_lat_unit_n*aa_pp_n + aa_bb_lat_unit_e*aa_pp_e;

    const double dd_lat_n           = in[34] + pp_proj_lat * aa_bb_lat_unit_n;
    const double dd_lat_e           = in[35] + pp_proj_lat * aa_bb_lat_unit_e;

    // calculate lateral track error
    const double normdd_pp_lat      = sqrt( (in[9] - dd_lat_n)*(in[9] - dd_lat_n) + (in[10] - dd_lat_e)*(in[10] - dd_lat_e) );

    // calculate the L1 length required for the desired period
    L1R_lat                         = in[27] * in[28] / 3.14159265359;
    double L1_lat                   = L1R_lat * V_lat;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lat          = normdd_pp_lat;
    if (L1_lat < L1min_lat) {
        L1_lat = L1min_lat;
    }

    // calculate L1 vector
    const double normdd_rr_lat      = sqrt(L1_lat*L1_lat - normdd_pp_lat*normdd_pp_lat);
    const double rr_lat_n           = aa_bb_lat_unit_n * normdd_rr_lat + dd_lat_n;
    const double rr_lat_e           = aa_bb_lat_unit_e * normdd_rr_lat + dd_lat_e;
    LL_lat_n                        = rr_lat_n - in[9];
    LL_lat_e                        = rr_lat_e - in[10];

} else if ( fabs(in[33]-1.0) < 0.5 ) { // spiral

	// lateral / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    
    // calculate the distance from the aircraft to the circle
    double cc_pp_n     = in[9] - in[34];
    double cc_pp_e     = in[10] - in[35];
    
    if ( fabs(cc_pp_n) < 0.1 && fabs(cc_pp_e) < 0.1 ) { // cannot be in center of circle
        cc_pp_n = 0.1;
        cc_pp_e = 0.1;
    }

    const double normcc_pp2  = cc_pp_n*cc_pp_n + cc_pp_e*cc_pp_e;
    const double normcc_pp   = sqrt(normcc_pp2);

    const double pp_dd       = normcc_pp - in[37];

    // check circle tracking feasibility, calculate L1 ratio
    const double R08_V              = in[37] / V_lat * 0.5;
    L1R_lat                         = in[28] * in[27] / 3.14159265359;
    L1R_lat                         = (L1R_lat < R08_V) ? R08_V : L1R_lat;

    // calculate the L1 length required for the desired period
    double L1_lat                   = L1R_lat * V_lat;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lat          = fabs(pp_dd);
    const double L1max_lat          = 2.0*in[37] + pp_dd;
    if (L1_lat > L1max_lat) {
        L1_lat  = L1max_lat;
    } else if (L1_lat < L1min_lat) {
        L1_lat  = L1min_lat;
    }

    // calculate lateral components of L1 vector
    double cosgamL1                 = (L1_lat*L1_lat + normcc_pp2 - in[37]*in[37]) / 2.0 / L1_lat / normcc_pp;
    if (cosgamL1 > 1.0) {
        cosgamL1 = 1.0;
    }
    if (cosgamL1 < -1.0) {
        cosgamL1 = -1.0;
    }
    const double gamL1              = acos(cosgamL1);
    const double b_pp               = atan2(-cc_pp_e,-cc_pp_n);
    double b_L1                     = b_pp - in[38] * gamL1;
    if (b_L1 > 3.14159265359) {
        b_L1 = b_L1 - 6.28318530718;
    }
    if (b_L1 < -3.14159265359) {
        b_L1 = b_L1 + 6.28318530718;
    }

    LL_lat_n                        = L1_lat * cos(b_L1);
    LL_lat_e                        = L1_lat * sin(b_L1);

    // longitudinal / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

    // spiral angular position: [0,2*pi)
    const double xi                 = atan2(cc_pp_e, cc_pp_n);
    double delta_xi = xi-in[40];
    if (in[38]>0.0 && in[40]>xi) {
        delta_xi                    = delta_xi + 6.28318530718;
    } else if (in[38]<0.0 && xi>in[40]) {
        delta_xi                    = delta_xi - 6.28318530718;
    }

    // closest point on nearest spiral leg
    double dd_d_sp;
    if (fabs(in[39]) < 0.001) {
        
        dd_d_sp                         = in[36];
    
    } else {
        
        const double Rtangam            = in[37] * tan(in[39]);
        
        // spiral height delta for current angle
        const double delta_d_xi         = -delta_xi * in[38] * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end     = -in[41] * Rtangam;

        // nearest spiral leg
        double delta_d_k            = round( (in[11] - (in[36] + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
        const double delta_d_end_k  = round( (delta_d_sp_end - (in[36] + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
        
        // check
        if (delta_d_k * in[39] > 0.0) { //NOTE: gam is actually being used for its sign, but writing a sign operator doesnt make a difference here
            delta_d_k = 0.0;
        } else if (fabs(delta_d_k)-fabs(delta_d_end_k) > 0 ) {
            delta_d_k = (delta_d_k < 0.0) ? -fabs(delta_d_end_k) : fabs(delta_d_end_k);
        }

        // closest point on nearest spiral leg
        const double delta_d_sp         = delta_d_k + delta_d_xi;
        dd_d_sp                         = in[36] + delta_d_sp;
    }

    // calculate longitudinal track error
    const double aa_bb_lon_unit_ne      = cos(in[39]);
    const double aa_bb_lon_unit_d       = -sin(in[39]);
    const double pp_dd_d                = dd_d_sp - in[11];
    const double xtrackerr_lon          = pp_dd_d / aa_bb_lon_unit_ne;

    // calculate the L1 length required for the desired period
    L1R_lon                 = in[25] * in[26] / 3.14159265359;
    double L1_lon           = L1R_lon * V_lon;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lon  = fabs(xtrackerr_lon);
    if (L1_lon < L1min_lon) {
        L1_lon = L1min_lon;
    }

    // calculate L1 vector
    const double normdd_rr_lon  = sqrt(L1_lon*L1_lon - xtrackerr_lon*xtrackerr_lon);
    const double rr_lon_ne      = aa_bb_lon_unit_ne * normdd_rr_lon;
    const double rr_lon_d       = aa_bb_lon_unit_d * normdd_rr_lon + pp_dd_d;
    LL_lon_ne                   = rr_lon_ne - pp_dd_d * aa_bb_lon_unit_d / aa_bb_lon_unit_ne;
    LL_lon_d                    = rr_lon_d;
    
}

const double atan2_01 = atan2(t10, V_lat);
const double atan2_02 = atan2(LL_lon_d, LL_lon_ne);
double etalon = atan2_01-atan2_02;
if (etalon>3.14159265359) etalon = etalon - 6.28318530718;
if (etalon<-3.14159265359) etalon = etalon + 6.28318530718;

const double atan2_03 = atan2(LL_lat_e, LL_lat_n);
const double atan2_04 = atan2(t9, t7);
double etalat = atan2_03-atan2_04;
if (etalat>3.14159265359) etalat = etalat - 6.28318530718;
if (etalat<-3.14159265359) etalat = etalat + 6.28318530718;

/* outputs */

out[0] = in[14]+in[32]-sqrt(t37);
out[1] = -in[7]+atan((V_lon*(in[12]+etalon*(in[26]*in[26])*4.0)*(1.0E2/9.81E2))/L1R_lon);
out[2] = -in[6]+atan((V_lat*(in[13]+etalat*(in[28]*in[28])*4.0)*(1.0E2/9.81E2))/L1R_lat);
out[3] = asin(in[1]*1.0/sqrt(t37));
out[4] = in[3];
out[5] = in[4];
out[6] = in[5];
out[7] = in[15]*in[44]+in[20]*in[45];
out[8] = in[16]*in[44]+in[21]*in[45];
out[9] = in[17]*in[44]+in[22]*in[45];
out[10] = in[18]*in[44]+in[23]*in[45];
out[11] = in[19]*in[44]+in[24]*in[45];

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

const double t2 = sin(in[6]);
const double t3 = sin(in[8]);
const double t4 = cos(in[6]);
const double t5 = cos(in[8]);
const double t6 = sin(in[7]);
const double t8 = cos(in[7]);
const double t11 = t3*t4;
const double t12 = t2*t5*t6;
const double t13 = t11-t12;
const double t14 = in[1]*t13;
const double t15 = t2*t3;
const double t16 = t4*t5*t6;
const double t17 = t15+t16;
const double t18 = in[2]*t17;
const double t19 = in[0]*t5*t8;
const double t7 = in[46]-t14+t18+t19;
const double t21 = t4*t5;
const double t22 = t2*t3*t6;
const double t23 = t21+t22;
const double t24 = in[1]*t23;
const double t25 = t2*t5;
const double t26 = t3*t4*t6;
const double t27 = t25-t26;
const double t28 = in[2]*t27;
const double t29 = in[0]*t3*t8;
const double t9 = in[47]+t24-t28+t29;
const double t31 = in[0]*t6;
const double t32 = in[2]*t4*t8;
const double t33 = in[1]*t2*t8;
const double t10 = in[48]-t31+t32+t33;
const double t20 = t7*t7;
const double t30 = t9*t9;
const double t34 = in[0]*in[0];
const double t35 = in[1]*in[1];
const double t36 = in[2]*in[2];
const double t37 = t34+t35+t36;

/* augmented guidance logic */

const double V_lat = sqrt(t20+t30);
const double V_lon = sqrt(t20+t30+t10*t10);

double LL_lon_ne        = 1.0;
double LL_lon_d         = 0.0;
double LL_lat_n         = 1.0;
double LL_lat_e         = 0.0;
double L1R_lon          = 1.0;
double L1R_lat          = 1.0;

if (fabs(in[33]) < 0.5) { // line

    // calculate vector from waypoint a to b
    const double aa_bb_n        = in[37] - in[34];
    const double aa_bb_e        = in[38] - in[35];
    const double aa_bb_d        = in[39] - in[36];

    const double normaa_bb      = sqrt( aa_bb_n*aa_bb_n + aa_bb_e*aa_bb_e + aa_bb_d*aa_bb_d );
    const double aa_bb_unit_n   = aa_bb_n / normaa_bb;
    const double aa_bb_unit_e   = aa_bb_e / normaa_bb;
    const double aa_bb_unit_d   = aa_bb_d / normaa_bb;

    // calculate closest point on line a->b
    const double aa_pp_n        = in[9] - in[34];
    const double aa_pp_e        = in[10] - in[35];
    const double aa_pp_d        = in[11] - in[36];

    const double pp_proj        = aa_bb_unit_n*aa_pp_n + aa_bb_unit_e*aa_pp_e + aa_bb_unit_d*aa_pp_d;

    const double dd_n           = in[34] + pp_proj * aa_bb_unit_n;
    const double dd_e           = in[35] + pp_proj * aa_bb_unit_e;
    const double dd_d           = in[36] + pp_proj * aa_bb_unit_d;

    // longitudinal / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

    // calculate longitudinal track error
    const double aa_bb_lon_unit_ne   = sqrt( aa_bb_unit_n*aa_bb_unit_n + aa_bb_unit_e*aa_bb_unit_e );
    const double aa_bb_lon_unit_d    = aa_bb_unit_d;

    const double pp_dd_d             = dd_d - in[11];
    const double xtrackerr_lon       = pp_dd_d / aa_bb_lon_unit_ne;

    // calculate the L1 length required for the desired period
    L1R_lon                         = in[25] * in[26] / 3.14159265359;
    double L1_lon                   = L1R_lon * V_lon;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lon          = fabs(xtrackerr_lon);
    if (L1_lon < L1min_lon) { 
        L1_lon = L1min_lon;
    }

    // calculate L1 vector
    const double normdd_rr_lon      = sqrt(L1_lon*L1_lon - xtrackerr_lon*xtrackerr_lon);
    const double rr_lon_ne          = aa_bb_lon_unit_ne * normdd_rr_lon;
    const double rr_lon_d           = aa_bb_lon_unit_d * normdd_rr_lon + pp_dd_d;
    LL_lon_ne                       = rr_lon_ne - pp_dd_d * aa_bb_lon_unit_d / aa_bb_lon_unit_ne;
    LL_lon_d                        = rr_lon_d;

    // lateral / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

    const double normaa_bb_lat       = sqrt( aa_bb_n*aa_bb_n + aa_bb_e*aa_bb_e );
    const double aa_bb_lat_unit_n    = aa_bb_n / normaa_bb_lat;
    const double aa_bb_lat_unit_e    = aa_bb_e / normaa_bb_lat;

    // calculate closest point on line a->b
    const double pp_proj_lat        = aa_bb_lat_unit_n*aa_pp_n + aa_bb_lat_unit_e*aa_pp_e;

    const double dd_lat_n           = in[34] + pp_proj_lat * aa_bb_lat_unit_n;
    const double dd_lat_e           = in[35] + pp_proj_lat * aa_bb_lat_unit_e;

    // calculate lateral track error
    const double normdd_pp_lat      = sqrt( (in[9] - dd_lat_n)*(in[9] - dd_lat_n) + (in[10] - dd_lat_e)*(in[10] - dd_lat_e) );

    // calculate the L1 length required for the desired period
    L1R_lat                         = in[27] * in[28] / 3.14159265359;
    double L1_lat                   = L1R_lat * V_lat;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lat          = normdd_pp_lat;
    if (L1_lat < L1min_lat) {
        L1_lat = L1min_lat;
    }

    // calculate L1 vector
    const double normdd_rr_lat      = sqrt(L1_lat*L1_lat - normdd_pp_lat*normdd_pp_lat);
    const double rr_lat_n           = aa_bb_lat_unit_n * normdd_rr_lat + dd_lat_n;
    const double rr_lat_e           = aa_bb_lat_unit_e * normdd_rr_lat + dd_lat_e;
    LL_lat_n                        = rr_lat_n - in[9];
    LL_lat_e                        = rr_lat_e - in[10];

} else if ( fabs(in[33]-1.0) < 0.5 ) { // spiral

	// lateral / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    
    // calculate the distance from the aircraft to the circle
    double cc_pp_n     = in[9] - in[34];
    double cc_pp_e     = in[10] - in[35];
    
    if ( fabs(cc_pp_n) < 0.1 && fabs(cc_pp_e) < 0.1 ) { // cannot be in center of circle
        cc_pp_n = 0.1;
        cc_pp_e = 0.1;
    }

    const double normcc_pp2  = cc_pp_n*cc_pp_n + cc_pp_e*cc_pp_e;
    const double normcc_pp   = sqrt(normcc_pp2);

    const double pp_dd       = normcc_pp - in[37];

    // check circle tracking feasibility, calculate L1 ratio
    const double R08_V              = in[37] / V_lat * 0.5;
    L1R_lat                         = in[28] * in[27] / 3.14159265359;
    L1R_lat                         = (L1R_lat < R08_V) ? R08_V : L1R_lat;

    // calculate the L1 length required for the desired period
    double L1_lat                   = L1R_lat * V_lat;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lat          = fabs(pp_dd);
    const double L1max_lat          = 2.0*in[37] + pp_dd;
    if (L1_lat > L1max_lat) {
        L1_lat  = L1max_lat;
    } else if (L1_lat < L1min_lat) {
        L1_lat  = L1min_lat;
    }

    // calculate lateral components of L1 vector
    double cosgamL1                 = (L1_lat*L1_lat + normcc_pp2 - in[37]*in[37]) / 2.0 / L1_lat / normcc_pp;
    if (cosgamL1 > 1.0) {
        cosgamL1 = 1.0;
    }
    if (cosgamL1 < -1.0) {
        cosgamL1 = -1.0;
    }
    const double gamL1              = acos(cosgamL1);
    const double b_pp               = atan2(-cc_pp_e,-cc_pp_n);
    double b_L1                     = b_pp - in[38] * gamL1;
    if (b_L1 > 3.14159265359) {
        b_L1 = b_L1 - 6.28318530718;
    }
    if (b_L1 < -3.14159265359) {
        b_L1 = b_L1 + 6.28318530718;
    }

    LL_lat_n                        = L1_lat * cos(b_L1);
    LL_lat_e                        = L1_lat * sin(b_L1);

    // longitudinal / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

    // spiral angular position: [0,2*pi)
    const double xi                 = atan2(cc_pp_e, cc_pp_n);
    double delta_xi = xi-in[40];
    if (in[38]>0.0 && in[40]>xi) {
        delta_xi                    = delta_xi + 6.28318530718;
    } else if (in[38]<0.0 && xi>in[40]) {
        delta_xi                    = delta_xi - 6.28318530718;
    }

    // closest point on nearest spiral leg
    double dd_d_sp;
    if (fabs(in[39]) < 0.001) {
        
        dd_d_sp                         = in[36];
    
    } else {
        
        const double Rtangam            = in[37] * tan(in[39]);
        
        // spiral height delta for current angle
        const double delta_d_xi         = -delta_xi * in[38] * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end     = -in[41] * Rtangam;

        // nearest spiral leg
        double delta_d_k            = round( (in[11] - (in[36] + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
        const double delta_d_end_k  = round( (delta_d_sp_end - (in[36] + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
        
        // check
        if (delta_d_k * in[39] > 0.0) { //NOTE: gam is actually being used for its sign, but writing a sign operator doesnt make a difference here
            delta_d_k = 0.0;
        } else if (fabs(delta_d_k)-fabs(delta_d_end_k) > 0 ) {
            delta_d_k = (delta_d_k < 0.0) ? -fabs(delta_d_end_k) : fabs(delta_d_end_k);
        }

        // closest point on nearest spiral leg
        const double delta_d_sp         = delta_d_k + delta_d_xi;
        dd_d_sp                         = in[36] + delta_d_sp;
    }

    // calculate longitudinal track error
    const double aa_bb_lon_unit_ne      = cos(in[39]);
    const double aa_bb_lon_unit_d       = -sin(in[39]);
    const double pp_dd_d                = dd_d_sp - in[11];
    const double xtrackerr_lon          = pp_dd_d / aa_bb_lon_unit_ne;

    // calculate the L1 length required for the desired period
    L1R_lon                 = in[25] * in[26] / 3.14159265359;
    double L1_lon           = L1R_lon * V_lon;

    // check that L1 vector does not exceed reasonable bounds
    const double L1min_lon  = fabs(xtrackerr_lon);
    if (L1_lon < L1min_lon) {
        L1_lon = L1min_lon;
    }

    // calculate L1 vector
    const double normdd_rr_lon  = sqrt(L1_lon*L1_lon - xtrackerr_lon*xtrackerr_lon);
    const double rr_lon_ne      = aa_bb_lon_unit_ne * normdd_rr_lon;
    const double rr_lon_d       = aa_bb_lon_unit_d * normdd_rr_lon + pp_dd_d;
    LL_lon_ne                   = rr_lon_ne - pp_dd_d * aa_bb_lon_unit_d / aa_bb_lon_unit_ne;
    LL_lon_d                    = rr_lon_d;
    
}

const double atan2_01 = atan2(t10, V_lat);
const double atan2_02 = atan2(LL_lon_d, LL_lon_ne);
double etalon = atan2_01-atan2_02;
if (etalon>3.14159265359) etalon = etalon - 6.28318530718;
if (etalon<-3.14159265359) etalon = etalon + 6.28318530718;

const double atan2_03 = atan2(LL_lat_e, LL_lat_n);
const double atan2_04 = atan2(t9, t7);
double etalat = atan2_03-atan2_04;
if (etalat>3.14159265359) etalat = etalat - 6.28318530718;
if (etalat<-3.14159265359) etalat = etalat + 6.28318530718;

/* outputs */

out[0] = in[14]+in[32]-sqrt(t37);
out[1] = -in[7]+atan((V_lon*(in[12]+etalon*(in[26]*in[26])*4.0)*(1.0E2/9.81E2))/L1R_lon);
out[2] = -in[6]+atan((V_lat*(in[13]+etalat*(in[28]*in[28])*4.0)*(1.0E2/9.81E2))/L1R_lat);
out[3] = asin(in[1]*1.0/sqrt(t37));
out[4] = in[3];
out[5] = in[4];
out[6] = in[5];

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

