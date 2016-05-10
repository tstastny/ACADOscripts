#include "acado_common.h"
#include <math.h>
#include <string.h>

void rhs( const real_t *in, real_t *out ){

/* optimized intermediate calculations */

const double t2 = in[33]-in[36];
const double t3 = t2*t2;
const double t4 = in[34]-in[37];
const double t5 = in[35]-in[38];
const double t6 = t4*t4;
const double t7 = t5*t5;
const double t8 = t3+t6+t7;
const double t9 = 1.0/t8;
const double t10 = 1.0/sqrt(t8);
const double t11 = cos(in[7]);
const double t13 = cos(in[6]);
const double t14 = sin(in[6]);
const double t15 = sin(in[7]);
const double t50 = in[0]*t15;
const double t51 = in[2]*t11*t13;
const double t52 = in[1]*t11*t14;
const double t12 = in[45]-t50+t51+t52;
const double t16 = sin(in[8]);
const double t17 = cos(in[8]);
const double t30 = t13*t16;
const double t31 = t14*t15*t17;
const double t32 = t30-t31;
const double t33 = in[1]*t32;
const double t34 = t14*t16;
const double t35 = t13*t15*t17;
const double t36 = t34+t35;
const double t37 = in[2]*t36;
const double t38 = in[0]*t11*t17;
const double t18 = in[43]-t33+t37+t38;
const double t40 = t13*t17;
const double t41 = t14*t15*t16;
const double t42 = t40+t41;
const double t43 = in[1]*t42;
const double t44 = t14*t17;
const double t45 = t13*t15*t16;
const double t46 = t44-t45;
const double t47 = in[2]*t46;
const double t48 = in[0]*t11*t16;
const double t19 = in[44]+t43-t47+t48;
const double t20 = t3+t6;
const double t21 = 1.0/sqrt(t20);
const double t22 = in[9]-in[33];
const double t23 = in[10]-in[34];
const double t25 = t2*t21*t22;
const double t26 = t4*t21*t23;
const double t27 = t25+t26;
const double t75 = t2*t21*t27;
const double t24 = -in[9]+in[33]+t75;
const double t70 = t4*t21*t27;
const double t28 = -in[10]+in[34]+t70;
const double t29 = 1.0/3.141592653589793;
const double t39 = t18*t18;
const double t49 = t19*t19;
const double t53 = t39+t49;
const double t54 = sqrt(t53);
const double t55 = t2*t10*t22;
const double t56 = t4*t10*t23;
const double t57 = in[11]-in[35];
const double t58 = t5*t10*t57;
const double t59 = t55+t56+t58;
const double t60 = t5*t10*t59;
const double t61 = t3*t9;
const double t62 = t6*t9;
const double t63 = t61+t62;
const double t68 = 1.0/sqrt(t63);
const double t69 = -in[11]+in[35]+t60;

const double xtrackerr_lon = t68*t69;
const double normdd_pp_lat = sqrt(t24*t24+t28*t28);

double L1_lon = in[25]*in[26]*t29*sqrt(t39+t49+t12*t12);
double L1_lat = in[27]*in[28]*t29*t54;

// check that L1 vector does not exceed reasonable bounds
const double L1min_lon = fabs(xtrackerr_lon);
if (L1_lon < L1min_lon) L1_lon = L1min_lon;
const double L1min_lat = normdd_pp_lat;
if (L1_lat < L1min_lat) L1_lat = L1min_lat;

const double t64 = L1_lon*L1_lon;
const double t65 = xtrackerr_lon*xtrackerr_lon;
const double t66 = t64-t65;
const double t67 = sqrt(t66);
const double t71 = L1_lat*L1_lat;
const double t72 = normdd_pp_lat*normdd_pp_lat;
const double t73 = t71-t72;
const double t74 = sqrt(t73);

const double atan2_01 = atan2(t12, t54);
const double atan2_02 = atan2(-in[11]+in[35]+t60-t5*t10*t67, sqrt(t63)*t67+t5*t10*t68*t69);
double etalon = atan2_01-atan2_02;
if (etalon>3.14159265359) etalon = etalon - 6.28318530718;
if (etalon<-3.14159265359) etalon = etalon + 6.28318530718;

const double atan2_03 = atan2(-in[10]+in[34]+t70-t4*t21*t74, -in[9]+in[33]+t75-t2*t21*t74);
const double atan2_04 = atan2(t19, t18);
double etalat = atan2_03-atan2_04;
if (etalat>3.14159265359) etalat = etalat - 6.28318530718;
if (etalat<-3.14159265359) etalat = etalat + 6.28318530718;

const double t76 = 1.0/in[0];
const double t77 = in[2]*t76;
const double t78 = atan(t77);
const double t79 = 1.0/(in[0]*in[0]);
const double t80 = in[2]*in[2];
const double t81 = t79*t80;
const double t82 = t81+1.0;
const double t83 = 1.0/sqrt(t82);
const double t84 = t78*t78;
const double t85 = in[0]*in[0];
const double t86 = in[1]*in[1];
const double t87 = t85*2.773E-1;
const double t88 = t86*2.773E-1;
const double t89 = t80*2.773E-1;
const double t90 = t87+t88+t89;
const double t91 = t78*1.0852E1;
const double t92 = t84*3.49278E1;
const double t93 = t78*t84*3.45338E1;
const double t94 = t91-t92+t93-1.1754E-1;
const double t95 = t78*4.6194E-1;
const double t96 = t84*1.1993;
const double t97 = t95+t96+6.2874E-2;
const double t98 = t80+t85+t86;
const double t99 = 1.0/sqrt(t98);
const double t100 = in[1]*t99;
const double t101 = asin(t100);
const double t102 = in[23]*6.0945E-2;
const double t103 = in[3]*t99*1.08589635E-1;
const double t104 = in[5]*t99*1.0706801E-1;
const double t116 = t101*4.2987E-2;
const double t105 = t102+t103+t104-t116;
const double t106 = in[22]*5.7723E-2;
const double t107 = t101*1.5419E-2;
const double t108 = in[3]*t99*2.1327355E-1;
const double t114 = in[5]*t99*1.5132075E-2;
const double t109 = t106+t107+t108-t114;
const double t110 = t85*7.18207E-1;
const double t111 = t86*7.18207E-1;
const double t112 = t80*7.18207E-1;
const double t113 = t110+t111+t112;
const double t115 = t83*t109;
const double t117 = in[2]*t76*t83*t105;
const double t118 = in[2]*t76*t83*t109;
const double t119 = t117+t118;
const double t120 = in[5]*t13;
const double t121 = in[4]*t14;
const double t122 = t120+t121;

/* rhs */

out[0] = in[20]*4.713471698113208-t15*(9.81E2/1.0E2)+in[1]*in[5]-in[2]*in[4]-t90*(t83*t97-in[2]*t76*t83*t94)*(2.0E1/5.3E1);
out[1] = -in[0]*in[5]+in[2]*in[3]+t11*t14*(9.81E2/1.0E2)-t90*t101*1.159735849056604E-1;
out[2] = in[0]*in[4]-in[1]*in[3]+t11*t13*(9.81E2/1.0E2)-t90*(t83*t94+in[2]*t76*t83*t97)*(2.0E1/5.3E1);
out[3] = t113*t119*(-9.263652961916226E-1)+in[4]*(in[3]*2.270436E-2-in[5]*7.616616999999998E-2)*1.22697390224056E1-t113*(t115-in[2]*t76*t83*t105)*6.433024169447254;
out[4] = in[3]*in[5]*9.181328545780969E-1-(in[3]*in[3])*1.93639394716594E-1+(in[5]*in[5])*1.93639394716594E-1-(t80*4.9914E-2+t85*4.9914E-2+t86*4.9914E-2)*(in[21]*3.5262+t78*2.0621+in[4]*t99*4.497507-9.8911E-2)*2.564760194921775;
out[5] = t113*t119*(-2.040702994206499)-t113*(t115-t117)*9.263652961916226E-1-in[4]*(in[3]*3.148557560000001E-2+in[5]*2.270436E-2)*1.22697390224056E1;
out[6] = in[3]+t122*tan(in[7]);
out[7] = in[4]*t13-in[5]*t14;
out[8] = t122/t11;
out[9] = t18;
out[10] = t19;
out[11] = t12;
out[12] = etalon*in[29];
out[13] = etalat*in[30];
out[14] = in[31]*(in[32]-sqrt(t98));
out[15] = in[15]*in[39]+in[20]*in[40];
out[16] = in[16]*in[39]+in[21]*in[40];
out[17] = in[17]*in[39]+in[22]*in[40];
out[18] = in[18]*in[39]+in[23]*in[40];
out[19] = in[19]*in[39]+in[24]*in[40];

}

void rhs_eval( real_t *in, real_t *out ){

/* optimized intermediate calculations */

const double t2 = in[33]-in[36];
const double t3 = t2*t2;
const double t4 = in[34]-in[37];
const double t5 = in[35]-in[38];
const double t6 = t4*t4;
const double t7 = t5*t5;
const double t8 = t3+t6+t7;
const double t9 = 1.0/t8;
const double t10 = 1.0/sqrt(t8);
const double t11 = cos(in[7]);
const double t13 = cos(in[6]);
const double t14 = sin(in[6]);
const double t15 = sin(in[7]);
const double t50 = in[0]*t15;
const double t51 = in[2]*t11*t13;
const double t52 = in[1]*t11*t14;
const double t12 = in[45]-t50+t51+t52;
const double t16 = sin(in[8]);
const double t17 = cos(in[8]);
const double t30 = t13*t16;
const double t31 = t14*t15*t17;
const double t32 = t30-t31;
const double t33 = in[1]*t32;
const double t34 = t14*t16;
const double t35 = t13*t15*t17;
const double t36 = t34+t35;
const double t37 = in[2]*t36;
const double t38 = in[0]*t11*t17;
const double t18 = in[43]-t33+t37+t38;
const double t40 = t13*t17;
const double t41 = t14*t15*t16;
const double t42 = t40+t41;
const double t43 = in[1]*t42;
const double t44 = t14*t17;
const double t45 = t13*t15*t16;
const double t46 = t44-t45;
const double t47 = in[2]*t46;
const double t48 = in[0]*t11*t16;
const double t19 = in[44]+t43-t47+t48;
const double t20 = t3+t6;
const double t21 = 1.0/sqrt(t20);
const double t22 = in[9]-in[33];
const double t23 = in[10]-in[34];
const double t25 = t2*t21*t22;
const double t26 = t4*t21*t23;
const double t27 = t25+t26;
const double t75 = t2*t21*t27;
const double t24 = -in[9]+in[33]+t75;
const double t70 = t4*t21*t27;
const double t28 = -in[10]+in[34]+t70;
const double t29 = 1.0/3.141592653589793;
const double t39 = t18*t18;
const double t49 = t19*t19;
const double t53 = t39+t49;
const double t54 = sqrt(t53);
const double t55 = t2*t10*t22;
const double t56 = t4*t10*t23;
const double t57 = in[11]-in[35];
const double t58 = t5*t10*t57;
const double t59 = t55+t56+t58;
const double t60 = t5*t10*t59;
const double t61 = t3*t9;
const double t62 = t6*t9;
const double t63 = t61+t62;
const double t68 = 1.0/sqrt(t63);
const double t69 = -in[11]+in[35]+t60;

const double xtrackerr_lon = t68*t69;
const double normdd_pp_lat = sqrt(t24*t24+t28*t28);

double L1_lon = in[25]*in[26]*t29*sqrt(t39+t49+t12*t12);
double L1_lat = in[27]*in[28]*t29*t54;

// check that L1 vector does not exceed reasonable bounds
const double L1min_lon = fabs(xtrackerr_lon);
if (L1_lon < L1min_lon) L1_lon = L1min_lon;
const double L1min_lat = normdd_pp_lat;
if (L1_lat < L1min_lat) L1_lat = L1min_lat;

const double t64 = L1_lon*L1_lon;
const double t65 = xtrackerr_lon*xtrackerr_lon;
const double t66 = t64-t65;
const double t67 = sqrt(t66);
const double t71 = L1_lat*L1_lat;
const double t72 = normdd_pp_lat*normdd_pp_lat;
const double t73 = t71-t72;
const double t74 = sqrt(t73);

const double atan2_01 = atan2(t12, t54);
const double atan2_02 = atan2(-in[11]+in[35]+t60-t5*t10*t67, sqrt(t63)*t67+t5*t10*t68*t69);
double etalon = atan2_01-atan2_02;
if (etalon>3.14159265359) etalon = etalon - 6.28318530718;
if (etalon<-3.14159265359) etalon = etalon + 6.28318530718;

const double atan2_03 = atan2(-in[10]+in[34]+t70-t4*t21*t74, -in[9]+in[33]+t75-t2*t21*t74);
const double atan2_04 = atan2(t19, t18);
double etalat = atan2_03-atan2_04;
if (etalat>3.14159265359) etalat = etalat - 6.28318530718;
if (etalat<-3.14159265359) etalat = etalat + 6.28318530718;

const double t76 = 1.0/in[0];
const double t77 = in[2]*t76;
const double t78 = atan(t77);
const double t79 = 1.0/(in[0]*in[0]);
const double t80 = in[2]*in[2];
const double t81 = t79*t80;
const double t82 = t81+1.0;
const double t83 = 1.0/sqrt(t82);
const double t84 = t78*t78;
const double t85 = in[0]*in[0];
const double t86 = in[1]*in[1];
const double t87 = t85*2.773E-1;
const double t88 = t86*2.773E-1;
const double t89 = t80*2.773E-1;
const double t90 = t87+t88+t89;
const double t91 = t78*1.0852E1;
const double t92 = t84*3.49278E1;
const double t93 = t78*t84*3.45338E1;
const double t94 = t91-t92+t93-1.1754E-1;
const double t95 = t78*4.6194E-1;
const double t96 = t84*1.1993;
const double t97 = t95+t96+6.2874E-2;
const double t98 = t80+t85+t86;
const double t99 = 1.0/sqrt(t98);
const double t100 = in[1]*t99;
const double t101 = asin(t100);
const double t102 = in[23]*6.0945E-2;
const double t103 = in[3]*t99*1.08589635E-1;
const double t104 = in[5]*t99*1.0706801E-1;
const double t116 = t101*4.2987E-2;
const double t105 = t102+t103+t104-t116;
const double t106 = in[22]*5.7723E-2;
const double t107 = t101*1.5419E-2;
const double t108 = in[3]*t99*2.1327355E-1;
const double t114 = in[5]*t99*1.5132075E-2;
const double t109 = t106+t107+t108-t114;
const double t110 = t85*7.18207E-1;
const double t111 = t86*7.18207E-1;
const double t112 = t80*7.18207E-1;
const double t113 = t110+t111+t112;
const double t115 = t83*t109;
const double t117 = in[2]*t76*t83*t105;
const double t118 = in[2]*t76*t83*t109;
const double t119 = t117+t118;
const double t120 = in[5]*t13;
const double t121 = in[4]*t14;
const double t122 = t120+t121;

/* rhs */

out[0] = in[20]*4.713471698113208-t15*(9.81E2/1.0E2)+in[1]*in[5]-in[2]*in[4]-t90*(t83*t97-in[2]*t76*t83*t94)*(2.0E1/5.3E1);
out[1] = -in[0]*in[5]+in[2]*in[3]+t11*t14*(9.81E2/1.0E2)-t90*t101*1.159735849056604E-1;
out[2] = in[0]*in[4]-in[1]*in[3]+t11*t13*(9.81E2/1.0E2)-t90*(t83*t94+in[2]*t76*t83*t97)*(2.0E1/5.3E1);
out[3] = t113*t119*(-9.263652961916226E-1)+in[4]*(in[3]*2.270436E-2-in[5]*7.616616999999998E-2)*1.22697390224056E1-t113*(t115-in[2]*t76*t83*t105)*6.433024169447254;
out[4] = in[3]*in[5]*9.181328545780969E-1-(in[3]*in[3])*1.93639394716594E-1+(in[5]*in[5])*1.93639394716594E-1-(t80*4.9914E-2+t85*4.9914E-2+t86*4.9914E-2)*(in[21]*3.5262+t78*2.0621+in[4]*t99*4.497507-9.8911E-2)*2.564760194921775;
out[5] = t113*t119*(-2.040702994206499)-t113*(t115-t117)*9.263652961916226E-1-in[4]*(in[3]*3.148557560000001E-2+in[5]*2.270436E-2)*1.22697390224056E1;
out[6] = in[3]+t122*tan(in[7]);
out[7] = in[4]*t13-in[5]*t14;
out[8] = t122/t11;
out[9] = t18;
out[10] = t19;
out[11] = t12;
out[12] = etalon*in[29];
out[13] = etalat*in[30];
out[14] = in[31]*(in[32]-sqrt(t98));
out[15] = in[15]*in[39]+in[20]*in[40];
out[16] = in[16]*in[39]+in[21]*in[40];
out[17] = in[17]*in[39]+in[22]*in[40];
out[18] = in[18]*in[39]+in[23]*in[40];
out[19] = in[19]*in[39]+in[24]*in[40];

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

const double t2 = in[33]-in[36];
const double t3 = t2*t2;
const double t4 = in[34]-in[37];
const double t5 = in[35]-in[38];
const double t6 = t4*t4;
const double t7 = t5*t5;
const double t8 = t3+t6+t7;
const double t9 = 1.0/t8;
const double t10 = 1.0/sqrt(t8);
const double t11 = cos(in[7]);
const double t13 = cos(in[6]);
const double t14 = sin(in[6]);
const double t15 = sin(in[7]);
const double t50 = in[0]*t15;
const double t51 = in[2]*t11*t13;
const double t52 = in[1]*t11*t14;
const double t12 = in[45]-t50+t51+t52;
const double t16 = sin(in[8]);
const double t17 = cos(in[8]);
const double t30 = t13*t16;
const double t31 = t14*t15*t17;
const double t32 = t30-t31;
const double t33 = in[1]*t32;
const double t34 = t14*t16;
const double t35 = t13*t15*t17;
const double t36 = t34+t35;
const double t37 = in[2]*t36;
const double t38 = in[0]*t11*t17;
const double t18 = in[43]-t33+t37+t38;
const double t40 = t13*t17;
const double t41 = t14*t15*t16;
const double t42 = t40+t41;
const double t43 = in[1]*t42;
const double t44 = t14*t17;
const double t45 = t13*t15*t16;
const double t46 = t44-t45;
const double t47 = in[2]*t46;
const double t48 = in[0]*t11*t16;
const double t19 = in[44]+t43-t47+t48;
const double t20 = t3+t6;
const double t21 = 1.0/sqrt(t20);
const double t22 = in[9]-in[33];
const double t23 = in[10]-in[34];
const double t25 = t2*t21*t22;
const double t26 = t4*t21*t23;
const double t27 = t25+t26;
const double t75 = t2*t21*t27;
const double t24 = -in[9]+in[33]+t75;
const double t70 = t4*t21*t27;
const double t28 = -in[10]+in[34]+t70;
const double t29 = 1.0/3.141592653589793;
const double t39 = t18*t18;
const double t49 = t19*t19;
const double t53 = t39+t49;
const double t54 = sqrt(t53);
const double t55 = t2*t10*t22;
const double t56 = t4*t10*t23;
const double t57 = in[11]-in[35];
const double t58 = t5*t10*t57;
const double t59 = t55+t56+t58;
const double t60 = t5*t10*t59;
const double t61 = t3*t9;
const double t62 = t6*t9;
const double t63 = t61+t62;
const double t68 = 1.0/sqrt(t63);
const double t69 = -in[11]+in[35]+t60;
const double t76 = t12*t12;
const double t77 = t39+t49+t76;
const double t78 = sqrt(t77);

const double xtrackerr_lon = t68*t69;
const double normdd_pp_lat = sqrt(t24*t24+t28*t28);

double L1_lon = in[25]*in[26]*t29*t78;
double L1_lat = in[27]*in[28]*t29*t54;

// check that L1 vector does not exceed reasonable bounds
const double L1min_lon = fabs(xtrackerr_lon);
if (L1_lon < L1min_lon) L1_lon = L1min_lon;
const double L1min_lat = normdd_pp_lat;
if (L1_lat < L1min_lat) L1_lat = L1min_lat;

const double t64 = L1_lon*L1_lon;
const double t65 = xtrackerr_lon*xtrackerr_lon;
const double t66 = t64-t65;
const double t67 = sqrt(t66);
const double t71 = L1_lat*L1_lat;
const double t72 = normdd_pp_lat*normdd_pp_lat;
const double t73 = t71-t72;
const double t74 = sqrt(t73);
const double t79 = in[0]*in[0];
const double t80 = in[1]*in[1];
const double t81 = in[2]*in[2];
const double t82 = t79+t80+t81;

const double atan2_01 = atan2(t12, t54);
const double atan2_02 = atan2(-in[11]+in[35]+t60-t5*t10*t67, sqrt(t63)*t67+t5*t10*t68*t69);
double etalon = atan2_01-atan2_02;
if (etalon>3.14159265359) etalon = etalon - 6.28318530718;
if (etalon<-3.14159265359) etalon = etalon + 6.28318530718;

const double atan2_03 = atan2(-in[10]+in[34]+t70-t4*t21*t74, -in[9]+in[33]+t75-t2*t21*t74);
const double atan2_04 = atan2(t19, t18);
double etalat = atan2_03-atan2_04;
if (etalat>3.14159265359) etalat = etalat - 6.28318530718;
if (etalat<-3.14159265359) etalat = etalat + 6.28318530718;

/* outputs */

out[0] = in[14]+in[32]-sqrt(t82);
out[1] = -in[7]+atan((3.141592653589793*t78*(in[12]+etalon*(in[26]*in[26])*4.0)*(1.0E2/9.81E2))/(in[25]*in[26]));
out[2] = -in[6]+atan((3.141592653589793*t54*(in[13]+etalat*(in[28]*in[28])*4.0)*(1.0E2/9.81E2))/(in[27]*in[28]));
out[3] = asin(in[1]*1.0/sqrt(t82));
out[4] = in[3];
out[5] = in[4];
out[6] = in[5];
out[7] = in[15]*in[41]+in[20]*in[42];
out[8] = in[16]*in[41]+in[21]*in[42];
out[9] = in[17]*in[41]+in[22]*in[42];
out[10] = in[18]*in[41]+in[23]*in[42];
out[11] = in[19]*in[41]+in[24]*in[42];

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

const double t2 = in[33]-in[36];
const double t3 = t2*t2;
const double t4 = in[34]-in[37];
const double t5 = in[35]-in[38];
const double t6 = t4*t4;
const double t7 = t5*t5;
const double t8 = t3+t6+t7;
const double t9 = 1.0/t8;
const double t10 = 1.0/sqrt(t8);
const double t11 = cos(in[7]);
const double t13 = cos(in[6]);
const double t14 = sin(in[6]);
const double t15 = sin(in[7]);
const double t50 = in[0]*t15;
const double t51 = in[2]*t11*t13;
const double t52 = in[1]*t11*t14;
const double t12 = in[45]-t50+t51+t52;
const double t16 = sin(in[8]);
const double t17 = cos(in[8]);
const double t30 = t13*t16;
const double t31 = t14*t15*t17;
const double t32 = t30-t31;
const double t33 = in[1]*t32;
const double t34 = t14*t16;
const double t35 = t13*t15*t17;
const double t36 = t34+t35;
const double t37 = in[2]*t36;
const double t38 = in[0]*t11*t17;
const double t18 = in[43]-t33+t37+t38;
const double t40 = t13*t17;
const double t41 = t14*t15*t16;
const double t42 = t40+t41;
const double t43 = in[1]*t42;
const double t44 = t14*t17;
const double t45 = t13*t15*t16;
const double t46 = t44-t45;
const double t47 = in[2]*t46;
const double t48 = in[0]*t11*t16;
const double t19 = in[44]+t43-t47+t48;
const double t20 = t3+t6;
const double t21 = 1.0/sqrt(t20);
const double t22 = in[9]-in[33];
const double t23 = in[10]-in[34];
const double t25 = t2*t21*t22;
const double t26 = t4*t21*t23;
const double t27 = t25+t26;
const double t75 = t2*t21*t27;
const double t24 = -in[9]+in[33]+t75;
const double t70 = t4*t21*t27;
const double t28 = -in[10]+in[34]+t70;
const double t29 = 1.0/3.141592653589793;
const double t39 = t18*t18;
const double t49 = t19*t19;
const double t53 = t39+t49;
const double t54 = sqrt(t53);
const double t55 = t2*t10*t22;
const double t56 = t4*t10*t23;
const double t57 = in[11]-in[35];
const double t58 = t5*t10*t57;
const double t59 = t55+t56+t58;
const double t60 = t5*t10*t59;
const double t61 = t3*t9;
const double t62 = t6*t9;
const double t63 = t61+t62;
const double t68 = 1.0/sqrt(t63);
const double t69 = -in[11]+in[35]+t60;
const double t76 = t12*t12;
const double t77 = t39+t49+t76;
const double t78 = sqrt(t77);

const double xtrackerr_lon = t68*t69;
const double normdd_pp_lat = sqrt(t24*t24+t28*t28);

double L1_lon = in[25]*in[26]*t29*t78;
double L1_lat = in[27]*in[28]*t29*t54;

// check that L1 vector does not exceed reasonable bounds
const double L1min_lon = fabs(xtrackerr_lon);
if (L1_lon < L1min_lon) L1_lon = L1min_lon;
const double L1min_lat = normdd_pp_lat;
if (L1_lat < L1min_lat) L1_lat = L1min_lat;

const double t64 = L1_lon*L1_lon;
const double t65 = xtrackerr_lon*xtrackerr_lon;
const double t66 = t64-t65;
const double t67 = sqrt(t66);
const double t71 = L1_lat*L1_lat;
const double t72 = normdd_pp_lat*normdd_pp_lat;
const double t73 = t71-t72;
const double t74 = sqrt(t73);
const double t79 = in[0]*in[0];
const double t80 = in[1]*in[1];
const double t81 = in[2]*in[2];
const double t82 = t79+t80+t81;

const double atan2_01 = atan2(t12, t54);
const double atan2_02 = atan2(-in[11]+in[35]+t60-t5*t10*t67, sqrt(t63)*t67+t5*t10*t68*t69);
double etalon = atan2_01-atan2_02;
if (etalon>3.14159265359) etalon = etalon - 6.28318530718;
if (etalon<-3.14159265359) etalon = etalon + 6.28318530718;

const double atan2_03 = atan2(-in[10]+in[34]+t70-t4*t21*t74, -in[9]+in[33]+t75-t2*t21*t74);
const double atan2_04 = atan2(t19, t18);
double etalat = atan2_03-atan2_04;
if (etalat>3.14159265359) etalat = etalat - 6.28318530718;
if (etalat<-3.14159265359) etalat = etalat + 6.28318530718;

/* outputs */

out[0] = in[14]+in[32]-sqrt(t82);
out[1] = -in[7]+atan((3.141592653589793*t78*(in[12]+etalon*(in[26]*in[26])*4.0)*(1.0E2/9.81E2))/(in[25]*in[26]));
out[2] = -in[6]+atan((3.141592653589793*t54*(in[13]+etalat*(in[28]*in[28])*4.0)*(1.0E2/9.81E2))/(in[27]*in[28]));
out[3] = asin(in[1]*1.0/sqrt(t82));
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
