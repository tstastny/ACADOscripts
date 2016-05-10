#include <math.h>
#include <string.h>
#include <stdio.h>

#define ACADO_NOD 5
#define ACADO_NU 5
#define ACADO_NX 15
#define ACADO_NY 12
#define ACADO_NYN 7

// ////////////////////////////////////////////////////////////////////////
// analytic model
// ////////////////////////////////////////////////////////////////////////

void rhs( const double *in, double *out ){

/* optimized intermediate calculations */

const double t2 = 1.0/in[0];
const double t3 = in[2]*t2;
const double t4 = atan(t3);
const double t5 = 1.0/(in[0]*in[0]);
const double t6 = in[2]*in[2];
const double t7 = t5*t6;
const double t8 = t7+1.0;
const double t9 = 1.0/sqrt(t8);
const double t10 = t4*t4;
const double t11 = in[0]*in[0];
const double t12 = in[1]*in[1];
const double t13 = t11*2.773E-1;
const double t14 = t12*2.773E-1;
const double t15 = t6*2.773E-1;
const double t16 = t13+t14+t15;
const double t17 = cos(in[7]);
const double t18 = t4*1.0852E1;
const double t19 = t10*3.49278E1;
const double t20 = t4*t10*3.45338E1;
const double t21 = t18-t19+t20-1.1754E-1;
const double t22 = t4*4.6194E-1;
const double t23 = t10*1.1993;
const double t24 = t22+t23+6.2874E-2;
const double t25 = t6+t11+t12;
const double t26 = 1.0/sqrt(t25);
const double t27 = in[1]*t26;
const double t28 = asin(t27);
const double t29 = in[17]*5.7723E-2;
const double t30 = t28*1.5419E-2;
const double t31 = in[3]*t26*2.1327355E-1;
const double t43 = in[5]*t26*1.5132075E-2;
const double t32 = t29+t30+t31-t43;
const double t33 = in[18]*6.0945E-2;
const double t34 = in[3]*t26*1.08589635E-1;
const double t35 = in[5]*t26*1.0706801E-1;
const double t41 = t28*4.2987E-2;
const double t36 = t33+t34+t35-t41;
const double t37 = t11*7.18207E-1;
const double t38 = t12*7.18207E-1;
const double t39 = t6*7.18207E-1;
const double t40 = t37+t38+t39;
const double t42 = t9*t36;
const double t44 = in[2]*t2*t9*t32;
const double t45 = t42+t44;
const double t46 = t9*t32;
const double t47 = t46-in[2]*t2*t9*t36;
const double t48 = cos(in[6]);
const double t49 = sin(in[6]);

/* rhs */

out[0] = in[15]*4.713471698113208-sin(in[7])*(9.81E2/1.0E2)+in[1]*in[5]-in[2]*in[4]-t16*(t9*t24-in[2]*t2*t9*t21)*(2.0E1/5.3E1);
out[1] = -in[0]*in[5]+in[2]*in[3]-t16*t28*1.159735849056604E-1+t17*t49*(9.81E2/1.0E2);
out[2] = in[0]*in[4]-in[1]*in[3]+t17*t48*(9.81E2/1.0E2)-t16*(t9*t21+in[2]*t2*t9*t24)*(2.0E1/5.3E1);
out[3] = t40*t45*(-9.263652961916226E-1)-t40*t47*6.433024169447254+in[4]*(in[3]*2.270436E-2-in[5]*7.616616999999998E-2)*1.22697390224056E1;
out[4] = in[3]*in[5]*9.181328545780969E-1-(in[3]*in[3])*1.93639394716594E-1+(in[5]*in[5])*1.93639394716594E-1-(t6*4.9914E-2+t11*4.9914E-2+t12*4.9914E-2)*(in[16]*3.5262+t4*2.0621+in[4]*t26*4.497507-9.8911E-2)*2.564760194921775;
out[5] = t40*t45*(-2.040702994206499)-t40*t47*9.263652961916226E-1-in[4]*(in[3]*3.148557560000001E-2+in[5]*2.270436E-2)*1.22697390224056E1;
out[6] = in[3]+tan(in[7])*(in[4]*t49+in[5]*t48);
out[7] = in[4]*t48-in[5]*t49;
out[8] = in[19];
out[9] = in[24]*sqrt(t25);
out[10] = in[10]*in[20]+in[15]*in[21];
out[11] = in[11]*in[20]+in[16]*in[21];
out[12] = in[12]*in[20]+in[17]*in[21];
out[13] = in[13]*in[20]+in[18]*in[21];
out[14] = in[14]*in[20]+in[19]*in[21];

}

void rhs_jac( const double *in, double *out ){

/* optimized intermediate calculations */

const double t2 = in[2]*in[2];
const double t3 = 1.0/(in[0]*in[0]);
const double t4 = t2*t3;
const double t5 = t4+1.0;
const double t6 = 1.0/t5;
const double t7 = 1.0/in[0];
const double t8 = 1.0/sqrt(t5);
const double t9 = in[2]*t7;
const double t10 = atan(t9);
const double t11 = t10*t10;
const double t12 = t10*1.0852E1;
const double t13 = t11*3.49278E1;
const double t14 = t10*t11*3.45338E1;
const double t15 = t12-t13+t14-1.1754E-1;
const double t16 = 1.0/pow(t5,3.0/2.0);
const double t17 = t10*4.6194E-1;
const double t18 = t11*1.1993;
const double t19 = t17+t18+6.2874E-2;
const double t20 = t8*t19;
const double t29 = in[2]*t7*t8*t15;
const double t21 = t20-t29;
const double t22 = in[0]*in[0];
const double t23 = t22*2.773E-1;
const double t24 = in[1]*in[1];
const double t25 = t24*2.773E-1;
const double t26 = t2*2.773E-1;
const double t27 = t23+t25+t26;
const double t28 = 1.0/(in[0]*in[0]*in[0]);
const double t30 = t2+t22+t24;
const double t31 = 1.0/sqrt(t30);
const double t32 = in[1]*t31;
const double t33 = asin(t32);
const double t34 = 1.0/t30;
const double t38 = t24*t34;
const double t35 = -t38+1.0;
const double t36 = 1.0/sqrt(t35);
const double t37 = 1.0/pow(t30,3.0/2.0);
const double t39 = cos(in[7]);
const double t40 = in[2]*t3*t6*1.0852E1;
const double t41 = in[2]*t3*t6*t11*1.036014E2;
const double t42 = t40+t41-in[2]*t3*t6*t10*6.98556E1;
const double t43 = in[2]*t3*t6*4.6194E-1;
const double t44 = in[2]*t3*t6*t10*2.3986;
const double t45 = t43+t44;
const double t46 = 1.0/(in[0]*in[0]*in[0]*in[0]);
const double t47 = t8*t15;
const double t48 = in[2]*t7*t8*t19;
const double t49 = t47+t48;
const double t50 = t6*t7*1.0852E1;
const double t51 = t6*t7*t11*1.036014E2;
const double t52 = t50+t51-t6*t7*t10*6.98556E1;
const double t53 = t6*t7*4.6194E-1;
const double t54 = t6*t7*t10*2.3986;
const double t55 = t53+t54;
const double t56 = t2*t16*t19*t28;
const double t57 = sin(in[6]);
const double t58 = cos(in[6]);
const double t59 = sin(in[7]);
const double t60 = in[17]*5.7723E-2;
const double t61 = t33*1.5419E-2;
const double t62 = in[3]*t31*2.1327355E-1;
const double t68 = in[5]*t31*1.5132075E-2;
const double t63 = t60+t61+t62-t68;
const double t64 = in[18]*6.0945E-2;
const double t65 = in[3]*t31*1.08589635E-1;
const double t66 = in[5]*t31*1.0706801E-1;
const double t69 = t33*4.2987E-2;
const double t67 = t64+t65+t66-t69;
const double t70 = t22*7.18207E-1;
const double t71 = t24*7.18207E-1;
const double t72 = t2*7.18207E-1;
const double t73 = t70+t71+t72;
const double t74 = in[0]*in[3]*t37*2.1327355E-1;
const double t75 = in[0]*in[1]*t36*t37*1.5419E-2;
const double t113 = in[0]*in[5]*t37*1.5132075E-2;
const double t76 = t74+t75-t113;
const double t77 = in[0]*in[3]*t37*1.08589635E-1;
const double t78 = in[0]*in[5]*t37*1.0706801E-1;
const double t110 = in[0]*in[1]*t36*t37*4.2987E-2;
const double t79 = t77+t78-t110;
const double t80 = t8*t67;
const double t81 = in[2]*t7*t8*t63;
const double t82 = t80+t81;
const double t83 = t8*t63;
const double t94 = in[2]*t7*t8*t67;
const double t84 = t83-t94;
const double t86 = t24*t37;
const double t85 = t31-t86;
const double t87 = t36*t85*4.2987E-2;
const double t88 = in[1]*in[3]*t37*1.08589635E-1;
const double t89 = in[1]*in[5]*t37*1.0706801E-1;
const double t90 = t87+t88+t89;
const double t91 = t36*t85*1.5419E-2;
const double t92 = in[1]*in[5]*t37*1.5132075E-2;
const double t119 = in[1]*in[3]*t37*2.1327355E-1;
const double t93 = t91+t92-t119;
const double t95 = t2*t16*t28*t63;
const double t96 = in[2]*in[3]*t37*2.1327355E-1;
const double t97 = in[1]*in[2]*t36*t37*1.5419E-2;
const double t128 = in[2]*in[5]*t37*1.5132075E-2;
const double t98 = t96+t97-t128;
const double t99 = in[2]*in[3]*t37*1.08589635E-1;
const double t100 = in[2]*in[5]*t37*1.0706801E-1;
const double t125 = in[1]*in[2]*t36*t37*4.2987E-2;
const double t101 = t99+t100-t125;
const double t102 = in[16]*3.5262;
const double t103 = t10*2.0621;
const double t104 = in[4]*t31*4.497507;
const double t105 = t102+t103+t104-9.8911E-2;
const double t106 = t22*4.9914E-2;
const double t107 = t24*4.9914E-2;
const double t108 = t2*4.9914E-2;
const double t109 = t106+t107+t108;
const double t111 = t8*t79;
const double t112 = in[2]*t3*t8*t63;
const double t114 = in[2]*t7*t8*t76;
const double t133 = t2*t16*t28*t67;
const double t115 = t111+t112+t114-t133-in[2]*t2*t16*t46*t63;
const double t116 = in[2]*t7*t8*t79;
const double t117 = in[2]*t3*t8*t67;
const double t118 = t95+t116+t117-t8*t76-in[2]*t2*t16*t46*t67;
const double t120 = t8*t93;
const double t121 = in[2]*t7*t8*t90;
const double t122 = t120+t121;
const double t123 = t8*t90;
const double t124 = t123-in[2]*t7*t8*t93;
const double t126 = t8*t101;
const double t127 = in[2]*t3*t16*t67;
const double t129 = in[2]*t7*t8*t98;
const double t130 = t95+t126+t127+t129-t7*t8*t63;
const double t131 = t8*t98;
const double t132 = t7*t8*t67;
const double t134 = in[2]*t3*t16*t63;
const double t135 = t8*t31*2.1327355E-1;
const double t136 = t135-in[2]*t7*t8*t31*1.08589635E-1;
const double t137 = t8*t31*1.08589635E-1;
const double t138 = in[2]*t7*t8*t31*2.1327355E-1;
const double t139 = t137+t138;
const double t140 = in[4]*2.785765718707447E-1;
const double t141 = t8*t31*1.0706801E-1;
const double t142 = t141-in[2]*t7*t8*t31*1.5132075E-2;
const double t143 = t8*t31*1.5132075E-2;
const double t144 = in[2]*t7*t8*t31*1.0706801E-1;
const double t145 = t143+t144;
const double t146 = tan(in[7]);
const double t147 = in[5]*t58;
const double t148 = in[4]*t57;

/* rhs_jac */

out[0] = in[0]*t21*(-2.092830188679245E-1)-t27*(t56-t8*t45+in[2]*t3*t8*t15+in[2]*t7*t8*t42-in[2]*t2*t15*t16*t46)*(2.0E1/5.3E1);
out[1] = in[5]-in[1]*t21*2.092830188679245E-1;
out[2] = -in[4]-in[2]*t21*2.092830188679245E-1+t27*(-t8*t55+t7*t8*t15+in[2]*t3*t16*t19+in[2]*t7*t8*t52-t2*t15*t16*t28)*(2.0E1/5.3E1);
out[3] = 0.0;
out[4] = -in[2];
out[5] = in[1];
out[6] = 0.0;
out[7] = t39*(-9.81E2/1.0E2);
out[8] = 0.0;
out[9] = 0.0;
out[10] = 0.0;
out[11] = 0.0;
out[12] = 0.0;
out[13] = 0.0;
out[14] = 0.0;
out[15] = 4.713471698113208;
out[16] = 0.0;
out[17] = 0.0;
out[18] = 0.0;
out[19] = 0.0;
out[20] = -in[5]-in[0]*t33*6.431895018867925E-2+in[0]*in[1]*t27*t36*t37*1.159735849056604E-1;
out[21] = in[1]*t33*(-6.431895018867925E-2)-t27*t36*t85*1.159735849056604E-1;
out[22] = in[3]-in[2]*t33*6.431895018867925E-2+in[1]*in[2]*t27*t36*t37*1.159735849056604E-1;
out[23] = in[2];
out[24] = 0.0;
out[25] = -in[0];
out[26] = t39*t58*(9.81E2/1.0E2);
out[27] = t57*t59*(-9.81E2/1.0E2);
out[28] = 0.0;
out[29] = 0.0;
out[30] = 0.0;
out[31] = 0.0;
out[32] = 0.0;
out[33] = 0.0;
out[34] = 0.0;
out[35] = 0.0;
out[36] = 0.0;
out[37] = 0.0;
out[38] = 0.0;
out[39] = 0.0;
out[40] = in[4]-in[0]*t49*2.092830188679245E-1+t27*(t8*t42+in[2]*t3*t8*t19+in[2]*t7*t8*t45-t2*t15*t16*t28-in[2]*t2*t16*t19*t46)*(2.0E1/5.3E1);
out[41] = -in[3]-in[1]*t49*2.092830188679245E-1;
out[42] = in[2]*t49*(-2.092830188679245E-1)-t27*(-t56+t8*t52+t7*t8*t19-in[2]*t3*t15*t16+in[2]*t7*t8*t55)*(2.0E1/5.3E1);
out[43] = -in[1];
out[44] = in[0];
out[45] = 0.0;
out[46] = t39*t57*(-9.81E2/1.0E2);
out[47] = t58*t59*(-9.81E2/1.0E2);
out[48] = 0.0;
out[49] = 0.0;
out[50] = 0.0;
out[51] = 0.0;
out[52] = 0.0;
out[53] = 0.0;
out[54] = 0.0;
out[55] = 0.0;
out[56] = 0.0;
out[57] = 0.0;
out[58] = 0.0;
out[59] = 0.0;
out[60] = in[0]*t82*(-1.330644080563793)-in[0]*t84*9.240485979332408+t73*t115*9.263652961916226E-1-t73*t118*6.433024169447254;
out[61] = in[1]*t82*(-1.330644080563793)-in[1]*t84*9.240485979332408-t73*t122*6.433024169447254+t73*t124*9.263652961916226E-1;
out[62] = in[2]*t82*(-1.330644080563793)-in[2]*t84*9.240485979332408+t73*t130*9.263652961916226E-1+t73*(t131+t132+t134-in[2]*t7*t8*t101-t2*t16*t28*t67)*6.433024169447254;
out[63] = t140-t73*t136*6.433024169447254-t73*t139*9.263652961916226E-1;
out[64] = in[3]*2.785765718707447E-1-in[5]*9.345390282361782E-1;
out[65] = in[4]*(-9.345390282361782E-1)-t73*t142*9.263652961916226E-1+t73*t145*6.433024169447254;
out[66] = 0.0;
out[67] = 0.0;
out[68] = 0.0;
out[69] = 0.0;
out[70] = 0.0;
out[71] = 0.0;
out[72] = 0.0;
out[73] = 0.0;
out[74] = 0.0;
out[75] = 0.0;
out[76] = 0.0;
out[77] = t8*t73*(-3.713334541330038E-1)-in[2]*t7*t8*t73*5.347258399206903E-2;
out[78] = t8*t73*(-5.645733297639844E-2)+in[2]*t7*t8*t73*3.920606580069629E-1;
out[79] = 0.0;
out[80] = in[0]*t105*(-2.560348807386509E-1)+t109*(in[0]*in[4]*t37*4.497507+in[2]*t3*t6*2.0621)*2.564760194921775;
out[81] = in[1]*t105*(-2.560348807386509E-1)+in[1]*in[4]*t37*t109*1.153502692998205E1;
out[82] = t109*(t6*t7*2.0621-in[2]*in[4]*t37*4.497507)*(-2.564760194921775)-in[2]*t105*2.560348807386509E-1;
out[83] = in[3]*(-3.87278789433188E-1)+in[5]*9.181328545780969E-1;
out[84] = t31*t109*(-1.153502692998205E1);
out[85] = in[3]*9.181328545780969E-1+in[5]*3.87278789433188E-1;
out[86] = 0.0;
out[87] = 0.0;
out[88] = 0.0;
out[89] = 0.0;
out[90] = 0.0;
out[91] = 0.0;
out[92] = 0.0;
out[93] = 0.0;
out[94] = 0.0;
out[95] = 0.0;
out[96] = t2*(-4.514150982303155E-1)-t22*4.514150982303155E-1-t24*4.514150982303155E-1;
out[97] = 0.0;
out[98] = 0.0;
out[99] = 0.0;
out[100] = in[0]*t82*(-2.931294350720134)-in[0]*t84*1.330644080563793+t73*t115*2.040702994206499-t73*t118*9.263652961916226E-1;
out[101] = in[1]*t82*(-2.931294350720134)-in[1]*t84*1.330644080563793-t73*t122*9.263652961916226E-1+t73*t124*2.040702994206499;
out[102] = in[2]*t82*(-2.931294350720134)-in[2]*t84*1.330644080563793+t73*t130*2.040702994206499+t73*(t131+t132-t133+t134-in[2]*t7*t8*t101)*9.263652961916226E-1;
out[103] = in[4]*(-3.863197955822216E-1)-t73*t136*9.263652961916226E-1-t73*t139*2.040702994206499;
out[104] = in[3]*(-3.863197955822216E-1)-in[5]*2.785765718707447E-1;
out[105] = -t140-t73*t142*2.040702994206499+t73*t145*9.263652961916226E-1;
out[106] = 0.0;
out[107] = 0.0;
out[108] = 0.0;
out[109] = 0.0;
out[110] = 0.0;
out[111] = 0.0;
out[112] = 0.0;
out[113] = 0.0;
out[114] = 0.0;
out[115] = 0.0;
out[116] = 0.0;
out[117] = t8*t73*(-5.347258399206903E-2)-in[2]*t7*t8*t73*1.177954989345817E-1;
out[118] = t8*t73*(-1.243706439819151E-1)+in[2]*t7*t8*t73*5.645733297639844E-2;
out[119] = 0.0;
out[120] = 0.0;
out[121] = 0.0;
out[122] = 0.0;
out[123] = 1.0;
out[124] = t57*t146;
out[125] = t58*t146;
out[126] = t146*(in[4]*t58-in[5]*t57);
out[127] = (t146*t146+1.0)*(t147+t148);
out[128] = 0.0;
out[129] = 0.0;
out[130] = 0.0;
out[131] = 0.0;
out[132] = 0.0;
out[133] = 0.0;
out[134] = 0.0;
out[135] = 0.0;
out[136] = 0.0;
out[137] = 0.0;
out[138] = 0.0;
out[139] = 0.0;
out[140] = 0.0;
out[141] = 0.0;
out[142] = 0.0;
out[143] = 0.0;
out[144] = t58;
out[145] = -t57;
out[146] = -t147-t148;
out[147] = 0.0;
out[148] = 0.0;
out[149] = 0.0;
out[150] = 0.0;
out[151] = 0.0;
out[152] = 0.0;
out[153] = 0.0;
out[154] = 0.0;
out[155] = 0.0;
out[156] = 0.0;
out[157] = 0.0;
out[158] = 0.0;
out[159] = 0.0;
out[160] = 0.0;
out[161] = 0.0;
out[162] = 0.0;
out[163] = 0.0;
out[164] = 0.0;
out[165] = 0.0;
out[166] = 0.0;
out[167] = 0.0;
out[168] = 0.0;
out[169] = 0.0;
out[170] = 0.0;
out[171] = 0.0;
out[172] = 0.0;
out[173] = 0.0;
out[174] = 0.0;
out[175] = 0.0;
out[176] = 0.0;
out[177] = 0.0;
out[178] = 0.0;
out[179] = 1.0;
out[180] = in[0]*in[24]*t31;
out[181] = in[1]*in[24]*t31;
out[182] = in[2]*in[24]*t31;
out[183] = 0.0;
out[184] = 0.0;
out[185] = 0.0;
out[186] = 0.0;
out[187] = 0.0;
out[188] = 0.0;
out[189] = 0.0;
out[190] = 0.0;
out[191] = 0.0;
out[192] = 0.0;
out[193] = 0.0;
out[194] = 0.0;
out[195] = 0.0;
out[196] = 0.0;
out[197] = 0.0;
out[198] = 0.0;
out[199] = 0.0;
out[200] = 0.0;
out[201] = 0.0;
out[202] = 0.0;
out[203] = 0.0;
out[204] = 0.0;
out[205] = 0.0;
out[206] = 0.0;
out[207] = 0.0;
out[208] = 0.0;
out[209] = 0.0;
out[210] = in[20];
out[211] = 0.0;
out[212] = 0.0;
out[213] = 0.0;
out[214] = 0.0;
out[215] = in[21];
out[216] = 0.0;
out[217] = 0.0;
out[218] = 0.0;
out[219] = 0.0;
out[220] = 0.0;
out[221] = 0.0;
out[222] = 0.0;
out[223] = 0.0;
out[224] = 0.0;
out[225] = 0.0;
out[226] = 0.0;
out[227] = 0.0;
out[228] = 0.0;
out[229] = 0.0;
out[230] = 0.0;
out[231] = in[20];
out[232] = 0.0;
out[233] = 0.0;
out[234] = 0.0;
out[235] = 0.0;
out[236] = in[21];
out[237] = 0.0;
out[238] = 0.0;
out[239] = 0.0;
out[240] = 0.0;
out[241] = 0.0;
out[242] = 0.0;
out[243] = 0.0;
out[244] = 0.0;
out[245] = 0.0;
out[246] = 0.0;
out[247] = 0.0;
out[248] = 0.0;
out[249] = 0.0;
out[250] = 0.0;
out[251] = 0.0;
out[252] = in[20];
out[253] = 0.0;
out[254] = 0.0;
out[255] = 0.0;
out[256] = 0.0;
out[257] = in[21];
out[258] = 0.0;
out[259] = 0.0;
out[260] = 0.0;
out[261] = 0.0;
out[262] = 0.0;
out[263] = 0.0;
out[264] = 0.0;
out[265] = 0.0;
out[266] = 0.0;
out[267] = 0.0;
out[268] = 0.0;
out[269] = 0.0;
out[270] = 0.0;
out[271] = 0.0;
out[272] = 0.0;
out[273] = in[20];
out[274] = 0.0;
out[275] = 0.0;
out[276] = 0.0;
out[277] = 0.0;
out[278] = in[21];
out[279] = 0.0;
out[280] = 0.0;
out[281] = 0.0;
out[282] = 0.0;
out[283] = 0.0;
out[284] = 0.0;
out[285] = 0.0;
out[286] = 0.0;
out[287] = 0.0;
out[288] = 0.0;
out[289] = 0.0;
out[290] = 0.0;
out[291] = 0.0;
out[292] = 0.0;
out[293] = 0.0;
out[294] = in[20];
out[295] = 0.0;
out[296] = 0.0;
out[297] = 0.0;
out[298] = 0.0;
out[299] = in[21];

}

void evaluateLSQ( const double *in, double *out ){

/* optimized intermediate calculations */

const double t2 = in[0]*in[0];
const double t3 = in[1]*in[1];
const double t4 = in[2]*in[2];
const double t5 = t2+t3+t4;
const double t6 = 1.0/sqrt(t5);
const double t7 = in[1]*t6;
const double t8 = 1.0/t5;
const double t12 = t3*t8;
const double t9 = -t12+1.0;
const double t10 = 1.0/sqrt(t9);
const double t11 = 1.0/pow(t5,3.0/2.0);

/* outputs */

out[0] = in[9]+sqrt(t5);
out[1] = in[7];
out[2] = in[6];
out[3] = asin(t7);
out[4] = in[3];
out[5] = in[4];
out[6] = in[5];
out[7] = in[10]*in[22]+in[15]*in[23];
out[8] = in[11]*in[22]+in[16]*in[23];
out[9] = in[12]*in[22]+in[17]*in[23];
out[10] = in[13]*in[22]+in[18]*in[23];
out[11] = in[14]*in[22]+in[19]*in[23];
out[12] = in[0]*t6;
out[13] = t7;
out[14] = in[2]*t6;
out[15] = 0.0;
out[16] = 0.0;
out[17] = 0.0;
out[18] = 0.0;
out[19] = 0.0;
out[20] = 0.0;
out[21] = 1.0;
out[22] = 0.0;
out[23] = 0.0;
out[24] = 0.0;
out[25] = 0.0;
out[26] = 0.0;
out[27] = 0.0;
out[28] = 0.0;
out[29] = 0.0;
out[30] = 0.0;
out[31] = 0.0;
out[32] = 0.0;
out[33] = 0.0;
out[34] = 1.0;
out[35] = 0.0;
out[36] = 0.0;
out[37] = 0.0;
out[38] = 0.0;
out[39] = 0.0;
out[40] = 0.0;
out[41] = 0.0;
out[42] = 0.0;
out[43] = 0.0;
out[44] = 0.0;
out[45] = 0.0;
out[46] = 0.0;
out[47] = 0.0;
out[48] = 1.0;
out[49] = 0.0;
out[50] = 0.0;
out[51] = 0.0;
out[52] = 0.0;
out[53] = 0.0;
out[54] = 0.0;
out[55] = 0.0;
out[56] = 0.0;
out[57] = -in[0]*in[1]*t10*t11;
out[58] = t10*(t6-t3*t11);
out[59] = -in[1]*in[2]*t10*t11;
out[60] = 0.0;
out[61] = 0.0;
out[62] = 0.0;
out[63] = 0.0;
out[64] = 0.0;
out[65] = 0.0;
out[66] = 0.0;
out[67] = 0.0;
out[68] = 0.0;
out[69] = 0.0;
out[70] = 0.0;
out[71] = 0.0;
out[72] = 0.0;
out[73] = 0.0;
out[74] = 0.0;
out[75] = 1.0;
out[76] = 0.0;
out[77] = 0.0;
out[78] = 0.0;
out[79] = 0.0;
out[80] = 0.0;
out[81] = 0.0;
out[82] = 0.0;
out[83] = 0.0;
out[84] = 0.0;
out[85] = 0.0;
out[86] = 0.0;
out[87] = 0.0;
out[88] = 0.0;
out[89] = 0.0;
out[90] = 0.0;
out[91] = 1.0;
out[92] = 0.0;
out[93] = 0.0;
out[94] = 0.0;
out[95] = 0.0;
out[96] = 0.0;
out[97] = 0.0;
out[98] = 0.0;
out[99] = 0.0;
out[100] = 0.0;
out[101] = 0.0;
out[102] = 0.0;
out[103] = 0.0;
out[104] = 0.0;
out[105] = 0.0;
out[106] = 0.0;
out[107] = 1.0;
out[108] = 0.0;
out[109] = 0.0;
out[110] = 0.0;
out[111] = 0.0;
out[112] = 0.0;
out[113] = 0.0;
out[114] = 0.0;
out[115] = 0.0;
out[116] = 0.0;
out[117] = 0.0;
out[118] = 0.0;
out[119] = 0.0;
out[120] = 0.0;
out[121] = 0.0;
out[122] = 0.0;
out[123] = 0.0;
out[124] = 0.0;
out[125] = 0.0;
out[126] = 0.0;
out[127] = in[22];
out[128] = 0.0;
out[129] = 0.0;
out[130] = 0.0;
out[131] = 0.0;
out[132] = 0.0;
out[133] = 0.0;
out[134] = 0.0;
out[135] = 0.0;
out[136] = 0.0;
out[137] = 0.0;
out[138] = 0.0;
out[139] = 0.0;
out[140] = 0.0;
out[141] = 0.0;
out[142] = 0.0;
out[143] = in[22];
out[144] = 0.0;
out[145] = 0.0;
out[146] = 0.0;
out[147] = 0.0;
out[148] = 0.0;
out[149] = 0.0;
out[150] = 0.0;
out[151] = 0.0;
out[152] = 0.0;
out[153] = 0.0;
out[154] = 0.0;
out[155] = 0.0;
out[156] = 0.0;
out[157] = 0.0;
out[158] = 0.0;
out[159] = in[22];
out[160] = 0.0;
out[161] = 0.0;
out[162] = 0.0;
out[163] = 0.0;
out[164] = 0.0;
out[165] = 0.0;
out[166] = 0.0;
out[167] = 0.0;
out[168] = 0.0;
out[169] = 0.0;
out[170] = 0.0;
out[171] = 0.0;
out[172] = 0.0;
out[173] = 0.0;
out[174] = 0.0;
out[175] = in[22];
out[176] = 0.0;
out[177] = 0.0;
out[178] = 0.0;
out[179] = 0.0;
out[180] = 0.0;
out[181] = 0.0;
out[182] = 0.0;
out[183] = 0.0;
out[184] = 0.0;
out[185] = 0.0;
out[186] = 0.0;
out[187] = 0.0;
out[188] = 0.0;
out[189] = 0.0;
out[190] = 0.0;
out[191] = in[22];
out[192] = 0.0;
out[193] = 0.0;
out[194] = 0.0;
out[195] = 0.0;
out[196] = 0.0;
out[197] = 0.0;
out[198] = 0.0;
out[199] = 0.0;
out[200] = 0.0;
out[201] = 0.0;
out[202] = 0.0;
out[203] = 0.0;
out[204] = 0.0;
out[205] = 0.0;
out[206] = 0.0;
out[207] = 0.0;
out[208] = 0.0;
out[209] = 0.0;
out[210] = 0.0;
out[211] = 0.0;
out[212] = 0.0;
out[213] = 0.0;
out[214] = 0.0;
out[215] = 0.0;
out[216] = 0.0;
out[217] = 0.0;
out[218] = 0.0;
out[219] = 0.0;
out[220] = 0.0;
out[221] = 0.0;
out[222] = 0.0;
out[223] = 0.0;
out[224] = 0.0;
out[225] = 0.0;
out[226] = 0.0;
out[227] = in[23];
out[228] = 0.0;
out[229] = 0.0;
out[230] = 0.0;
out[231] = 0.0;
out[232] = 0.0;
out[233] = in[23];
out[234] = 0.0;
out[235] = 0.0;
out[236] = 0.0;
out[237] = 0.0;
out[238] = 0.0;
out[239] = in[23];
out[240] = 0.0;
out[241] = 0.0;
out[242] = 0.0;
out[243] = 0.0;
out[244] = 0.0;
out[245] = in[23];
out[246] = 0.0;
out[247] = 0.0;
out[248] = 0.0;
out[249] = 0.0;
out[250] = 0.0;
out[251] = in[23];

}

void evaluateLSQEndTerm( const double *in, double *out ){

/* optimized intermediate calculations */

const double t2 = in[0]*in[0];
const double t3 = in[1]*in[1];
const double t4 = in[2]*in[2];
const double t5 = t2+t3+t4;
const double t6 = 1.0/sqrt(t5);
const double t7 = in[1]*t6;
const double t8 = 1.0/t5;
const double t12 = t3*t8;
const double t9 = -t12+1.0;
const double t10 = 1.0/sqrt(t9);
const double t11 = 1.0/pow(t5,3.0/2.0);

/* outputs */

out[0] = in[9]+sqrt(t5);
out[1] = in[7];
out[2] = in[6];
out[3] = asin(t7);
out[4] = in[3];
out[5] = in[4];
out[6] = in[5];
out[7] = in[0]*t6;
out[8] = t7;
out[9] = in[2]*t6;
out[10] = 0.0;
out[11] = 0.0;
out[12] = 0.0;
out[13] = 0.0;
out[14] = 0.0;
out[15] = 0.0;
out[16] = 1.0;
out[17] = 0.0;
out[18] = 0.0;
out[19] = 0.0;
out[20] = 0.0;
out[21] = 0.0;
out[22] = 0.0;
out[23] = 0.0;
out[24] = 0.0;
out[25] = 0.0;
out[26] = 0.0;
out[27] = 0.0;
out[28] = 0.0;
out[29] = 1.0;
out[30] = 0.0;
out[31] = 0.0;
out[32] = 0.0;
out[33] = 0.0;
out[34] = 0.0;
out[35] = 0.0;
out[36] = 0.0;
out[37] = 0.0;
out[38] = 0.0;
out[39] = 0.0;
out[40] = 0.0;
out[41] = 0.0;
out[42] = 0.0;
out[43] = 1.0;
out[44] = 0.0;
out[45] = 0.0;
out[46] = 0.0;
out[47] = 0.0;
out[48] = 0.0;
out[49] = 0.0;
out[50] = 0.0;
out[51] = 0.0;
out[52] = -in[0]*in[1]*t10*t11;
out[53] = t10*(t6-t3*t11);
out[54] = -in[1]*in[2]*t10*t11;
out[55] = 0.0;
out[56] = 0.0;
out[57] = 0.0;
out[58] = 0.0;
out[59] = 0.0;
out[60] = 0.0;
out[61] = 0.0;
out[62] = 0.0;
out[63] = 0.0;
out[64] = 0.0;
out[65] = 0.0;
out[66] = 0.0;
out[67] = 0.0;
out[68] = 0.0;
out[69] = 0.0;
out[70] = 1.0;
out[71] = 0.0;
out[72] = 0.0;
out[73] = 0.0;
out[74] = 0.0;
out[75] = 0.0;
out[76] = 0.0;
out[77] = 0.0;
out[78] = 0.0;
out[79] = 0.0;
out[80] = 0.0;
out[81] = 0.0;
out[82] = 0.0;
out[83] = 0.0;
out[84] = 0.0;
out[85] = 0.0;
out[86] = 1.0;
out[87] = 0.0;
out[88] = 0.0;
out[89] = 0.0;
out[90] = 0.0;
out[91] = 0.0;
out[92] = 0.0;
out[93] = 0.0;
out[94] = 0.0;
out[95] = 0.0;
out[96] = 0.0;
out[97] = 0.0;
out[98] = 0.0;
out[99] = 0.0;
out[100] = 0.0;
out[101] = 0.0;
out[102] = 1.0;
out[103] = 0.0;
out[104] = 0.0;
out[105] = 0.0;
out[106] = 0.0;
out[107] = 0.0;
out[108] = 0.0;
out[109] = 0.0;
out[110] = 0.0;
out[111] = 0.0;

}

// ////////////////////////////////////////////////////////////////////////
// numerical model
// ////////////////////////////////////////////////////////////////////////

void rhs_num( const double *in, double *out ){

/* optimized intermediate calculations */

const double t2 = 1.0/in[0];
const double t3 = in[2]*t2;
const double t4 = atan(t3);
const double t5 = 1.0/(in[0]*in[0]);
const double t6 = in[2]*in[2];
const double t7 = t5*t6;
const double t8 = t7+1.0;
const double t9 = 1.0/sqrt(t8);
const double t10 = t4*t4;
const double t11 = in[0]*in[0];
const double t12 = in[1]*in[1];
const double t13 = t11*2.773E-1;
const double t14 = t12*2.773E-1;
const double t15 = t6*2.773E-1;
const double t16 = t13+t14+t15;
const double t17 = cos(in[7]);
const double t18 = t4*1.0852E1;
const double t19 = t10*3.49278E1;
const double t20 = t4*t10*3.45338E1;
const double t21 = t18-t19+t20-1.1754E-1;
const double t22 = t4*4.6194E-1;
const double t23 = t10*1.1993;
const double t24 = t22+t23+6.2874E-2;
const double t25 = t6+t11+t12;
const double t26 = 1.0/sqrt(t25);
const double t27 = in[1]*t26;
const double t28 = asin(t27);
const double t29 = in[17]*5.7723E-2;
const double t30 = t28*1.5419E-2;
const double t31 = in[3]*t26*2.1327355E-1;
const double t43 = in[5]*t26*1.5132075E-2;
const double t32 = t29+t30+t31-t43;
const double t33 = in[18]*6.0945E-2;
const double t34 = in[3]*t26*1.08589635E-1;
const double t35 = in[5]*t26*1.0706801E-1;
const double t41 = t28*4.2987E-2;
const double t36 = t33+t34+t35-t41;
const double t37 = t11*7.18207E-1;
const double t38 = t12*7.18207E-1;
const double t39 = t6*7.18207E-1;
const double t40 = t37+t38+t39;
const double t42 = t9*t36;
const double t44 = in[2]*t2*t9*t32;
const double t45 = t42+t44;
const double t46 = t9*t32;
const double t47 = t46-in[2]*t2*t9*t36;
const double t48 = cos(in[6]);
const double t49 = sin(in[6]);

/* rhs */

out[0] = in[15]*4.713471698113208-sin(in[7])*(9.81E2/1.0E2)+in[1]*in[5]-in[2]*in[4]-t16*(t9*t24-in[2]*t2*t9*t21)*(2.0E1/5.3E1);
out[1] = -in[0]*in[5]+in[2]*in[3]-t16*t28*1.159735849056604E-1+t17*t49*(9.81E2/1.0E2);
out[2] = in[0]*in[4]-in[1]*in[3]+t17*t48*(9.81E2/1.0E2)-t16*(t9*t21+in[2]*t2*t9*t24)*(2.0E1/5.3E1);
out[3] = t40*t45*(-9.263652961916226E-1)-t40*t47*6.433024169447254+in[4]*(in[3]*2.270436E-2-in[5]*7.616616999999998E-2)*1.22697390224056E1;
out[4] = in[3]*in[5]*9.181328545780969E-1-(in[3]*in[3])*1.93639394716594E-1+(in[5]*in[5])*1.93639394716594E-1-(t6*4.9914E-2+t11*4.9914E-2+t12*4.9914E-2)*(in[16]*3.5262+t4*2.0621+in[4]*t26*4.497507-9.8911E-2)*2.564760194921775;
out[5] = t40*t45*(-2.040702994206499)-t40*t47*9.263652961916226E-1-in[4]*(in[3]*3.148557560000001E-2+in[5]*2.270436E-2)*1.22697390224056E1;
out[6] = in[3]+tan(in[7])*(in[4]*t49+in[5]*t48);
out[7] = in[4]*t48-in[5]*t49;
out[8] = in[19];
out[9] = in[24]*sqrt(t25);
out[10] = in[10]*in[20]+in[15]*in[21];
out[11] = in[11]*in[20]+in[16]*in[21];
out[12] = in[12]*in[20]+in[17]*in[21];
out[13] = in[13]*in[20]+in[18]*in[21];
out[14] = in[14]*in[20]+in[19]*in[21];

}

void rhs_eval( double *in, double *out ){

/* optimized intermediate calculations */

const double t2 = 1.0/in[0];
const double t3 = in[2]*t2;
const double t4 = atan(t3);
const double t5 = 1.0/(in[0]*in[0]);
const double t6 = in[2]*in[2];
const double t7 = t5*t6;
const double t8 = t7+1.0;
const double t9 = 1.0/sqrt(t8);
const double t10 = t4*t4;
const double t11 = in[0]*in[0];
const double t12 = in[1]*in[1];
const double t13 = t11*2.773E-1;
const double t14 = t12*2.773E-1;
const double t15 = t6*2.773E-1;
const double t16 = t13+t14+t15;
const double t17 = cos(in[7]);
const double t18 = t4*1.0852E1;
const double t19 = t10*3.49278E1;
const double t20 = t4*t10*3.45338E1;
const double t21 = t18-t19+t20-1.1754E-1;
const double t22 = t4*4.6194E-1;
const double t23 = t10*1.1993;
const double t24 = t22+t23+6.2874E-2;
const double t25 = t6+t11+t12;
const double t26 = 1.0/sqrt(t25);
const double t27 = in[1]*t26;
const double t28 = asin(t27);
const double t29 = in[17]*5.7723E-2;
const double t30 = t28*1.5419E-2;
const double t31 = in[3]*t26*2.1327355E-1;
const double t43 = in[5]*t26*1.5132075E-2;
const double t32 = t29+t30+t31-t43;
const double t33 = in[18]*6.0945E-2;
const double t34 = in[3]*t26*1.08589635E-1;
const double t35 = in[5]*t26*1.0706801E-1;
const double t41 = t28*4.2987E-2;
const double t36 = t33+t34+t35-t41;
const double t37 = t11*7.18207E-1;
const double t38 = t12*7.18207E-1;
const double t39 = t6*7.18207E-1;
const double t40 = t37+t38+t39;
const double t42 = t9*t36;
const double t44 = in[2]*t2*t9*t32;
const double t45 = t42+t44;
const double t46 = t9*t32;
const double t47 = t46-in[2]*t2*t9*t36;
const double t48 = cos(in[6]);
const double t49 = sin(in[6]);

/* rhs */

out[0] = in[15]*4.713471698113208-sin(in[7])*(9.81E2/1.0E2)+in[1]*in[5]-in[2]*in[4]-t16*(t9*t24-in[2]*t2*t9*t21)*(2.0E1/5.3E1);
out[1] = -in[0]*in[5]+in[2]*in[3]-t16*t28*1.159735849056604E-1+t17*t49*(9.81E2/1.0E2);
out[2] = in[0]*in[4]-in[1]*in[3]+t17*t48*(9.81E2/1.0E2)-t16*(t9*t21+in[2]*t2*t9*t24)*(2.0E1/5.3E1);
out[3] = t40*t45*(-9.263652961916226E-1)-t40*t47*6.433024169447254+in[4]*(in[3]*2.270436E-2-in[5]*7.616616999999998E-2)*1.22697390224056E1;
out[4] = in[3]*in[5]*9.181328545780969E-1-(in[3]*in[3])*1.93639394716594E-1+(in[5]*in[5])*1.93639394716594E-1-(t6*4.9914E-2+t11*4.9914E-2+t12*4.9914E-2)*(in[16]*3.5262+t4*2.0621+in[4]*t26*4.497507-9.8911E-2)*2.564760194921775;
out[5] = t40*t45*(-2.040702994206499)-t40*t47*9.263652961916226E-1-in[4]*(in[3]*3.148557560000001E-2+in[5]*2.270436E-2)*1.22697390224056E1;
out[6] = in[3]+tan(in[7])*(in[4]*t49+in[5]*t48);
out[7] = in[4]*t48-in[5]*t49;
out[8] = in[19];
out[9] = in[24]*sqrt(t25);
out[10] = in[10]*in[20]+in[15]*in[21];
out[11] = in[11]*in[20]+in[16]*in[21];
out[12] = in[12]*in[20]+in[17]*in[21];
out[13] = in[13]*in[20]+in[18]*in[21];
out[14] = in[14]*in[20]+in[19]*in[21];

}

void rhs_jac_num( const double *in, double *out ){

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

void lsq_obj_eval( double *in, double *out ){

/* optimized intermediate calculations */

const double t2 = in[0]*in[0];
const double t3 = in[1]*in[1];
const double t4 = in[2]*in[2];
const double t5 = t2+t3+t4;

/* outputs */

out[0] = in[9]+sqrt(t5);
out[1] = in[7];
out[2] = in[6];
out[3] = asin(in[1]*1.0/sqrt(t5));
out[4] = in[3];
out[5] = in[4];
out[6] = in[5];
out[7] = in[10]*in[22]+in[15]*in[23];
out[8] = in[11]*in[22]+in[16]*in[23];
out[9] = in[12]*in[22]+in[17]*in[23];
out[10] = in[13]*in[22]+in[18]*in[23];
out[11] = in[14]*in[22]+in[19]*in[23];

}

void evaluateLSQ_num( const double *in, double *out ){

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

void lsq_objN_eval( double *in, double *out ){

/* optimized intermediate calculations */

const double t2 = in[0]*in[0];
const double t3 = in[1]*in[1];
const double t4 = in[2]*in[2];
const double t5 = t2+t3+t4;

/* outputs */

out[0] = in[9]+sqrt(t5);
out[1] = in[7];
out[2] = in[6];
out[3] = asin(in[1]*1.0/sqrt(t5));
out[4] = in[3];
out[5] = in[4];
out[6] = in[5];

}

void evaluateLSQEndTerm_num( const double *in, double *out ){

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
    lsq_obj_eval( in_Delta, f_Delta_m );
    in_Delta[i] = in[i] + Delta;
    lsq_obj_eval( in_Delta, f_Delta_p );
    in_Delta[i] = in[i];
 
    for (j = 0; j < ACADO_NYN; j=j+1) {
        out[ACADO_NYN+j*ACADO_NX+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;
    }
 
}

}

// ////////////////////////////////////////////////////////////////////////
// main
// ////////////////////////////////////////////////////////////////////////

int main() {
    
    const double in[ACADO_NX+ACADO_NU+ACADO_NOD] = {13.0, 0.0, 1.0,         // states (body velocities)
            0.1, -0.15, 0.06,                                               // states (body rates)
            -0.05, 0.07,                                                    // states (attitude)
            0.0, 0.0,                                                       // states (dummy, intg)
            0.0, 0.0, 0.0, 0.0, 0.0,                                        // states (x_robust)
            0.5, -0.01, 0.0, 0.0, 0.0,                                      // ctrls
            -8.0, 1.0, -127.0, 16.0, 0.0};                                  // onlinedata (W_robust, kiV)
            
    double out_rhs[ACADO_NX];
    double out_rhs_jac[ACADO_NX*(ACADO_NX+ACADO_NU)];
    double out_lsq[ACADO_NY+ACADO_NY*(ACADO_NX+ACADO_NU)];
    double out_lsqN[ACADO_NYN+ACADO_NYN*ACADO_NX];

    double out_rhs_num[ACADO_NX];
    double out_rhs_jac_num[ACADO_NX*(ACADO_NX+ACADO_NU)];
    double out_lsq_num[ACADO_NY+ACADO_NY*(ACADO_NX+ACADO_NU)];
    double out_lsqN_num[ACADO_NYN+ACADO_NYN*ACADO_NX];
    
    rhs( in, out_rhs );
    rhs_jac( in, out_rhs_jac );
    evaluateLSQ( in, out_lsq );
    evaluateLSQEndTerm( in, out_lsqN );
    
    rhs_num( in, out_rhs_num );
    rhs_jac_num( in, out_rhs_jac_num );
    evaluateLSQ_num( in, out_lsq_num );
    evaluateLSQEndTerm_num( in, out_lsqN_num );
    
    fcloseall();
    FILE *fp;
    fp = fopen("model_output.txt","w+");
    
    int i;
    for (i = 0; i < ACADO_NX*(ACADO_NX+ACADO_NU); i=i+1) {
                
        if ( i < ACADO_NX ) {

            fprintf(fp, "%f; %f; %f; %f; %f; %f; %f; %f; \n",
                    out_rhs[i], out_rhs_num[i], out_rhs_jac[i], out_rhs_jac_num[i],
                    out_lsq[i], out_lsq_num[i], out_lsqN[i], out_lsqN_num[i]);
        
        } else if ( i < ACADO_NYN+ACADO_NYN*ACADO_NX ) {
            
            fprintf(fp, "%f; %f; %f; %f; %f; %f; %f; %f; \n",
                    999.9, 999.9, out_rhs_jac[i], out_rhs_jac_num[i],
                    out_lsq[i], out_lsq_num[i], out_lsqN[i], out_lsqN_num[i]);
            
        } else if ( i < ACADO_NY+ACADO_NY*(ACADO_NX+ACADO_NU) ) {
            
            fprintf(fp, "%f; %f; %f; %f; %f; %f; %f; %f; \n",
                    999.9, 999.9, out_rhs_jac[i], out_rhs_jac_num[i],
                    out_lsq[i], out_lsq_num[i], 999.9, 999.9);
            
        } else {
            
            fprintf(fp, "%f; %f; %f; %f; %f; %f; %f; %f; \n",
                    999.9, 999.9, out_rhs_jac[i], out_rhs_jac_num[i],
                    999.9, 999.9, 999.9, 999.9);
            
        }
        
    }
        
    fclose(fp);
    
    return 0;
    
}
    