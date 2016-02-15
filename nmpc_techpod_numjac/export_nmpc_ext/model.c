#include "acado_common.h"
#include <string.h>

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

void rhs_jac( const double *in, double *out ){

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

void evaluateLSQ( const double *in, double *out ){

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

void evaluateLSQEndTerm( const double *in, double *out ){

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