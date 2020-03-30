/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 4 + 3];

acadoWorkspace.state[32] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.state[33] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.state[34] = acadoVariables.u[lRun1 * 3 + 2];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 4] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 4 + 4];
acadoWorkspace.d[lRun1 * 4 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 4 + 5];
acadoWorkspace.d[lRun1 * 4 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 4 + 6];
acadoWorkspace.d[lRun1 * 4 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 4 + 7];

acadoWorkspace.evGx[lRun1 * 16] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 16 + 1] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 16 + 2] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 16 + 3] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 16 + 4] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 16 + 5] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 16 + 6] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 16 + 7] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 16 + 8] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 16 + 9] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 16 + 10] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 16 + 11] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 16 + 12] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 16 + 13] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 16 + 14] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 16 + 15] = acadoWorkspace.state[19];

acadoWorkspace.evGu[lRun1 * 12] = acadoWorkspace.state[20];
acadoWorkspace.evGu[lRun1 * 12 + 1] = acadoWorkspace.state[21];
acadoWorkspace.evGu[lRun1 * 12 + 2] = acadoWorkspace.state[22];
acadoWorkspace.evGu[lRun1 * 12 + 3] = acadoWorkspace.state[23];
acadoWorkspace.evGu[lRun1 * 12 + 4] = acadoWorkspace.state[24];
acadoWorkspace.evGu[lRun1 * 12 + 5] = acadoWorkspace.state[25];
acadoWorkspace.evGu[lRun1 * 12 + 6] = acadoWorkspace.state[26];
acadoWorkspace.evGu[lRun1 * 12 + 7] = acadoWorkspace.state[27];
acadoWorkspace.evGu[lRun1 * 12 + 8] = acadoWorkspace.state[28];
acadoWorkspace.evGu[lRun1 * 12 + 9] = acadoWorkspace.state[29];
acadoWorkspace.evGu[lRun1 * 12 + 10] = acadoWorkspace.state[30];
acadoWorkspace.evGu[lRun1 * 12 + 11] = acadoWorkspace.state[31];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = u[0];
out[5] = u[1];
out[6] = u[2];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[7];
tmpQ1[5] = + tmpQ2[8];
tmpQ1[6] = + tmpQ2[9];
tmpQ1[7] = + tmpQ2[10];
tmpQ1[8] = + tmpQ2[14];
tmpQ1[9] = + tmpQ2[15];
tmpQ1[10] = + tmpQ2[16];
tmpQ1[11] = + tmpQ2[17];
tmpQ1[12] = + tmpQ2[21];
tmpQ1[13] = + tmpQ2[22];
tmpQ1[14] = + tmpQ2[23];
tmpQ1[15] = + tmpQ2[24];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[28];
tmpR2[1] = +tmpObjS[29];
tmpR2[2] = +tmpObjS[30];
tmpR2[3] = +tmpObjS[31];
tmpR2[4] = +tmpObjS[32];
tmpR2[5] = +tmpObjS[33];
tmpR2[6] = +tmpObjS[34];
tmpR2[7] = +tmpObjS[35];
tmpR2[8] = +tmpObjS[36];
tmpR2[9] = +tmpObjS[37];
tmpR2[10] = +tmpObjS[38];
tmpR2[11] = +tmpObjS[39];
tmpR2[12] = +tmpObjS[40];
tmpR2[13] = +tmpObjS[41];
tmpR2[14] = +tmpObjS[42];
tmpR2[15] = +tmpObjS[43];
tmpR2[16] = +tmpObjS[44];
tmpR2[17] = +tmpObjS[45];
tmpR2[18] = +tmpObjS[46];
tmpR2[19] = +tmpObjS[47];
tmpR2[20] = +tmpObjS[48];
tmpR1[0] = + tmpR2[4];
tmpR1[1] = + tmpR2[5];
tmpR1[2] = + tmpR2[6];
tmpR1[3] = + tmpR2[11];
tmpR1[4] = + tmpR2[12];
tmpR1[5] = + tmpR2[13];
tmpR1[6] = + tmpR2[18];
tmpR1[7] = + tmpR2[19];
tmpR1[8] = + tmpR2[20];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = + tmpQN2[12];
tmpQN1[13] = + tmpQN2[13];
tmpQN1[14] = + tmpQN2[14];
tmpQN1[15] = + tmpQN2[15];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 20; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 3];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 3 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 7] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 7 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 7 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 7 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 7 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 7 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 7 + 6] = acadoWorkspace.objValueOut[6];

acado_setObjQ1Q2( &(acadoVariables.W[ runObj * 49 ]), &(acadoWorkspace.Q1[ runObj * 16 ]), &(acadoWorkspace.Q2[ runObj * 28 ]) );

acado_setObjR1R2( &(acadoVariables.W[ runObj * 49 ]), &(acadoWorkspace.R1[ runObj * 9 ]), &(acadoWorkspace.R2[ runObj * 21 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[80];
acadoWorkspace.objValueIn[1] = acadoVariables.x[81];
acadoWorkspace.objValueIn[2] = acadoVariables.x[82];
acadoWorkspace.objValueIn[3] = acadoVariables.x[83];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3];
dNew[1] += + Gx1[4]*dOld[0] + Gx1[5]*dOld[1] + Gx1[6]*dOld[2] + Gx1[7]*dOld[3];
dNew[2] += + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3];
dNew[3] += + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[4] + Gx1[2]*Gx2[8] + Gx1[3]*Gx2[12];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[9] + Gx1[3]*Gx2[13];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[14];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[15];
Gx3[4] = + Gx1[4]*Gx2[0] + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[12];
Gx3[5] = + Gx1[4]*Gx2[1] + Gx1[5]*Gx2[5] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[13];
Gx3[6] = + Gx1[4]*Gx2[2] + Gx1[5]*Gx2[6] + Gx1[6]*Gx2[10] + Gx1[7]*Gx2[14];
Gx3[7] = + Gx1[4]*Gx2[3] + Gx1[5]*Gx2[7] + Gx1[6]*Gx2[11] + Gx1[7]*Gx2[15];
Gx3[8] = + Gx1[8]*Gx2[0] + Gx1[9]*Gx2[4] + Gx1[10]*Gx2[8] + Gx1[11]*Gx2[12];
Gx3[9] = + Gx1[8]*Gx2[1] + Gx1[9]*Gx2[5] + Gx1[10]*Gx2[9] + Gx1[11]*Gx2[13];
Gx3[10] = + Gx1[8]*Gx2[2] + Gx1[9]*Gx2[6] + Gx1[10]*Gx2[10] + Gx1[11]*Gx2[14];
Gx3[11] = + Gx1[8]*Gx2[3] + Gx1[9]*Gx2[7] + Gx1[10]*Gx2[11] + Gx1[11]*Gx2[15];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[4] + Gx1[14]*Gx2[8] + Gx1[15]*Gx2[12];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[5] + Gx1[14]*Gx2[9] + Gx1[15]*Gx2[13];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[6] + Gx1[14]*Gx2[10] + Gx1[15]*Gx2[14];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[7] + Gx1[14]*Gx2[11] + Gx1[15]*Gx2[15];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[6] + Gx1[3]*Gu1[9];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[7] + Gx1[3]*Gu1[10];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[11];
Gu2[3] = + Gx1[4]*Gu1[0] + Gx1[5]*Gu1[3] + Gx1[6]*Gu1[6] + Gx1[7]*Gu1[9];
Gu2[4] = + Gx1[4]*Gu1[1] + Gx1[5]*Gu1[4] + Gx1[6]*Gu1[7] + Gx1[7]*Gu1[10];
Gu2[5] = + Gx1[4]*Gu1[2] + Gx1[5]*Gu1[5] + Gx1[6]*Gu1[8] + Gx1[7]*Gu1[11];
Gu2[6] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[3] + Gx1[10]*Gu1[6] + Gx1[11]*Gu1[9];
Gu2[7] = + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[4] + Gx1[10]*Gu1[7] + Gx1[11]*Gu1[10];
Gu2[8] = + Gx1[8]*Gu1[2] + Gx1[9]*Gu1[5] + Gx1[10]*Gu1[8] + Gx1[11]*Gu1[11];
Gu2[9] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[3] + Gx1[14]*Gu1[6] + Gx1[15]*Gu1[9];
Gu2[10] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[4] + Gx1[14]*Gu1[7] + Gx1[15]*Gu1[10];
Gu2[11] = + Gx1[12]*Gu1[2] + Gx1[13]*Gu1[5] + Gx1[14]*Gu1[8] + Gx1[15]*Gu1[11];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 180) + (iCol * 3)] += + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 1)] += + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 2)] += + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3)] += + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 1)] += + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 2)] += + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3)] += + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 1)] += + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 2)] += + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 180) + (iCol * 3)] = R11[0] + (real_t)1.0000000000000000e-10;
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 1)] = R11[1];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 2)] = R11[2];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3)] = R11[3];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 1)] = R11[4] + (real_t)1.0000000000000000e-10;
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 2)] = R11[5];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3)] = R11[6];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 1)] = R11[7];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 2)] = R11[8] + (real_t)1.0000000000000000e-10;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 180) + (iCol * 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 2)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 180) + (iCol * 3)] = acadoWorkspace.H[(iCol * 180) + (iRow * 3)];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 180 + 60) + (iRow * 3)];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 180 + 120) + (iRow * 3)];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3)] = acadoWorkspace.H[(iCol * 180) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 180 + 60) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 180 + 120) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3)] = acadoWorkspace.H[(iCol * 180) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 180 + 60) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 180 + 120) + (iRow * 3 + 2)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3];
dNew[1] = + Gx1[4]*dOld[0] + Gx1[5]*dOld[1] + Gx1[6]*dOld[2] + Gx1[7]*dOld[3];
dNew[2] = + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3];
dNew[3] = + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3];
}

void acado_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + acadoWorkspace.QN1[0]*dOld[0] + acadoWorkspace.QN1[1]*dOld[1] + acadoWorkspace.QN1[2]*dOld[2] + acadoWorkspace.QN1[3]*dOld[3];
dNew[1] = + acadoWorkspace.QN1[4]*dOld[0] + acadoWorkspace.QN1[5]*dOld[1] + acadoWorkspace.QN1[6]*dOld[2] + acadoWorkspace.QN1[7]*dOld[3];
dNew[2] = + acadoWorkspace.QN1[8]*dOld[0] + acadoWorkspace.QN1[9]*dOld[1] + acadoWorkspace.QN1[10]*dOld[2] + acadoWorkspace.QN1[11]*dOld[3];
dNew[3] = + acadoWorkspace.QN1[12]*dOld[0] + acadoWorkspace.QN1[13]*dOld[1] + acadoWorkspace.QN1[14]*dOld[2] + acadoWorkspace.QN1[15]*dOld[3];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6];
RDy1[1] = + R2[7]*Dy1[0] + R2[8]*Dy1[1] + R2[9]*Dy1[2] + R2[10]*Dy1[3] + R2[11]*Dy1[4] + R2[12]*Dy1[5] + R2[13]*Dy1[6];
RDy1[2] = + R2[14]*Dy1[0] + R2[15]*Dy1[1] + R2[16]*Dy1[2] + R2[17]*Dy1[3] + R2[18]*Dy1[4] + R2[19]*Dy1[5] + R2[20]*Dy1[6];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6];
QDy1[1] = + Q2[7]*Dy1[0] + Q2[8]*Dy1[1] + Q2[9]*Dy1[2] + Q2[10]*Dy1[3] + Q2[11]*Dy1[4] + Q2[12]*Dy1[5] + Q2[13]*Dy1[6];
QDy1[2] = + Q2[14]*Dy1[0] + Q2[15]*Dy1[1] + Q2[16]*Dy1[2] + Q2[17]*Dy1[3] + Q2[18]*Dy1[4] + Q2[19]*Dy1[5] + Q2[20]*Dy1[6];
QDy1[3] = + Q2[21]*Dy1[0] + Q2[22]*Dy1[1] + Q2[23]*Dy1[2] + Q2[24]*Dy1[3] + Q2[25]*Dy1[4] + Q2[26]*Dy1[5] + Q2[27]*Dy1[6];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[3]*QDy1[1] + E1[6]*QDy1[2] + E1[9]*QDy1[3];
U1[1] += + E1[1]*QDy1[0] + E1[4]*QDy1[1] + E1[7]*QDy1[2] + E1[10]*QDy1[3];
U1[2] += + E1[2]*QDy1[0] + E1[5]*QDy1[1] + E1[8]*QDy1[2] + E1[11]*QDy1[3];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[3]*Gx1[4] + E1[6]*Gx1[8] + E1[9]*Gx1[12];
H101[1] += + E1[0]*Gx1[1] + E1[3]*Gx1[5] + E1[6]*Gx1[9] + E1[9]*Gx1[13];
H101[2] += + E1[0]*Gx1[2] + E1[3]*Gx1[6] + E1[6]*Gx1[10] + E1[9]*Gx1[14];
H101[3] += + E1[0]*Gx1[3] + E1[3]*Gx1[7] + E1[6]*Gx1[11] + E1[9]*Gx1[15];
H101[4] += + E1[1]*Gx1[0] + E1[4]*Gx1[4] + E1[7]*Gx1[8] + E1[10]*Gx1[12];
H101[5] += + E1[1]*Gx1[1] + E1[4]*Gx1[5] + E1[7]*Gx1[9] + E1[10]*Gx1[13];
H101[6] += + E1[1]*Gx1[2] + E1[4]*Gx1[6] + E1[7]*Gx1[10] + E1[10]*Gx1[14];
H101[7] += + E1[1]*Gx1[3] + E1[4]*Gx1[7] + E1[7]*Gx1[11] + E1[10]*Gx1[15];
H101[8] += + E1[2]*Gx1[0] + E1[5]*Gx1[4] + E1[8]*Gx1[8] + E1[11]*Gx1[12];
H101[9] += + E1[2]*Gx1[1] + E1[5]*Gx1[5] + E1[8]*Gx1[9] + E1[11]*Gx1[13];
H101[10] += + E1[2]*Gx1[2] + E1[5]*Gx1[6] + E1[8]*Gx1[10] + E1[11]*Gx1[14];
H101[11] += + E1[2]*Gx1[3] + E1[5]*Gx1[7] + E1[8]*Gx1[11] + E1[11]*Gx1[15];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 12; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1] + E1[2]*U1[2];
dNew[1] += + E1[3]*U1[0] + E1[4]*U1[1] + E1[5]*U1[2];
dNew[2] += + E1[6]*U1[0] + E1[7]*U1[1] + E1[8]*U1[2];
dNew[3] += + E1[9]*U1[0] + E1[10]*U1[1] + E1[11]*U1[2];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
g1[2] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 20 */
static const int xBoundIndices[ 20 ] = 
{ 7, 11, 15, 19, 23, 27, 31, 35, 39, 43, 47, 51, 55, 59, 63, 67, 71, 75, 79, 83 };
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.T );
acado_multGxd( acadoWorkspace.d, &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.d[ 4 ]) );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 16 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 12 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.E[ 24 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.d[ 8 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.evGx[ 32 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.E[ 36 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.E[ 48 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.E[ 60 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 48 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.d[ 12 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.evGx[ 48 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.E[ 72 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 84 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.E[ 96 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.E[ 108 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.d[ 16 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.evGx[ 64 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.E[ 120 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.E[ 132 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 144 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.E[ 156 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.E[ 168 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.d[ 20 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.evGx[ 80 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.E[ 180 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.E[ 192 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.E[ 204 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.E[ 216 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.E[ 228 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.E[ 240 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.d[ 24 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.evGx[ 96 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.E[ 252 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.E[ 264 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.E[ 276 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.E[ 288 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.E[ 300 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 312 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.E[ 324 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.d[ 28 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.evGx[ 112 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.E[ 336 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.E[ 348 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.E[ 360 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.E[ 372 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.E[ 384 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.E[ 396 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.E[ 408 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 84 ]), &(acadoWorkspace.E[ 420 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 28 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.d[ 32 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.evGx[ 128 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.E[ 432 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.E[ 444 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.E[ 456 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.E[ 468 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.E[ 480 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.E[ 492 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.E[ 504 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.E[ 516 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.E[ 528 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 32 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.d[ 36 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.evGx[ 144 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.E[ 540 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.E[ 552 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.E[ 564 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.E[ 576 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.E[ 588 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.E[ 600 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.E[ 612 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.E[ 624 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.E[ 636 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.E[ 648 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 160 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.d[ 40 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGx[ 160 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.E[ 660 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.E[ 672 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.E[ 684 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.E[ 696 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.E[ 708 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.E[ 720 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.E[ 732 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.E[ 744 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.E[ 756 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.E[ 768 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.E[ 780 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 176 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.d[ 44 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.evGx[ 176 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.E[ 792 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.E[ 804 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.E[ 816 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.E[ 828 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.E[ 840 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.E[ 852 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.E[ 864 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.E[ 876 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.E[ 888 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.E[ 900 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.E[ 912 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 132 ]), &(acadoWorkspace.E[ 924 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 44 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.d[ 48 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.evGx[ 192 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.E[ 936 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.E[ 948 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.E[ 960 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.E[ 972 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.E[ 984 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.E[ 996 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.E[ 1008 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.E[ 1020 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.E[ 1032 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.E[ 1044 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.E[ 1056 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 924 ]), &(acadoWorkspace.E[ 1068 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.E[ 1080 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 208 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.d[ 52 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.evGx[ 208 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.E[ 1092 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.E[ 1104 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.E[ 1116 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.E[ 1128 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.E[ 1140 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.E[ 1152 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.E[ 1164 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.E[ 1176 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.E[ 1188 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.E[ 1200 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.E[ 1212 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1068 ]), &(acadoWorkspace.E[ 1224 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.E[ 1236 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 156 ]), &(acadoWorkspace.E[ 1248 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 224 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 52 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.d[ 56 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.evGx[ 224 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.E[ 1260 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.E[ 1272 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.E[ 1284 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.E[ 1296 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.E[ 1308 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.E[ 1320 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.E[ 1332 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.E[ 1344 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.E[ 1356 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.E[ 1368 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.E[ 1380 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.E[ 1392 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1236 ]), &(acadoWorkspace.E[ 1404 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.E[ 1416 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 168 ]), &(acadoWorkspace.E[ 1428 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 240 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.d[ 60 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.evGx[ 240 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.E[ 1440 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.E[ 1452 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.E[ 1464 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.E[ 1476 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.E[ 1488 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.E[ 1500 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.E[ 1512 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.E[ 1524 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.E[ 1536 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.E[ 1548 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.E[ 1560 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.E[ 1572 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.E[ 1584 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.E[ 1596 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1428 ]), &(acadoWorkspace.E[ 1608 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 180 ]), &(acadoWorkspace.E[ 1620 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.d[ 64 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.evGx[ 256 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.E[ 1632 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.E[ 1644 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.E[ 1656 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.E[ 1668 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.E[ 1680 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.E[ 1692 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.E[ 1704 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.E[ 1716 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.E[ 1728 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.E[ 1740 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.E[ 1752 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.E[ 1764 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.E[ 1776 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1596 ]), &(acadoWorkspace.E[ 1788 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.E[ 1800 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1620 ]), &(acadoWorkspace.E[ 1812 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 192 ]), &(acadoWorkspace.E[ 1824 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 272 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 64 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.d[ 68 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.evGx[ 272 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.E[ 1836 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.E[ 1848 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.E[ 1860 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.E[ 1872 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.E[ 1884 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.E[ 1896 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.E[ 1908 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.E[ 1920 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.E[ 1932 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.E[ 1944 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.E[ 1956 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.E[ 1968 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.E[ 1980 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.E[ 1992 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1800 ]), &(acadoWorkspace.E[ 2004 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1812 ]), &(acadoWorkspace.E[ 2016 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.E[ 2028 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 204 ]), &(acadoWorkspace.E[ 2040 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 68 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.d[ 72 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.evGx[ 288 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.E[ 2052 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.E[ 2064 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.E[ 2076 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.E[ 2088 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.E[ 2100 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.E[ 2112 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.E[ 2124 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.E[ 2136 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.E[ 2148 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.E[ 2160 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.E[ 2172 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.E[ 2184 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.E[ 2196 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.E[ 2208 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.E[ 2220 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.E[ 2232 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2028 ]), &(acadoWorkspace.E[ 2244 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2040 ]), &(acadoWorkspace.E[ 2256 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.E[ 2268 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 304 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.d[ 76 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGx[ 304 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.E[ 2280 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.E[ 2292 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.E[ 2304 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.E[ 2316 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.E[ 2328 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.E[ 2340 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.E[ 2352 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.E[ 2364 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.E[ 2376 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.E[ 2388 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.E[ 2400 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.E[ 2412 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.E[ 2424 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.E[ 2436 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.E[ 2448 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.E[ 2460 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2244 ]), &(acadoWorkspace.E[ 2472 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.E[ 2484 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2268 ]), &(acadoWorkspace.E[ 2496 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 228 ]), &(acadoWorkspace.E[ 2508 ]) );

acado_multGxGu( &(acadoWorkspace.Q1[ 16 ]), acadoWorkspace.E, acadoWorkspace.QE );
acado_multGxGu( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 588 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 660 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 684 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 708 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 804 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 828 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 852 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 924 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 948 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 972 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 996 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 1068 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1092 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1116 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 1236 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1260 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1272 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1284 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1308 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.E[ 1428 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1452 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1476 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1500 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1596 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 1620 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1644 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1656 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1668 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1692 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1800 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1812 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1836 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1848 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1860 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1884 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1896 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 2028 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.E[ 2040 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2052 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2076 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2088 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2100 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2244 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.E[ 2268 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2280 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2292 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2316 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2328 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2340 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2364 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2376 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2388 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2412 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2424 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2436 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2448 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.QE[ 2460 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2472 ]), &(acadoWorkspace.QE[ 2472 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2484 ]), &(acadoWorkspace.QE[ 2484 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2508 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 12 ]), &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 36 ]), &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.evGx[ 48 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 180 ]), &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 252 ]), &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 336 ]), &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 540 ]), &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 660 ]), &(acadoWorkspace.evGx[ 160 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 792 ]), &(acadoWorkspace.evGx[ 176 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 936 ]), &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1092 ]), &(acadoWorkspace.evGx[ 208 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1260 ]), &(acadoWorkspace.evGx[ 224 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1440 ]), &(acadoWorkspace.evGx[ 240 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1632 ]), &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1836 ]), &(acadoWorkspace.evGx[ 272 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 2052 ]), &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 2280 ]), &(acadoWorkspace.evGx[ 304 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 84 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 264 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 348 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 444 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 552 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 672 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 804 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 948 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1104 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1272 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1452 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1644 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1848 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2064 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2292 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 204 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 276 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 360 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 456 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 564 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 684 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 816 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 960 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1116 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1284 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1464 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1656 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1860 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2076 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2304 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 372 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 468 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 576 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 696 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 828 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 972 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1128 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1296 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1476 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1668 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1872 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2088 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2316 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 168 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 228 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 588 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 708 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 840 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 984 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1140 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1308 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1488 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1680 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1884 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2100 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2328 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 312 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 396 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 492 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 600 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 720 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 852 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 996 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1152 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1320 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1500 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1692 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1896 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2112 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2340 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 324 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 408 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 504 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 612 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 732 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 864 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1008 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1164 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1332 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1512 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1704 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1908 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2124 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2352 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 420 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 516 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 624 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 744 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 876 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1020 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1176 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1344 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1524 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1716 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1920 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2136 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2364 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 528 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 636 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 756 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 888 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1032 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1188 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1356 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1536 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1728 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1932 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2148 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2376 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 648 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 768 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 900 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1044 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1200 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1368 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1548 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1740 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1944 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2160 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2388 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 780 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 912 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1056 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1212 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1380 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1560 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1752 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1956 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2172 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2400 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 924 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1068 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1224 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1392 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1572 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1764 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1968 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2184 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2412 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1080 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1236 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1404 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1584 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1776 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1980 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2196 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2424 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 156 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1248 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.H10[ 156 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1416 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 156 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1596 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 156 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1788 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 156 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1992 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 156 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2208 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 156 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2436 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 156 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 168 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1428 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.H10[ 168 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1608 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 168 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1800 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 168 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2004 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 168 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2220 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 168 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2448 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 168 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 180 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1620 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.H10[ 180 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1812 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 180 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2016 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 180 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2232 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 180 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2460 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 180 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1824 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2028 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2244 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2472 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 204 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2040 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.H10[ 204 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2256 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 204 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2484 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 204 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 216 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2268 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 216 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2496 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 216 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 228 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2508 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.H10[ 228 ]) );

acado_setBlockH11_R1( 0, 0, acadoWorkspace.R1 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 660 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1092 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1260 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1836 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2052 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2280 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 804 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 948 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1272 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1452 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1644 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1848 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2292 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 684 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1116 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1284 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1656 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1860 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2076 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2304 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 828 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 972 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1476 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1668 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2088 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2316 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 588 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 708 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1308 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1884 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2100 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2328 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 852 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 996 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1500 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1692 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1896 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2340 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2352 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 0, 10 );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 0, 11 );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 0, 12 );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 0, 13 );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 0, 14 );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 0, 15 );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 0, 16 );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 0, 17 );
acado_setBlockH11( 0, 17, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 0, 17, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 0, 17, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 0, 18 );
acado_setBlockH11( 0, 18, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 0, 18, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 0, 19 );
acado_setBlockH11( 0, 19, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 1, 1, &(acadoWorkspace.R1[ 9 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 804 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 948 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1272 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1452 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1644 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1848 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2292 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 684 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1116 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1284 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1656 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1860 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2076 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2304 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 828 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 972 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1476 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1668 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2088 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2316 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 588 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 708 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1308 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1884 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2100 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2328 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 852 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 996 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1500 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1692 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1896 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2340 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2352 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 1, 10 );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 1, 11 );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 1, 12 );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 1, 13 );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 1, 14 );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 1, 15 );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 1, 16 );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 1, 17 );
acado_setBlockH11( 1, 17, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 1, 17, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 1, 17, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 1, 18 );
acado_setBlockH11( 1, 18, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 1, 18, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 1, 19 );
acado_setBlockH11( 1, 19, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 2, 2, &(acadoWorkspace.R1[ 18 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 684 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1116 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1284 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1656 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1860 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2076 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2304 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 828 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 972 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1476 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1668 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2088 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2316 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 588 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 708 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1308 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1884 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2100 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2328 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 852 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 996 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1500 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1692 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1896 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2340 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2352 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 2, 10 );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 2, 11 );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 2, 12 );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 2, 13 );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 2, 14 );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 2, 15 );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 2, 16 );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 2, 17 );
acado_setBlockH11( 2, 17, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 2, 17, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 2, 17, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 2, 18 );
acado_setBlockH11( 2, 18, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 2, 18, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 2, 19 );
acado_setBlockH11( 2, 19, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 3, 3, &(acadoWorkspace.R1[ 27 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 828 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 972 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1476 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1668 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2088 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2316 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 588 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 708 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1308 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1884 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2100 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2328 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 852 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 996 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1500 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1692 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1896 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2340 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2352 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 3, 10 );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 3, 11 );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 3, 12 );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 3, 13 );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 3, 14 );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 3, 15 );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 3, 16 );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 3, 17 );
acado_setBlockH11( 3, 17, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 3, 17, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 3, 17, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 3, 18 );
acado_setBlockH11( 3, 18, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 3, 18, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 3, 19 );
acado_setBlockH11( 3, 19, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 4, 4, &(acadoWorkspace.R1[ 36 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 588 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 708 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1308 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1884 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2100 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2328 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 852 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 996 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1500 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1692 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1896 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2340 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2352 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 4, 10 );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 4, 11 );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 4, 12 );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 4, 13 );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 4, 14 );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 4, 15 );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 4, 16 );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 4, 17 );
acado_setBlockH11( 4, 17, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 4, 17, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 4, 17, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 4, 18 );
acado_setBlockH11( 4, 18, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 4, 18, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 4, 19 );
acado_setBlockH11( 4, 19, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 5, 5, &(acadoWorkspace.R1[ 45 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 852 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 996 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1500 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1692 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1896 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2340 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2352 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 5, 10 );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 5, 11 );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 5, 12 );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 5, 13 );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 5, 14 );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 5, 15 );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 5, 16 );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 5, 17 );
acado_setBlockH11( 5, 17, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 5, 17, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 5, 17, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 5, 18 );
acado_setBlockH11( 5, 18, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 5, 18, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 5, 19 );
acado_setBlockH11( 5, 19, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 6, 6, &(acadoWorkspace.R1[ 54 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2352 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 6, 10 );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 6, 11 );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 6, 12 );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 6, 13 );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 6, 14 );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 6, 15 );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 6, 16 );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 6, 17 );
acado_setBlockH11( 6, 17, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 6, 17, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 6, 17, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 6, 18 );
acado_setBlockH11( 6, 18, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 6, 18, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 6, 19 );
acado_setBlockH11( 6, 19, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 7, 7, &(acadoWorkspace.R1[ 63 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 7, 10 );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 7, 11 );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 7, 12 );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 7, 13 );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 7, 14 );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 7, 15 );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 7, 16 );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 7, 17 );
acado_setBlockH11( 7, 17, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 7, 17, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 7, 17, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 7, 18 );
acado_setBlockH11( 7, 18, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 7, 18, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 7, 19 );
acado_setBlockH11( 7, 19, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 8, 8, &(acadoWorkspace.R1[ 72 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 8, 10 );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 8, 11 );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 8, 12 );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 8, 13 );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 8, 14 );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 8, 15 );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 8, 16 );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 8, 17 );
acado_setBlockH11( 8, 17, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 8, 17, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 8, 17, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 8, 18 );
acado_setBlockH11( 8, 18, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 8, 18, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 8, 19 );
acado_setBlockH11( 8, 19, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 9, 9, &(acadoWorkspace.R1[ 81 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 9, 10 );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 9, 11 );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 9, 12 );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 9, 13 );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 9, 14 );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 9, 15 );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 9, 16 );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 9, 17 );
acado_setBlockH11( 9, 17, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 9, 17, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 9, 17, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 9, 18 );
acado_setBlockH11( 9, 18, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 9, 18, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 9, 19 );
acado_setBlockH11( 9, 19, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 10, 10, &(acadoWorkspace.R1[ 90 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 10, 11 );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 10, 12 );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 10, 13 );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 10, 14 );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 10, 15 );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 10, 16 );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 10, 17 );
acado_setBlockH11( 10, 17, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 10, 17, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 10, 17, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 10, 18 );
acado_setBlockH11( 10, 18, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 10, 18, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 10, 19 );
acado_setBlockH11( 10, 19, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 11, 11, &(acadoWorkspace.R1[ 99 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 924 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1068 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 11, 12 );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1068 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 11, 13 );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 11, 14 );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 11, 15 );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 11, 16 );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 11, 17 );
acado_setBlockH11( 11, 17, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 11, 17, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 11, 17, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 11, 18 );
acado_setBlockH11( 11, 18, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 11, 18, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 11, 19 );
acado_setBlockH11( 11, 19, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 12, 12, &(acadoWorkspace.R1[ 108 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1236 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 12, 13 );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1236 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 12, 14 );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 12, 15 );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 12, 16 );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 12, 17 );
acado_setBlockH11( 12, 17, &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 12, 17, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 12, 17, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 12, 18 );
acado_setBlockH11( 12, 18, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 12, 18, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 12, 19 );
acado_setBlockH11( 12, 19, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 13, 13, &(acadoWorkspace.R1[ 117 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1596 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 13, 14 );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 1596 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 13, 15 );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 1596 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 13, 16 );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 13, 17 );
acado_setBlockH11( 13, 17, &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 13, 17, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 13, 17, &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 13, 18 );
acado_setBlockH11( 13, 18, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 13, 18, &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 13, 19 );
acado_setBlockH11( 13, 19, &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 14, 14, &(acadoWorkspace.R1[ 126 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 1428 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 1800 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 14, 15 );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 1800 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 14, 16 );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 1800 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 14, 17 );
acado_setBlockH11( 14, 17, &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 14, 17, &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 14, 17, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 14, 18 );
acado_setBlockH11( 14, 18, &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 14, 18, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 14, 19 );
acado_setBlockH11( 14, 19, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 15, 15, &(acadoWorkspace.R1[ 135 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 1620 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 1812 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 15, 16 );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 1812 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 15, 17 );
acado_setBlockH11( 15, 17, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 15, 17, &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 15, 17, &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 15, 18 );
acado_setBlockH11( 15, 18, &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 15, 18, &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 15, 19 );
acado_setBlockH11( 15, 19, &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 16, 16, &(acadoWorkspace.R1[ 144 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 2028 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 2244 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 2472 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 16, 17 );
acado_setBlockH11( 16, 17, &(acadoWorkspace.E[ 2028 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 16, 17, &(acadoWorkspace.E[ 2244 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 16, 17, &(acadoWorkspace.E[ 2472 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 16, 18 );
acado_setBlockH11( 16, 18, &(acadoWorkspace.E[ 2244 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 16, 18, &(acadoWorkspace.E[ 2472 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 16, 19 );
acado_setBlockH11( 16, 19, &(acadoWorkspace.E[ 2472 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 17, 17, &(acadoWorkspace.R1[ 153 ]) );
acado_setBlockH11( 17, 17, &(acadoWorkspace.E[ 2040 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 17, 17, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 17, 17, &(acadoWorkspace.E[ 2484 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 17, 18 );
acado_setBlockH11( 17, 18, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 17, 18, &(acadoWorkspace.E[ 2484 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 17, 19 );
acado_setBlockH11( 17, 19, &(acadoWorkspace.E[ 2484 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 18, 18, &(acadoWorkspace.R1[ 162 ]) );
acado_setBlockH11( 18, 18, &(acadoWorkspace.E[ 2268 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 18, 18, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 18, 19 );
acado_setBlockH11( 18, 19, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 19, 19, &(acadoWorkspace.R1[ 171 ]) );
acado_setBlockH11( 19, 19, &(acadoWorkspace.E[ 2508 ]), &(acadoWorkspace.QE[ 2508 ]) );


acado_copyHTH( 1, 0 );
acado_copyHTH( 2, 0 );
acado_copyHTH( 2, 1 );
acado_copyHTH( 3, 0 );
acado_copyHTH( 3, 1 );
acado_copyHTH( 3, 2 );
acado_copyHTH( 4, 0 );
acado_copyHTH( 4, 1 );
acado_copyHTH( 4, 2 );
acado_copyHTH( 4, 3 );
acado_copyHTH( 5, 0 );
acado_copyHTH( 5, 1 );
acado_copyHTH( 5, 2 );
acado_copyHTH( 5, 3 );
acado_copyHTH( 5, 4 );
acado_copyHTH( 6, 0 );
acado_copyHTH( 6, 1 );
acado_copyHTH( 6, 2 );
acado_copyHTH( 6, 3 );
acado_copyHTH( 6, 4 );
acado_copyHTH( 6, 5 );
acado_copyHTH( 7, 0 );
acado_copyHTH( 7, 1 );
acado_copyHTH( 7, 2 );
acado_copyHTH( 7, 3 );
acado_copyHTH( 7, 4 );
acado_copyHTH( 7, 5 );
acado_copyHTH( 7, 6 );
acado_copyHTH( 8, 0 );
acado_copyHTH( 8, 1 );
acado_copyHTH( 8, 2 );
acado_copyHTH( 8, 3 );
acado_copyHTH( 8, 4 );
acado_copyHTH( 8, 5 );
acado_copyHTH( 8, 6 );
acado_copyHTH( 8, 7 );
acado_copyHTH( 9, 0 );
acado_copyHTH( 9, 1 );
acado_copyHTH( 9, 2 );
acado_copyHTH( 9, 3 );
acado_copyHTH( 9, 4 );
acado_copyHTH( 9, 5 );
acado_copyHTH( 9, 6 );
acado_copyHTH( 9, 7 );
acado_copyHTH( 9, 8 );
acado_copyHTH( 10, 0 );
acado_copyHTH( 10, 1 );
acado_copyHTH( 10, 2 );
acado_copyHTH( 10, 3 );
acado_copyHTH( 10, 4 );
acado_copyHTH( 10, 5 );
acado_copyHTH( 10, 6 );
acado_copyHTH( 10, 7 );
acado_copyHTH( 10, 8 );
acado_copyHTH( 10, 9 );
acado_copyHTH( 11, 0 );
acado_copyHTH( 11, 1 );
acado_copyHTH( 11, 2 );
acado_copyHTH( 11, 3 );
acado_copyHTH( 11, 4 );
acado_copyHTH( 11, 5 );
acado_copyHTH( 11, 6 );
acado_copyHTH( 11, 7 );
acado_copyHTH( 11, 8 );
acado_copyHTH( 11, 9 );
acado_copyHTH( 11, 10 );
acado_copyHTH( 12, 0 );
acado_copyHTH( 12, 1 );
acado_copyHTH( 12, 2 );
acado_copyHTH( 12, 3 );
acado_copyHTH( 12, 4 );
acado_copyHTH( 12, 5 );
acado_copyHTH( 12, 6 );
acado_copyHTH( 12, 7 );
acado_copyHTH( 12, 8 );
acado_copyHTH( 12, 9 );
acado_copyHTH( 12, 10 );
acado_copyHTH( 12, 11 );
acado_copyHTH( 13, 0 );
acado_copyHTH( 13, 1 );
acado_copyHTH( 13, 2 );
acado_copyHTH( 13, 3 );
acado_copyHTH( 13, 4 );
acado_copyHTH( 13, 5 );
acado_copyHTH( 13, 6 );
acado_copyHTH( 13, 7 );
acado_copyHTH( 13, 8 );
acado_copyHTH( 13, 9 );
acado_copyHTH( 13, 10 );
acado_copyHTH( 13, 11 );
acado_copyHTH( 13, 12 );
acado_copyHTH( 14, 0 );
acado_copyHTH( 14, 1 );
acado_copyHTH( 14, 2 );
acado_copyHTH( 14, 3 );
acado_copyHTH( 14, 4 );
acado_copyHTH( 14, 5 );
acado_copyHTH( 14, 6 );
acado_copyHTH( 14, 7 );
acado_copyHTH( 14, 8 );
acado_copyHTH( 14, 9 );
acado_copyHTH( 14, 10 );
acado_copyHTH( 14, 11 );
acado_copyHTH( 14, 12 );
acado_copyHTH( 14, 13 );
acado_copyHTH( 15, 0 );
acado_copyHTH( 15, 1 );
acado_copyHTH( 15, 2 );
acado_copyHTH( 15, 3 );
acado_copyHTH( 15, 4 );
acado_copyHTH( 15, 5 );
acado_copyHTH( 15, 6 );
acado_copyHTH( 15, 7 );
acado_copyHTH( 15, 8 );
acado_copyHTH( 15, 9 );
acado_copyHTH( 15, 10 );
acado_copyHTH( 15, 11 );
acado_copyHTH( 15, 12 );
acado_copyHTH( 15, 13 );
acado_copyHTH( 15, 14 );
acado_copyHTH( 16, 0 );
acado_copyHTH( 16, 1 );
acado_copyHTH( 16, 2 );
acado_copyHTH( 16, 3 );
acado_copyHTH( 16, 4 );
acado_copyHTH( 16, 5 );
acado_copyHTH( 16, 6 );
acado_copyHTH( 16, 7 );
acado_copyHTH( 16, 8 );
acado_copyHTH( 16, 9 );
acado_copyHTH( 16, 10 );
acado_copyHTH( 16, 11 );
acado_copyHTH( 16, 12 );
acado_copyHTH( 16, 13 );
acado_copyHTH( 16, 14 );
acado_copyHTH( 16, 15 );
acado_copyHTH( 17, 0 );
acado_copyHTH( 17, 1 );
acado_copyHTH( 17, 2 );
acado_copyHTH( 17, 3 );
acado_copyHTH( 17, 4 );
acado_copyHTH( 17, 5 );
acado_copyHTH( 17, 6 );
acado_copyHTH( 17, 7 );
acado_copyHTH( 17, 8 );
acado_copyHTH( 17, 9 );
acado_copyHTH( 17, 10 );
acado_copyHTH( 17, 11 );
acado_copyHTH( 17, 12 );
acado_copyHTH( 17, 13 );
acado_copyHTH( 17, 14 );
acado_copyHTH( 17, 15 );
acado_copyHTH( 17, 16 );
acado_copyHTH( 18, 0 );
acado_copyHTH( 18, 1 );
acado_copyHTH( 18, 2 );
acado_copyHTH( 18, 3 );
acado_copyHTH( 18, 4 );
acado_copyHTH( 18, 5 );
acado_copyHTH( 18, 6 );
acado_copyHTH( 18, 7 );
acado_copyHTH( 18, 8 );
acado_copyHTH( 18, 9 );
acado_copyHTH( 18, 10 );
acado_copyHTH( 18, 11 );
acado_copyHTH( 18, 12 );
acado_copyHTH( 18, 13 );
acado_copyHTH( 18, 14 );
acado_copyHTH( 18, 15 );
acado_copyHTH( 18, 16 );
acado_copyHTH( 18, 17 );
acado_copyHTH( 19, 0 );
acado_copyHTH( 19, 1 );
acado_copyHTH( 19, 2 );
acado_copyHTH( 19, 3 );
acado_copyHTH( 19, 4 );
acado_copyHTH( 19, 5 );
acado_copyHTH( 19, 6 );
acado_copyHTH( 19, 7 );
acado_copyHTH( 19, 8 );
acado_copyHTH( 19, 9 );
acado_copyHTH( 19, 10 );
acado_copyHTH( 19, 11 );
acado_copyHTH( 19, 12 );
acado_copyHTH( 19, 13 );
acado_copyHTH( 19, 14 );
acado_copyHTH( 19, 15 );
acado_copyHTH( 19, 16 );
acado_copyHTH( 19, 17 );
acado_copyHTH( 19, 18 );

acado_multQ1d( &(acadoWorkspace.Q1[ 16 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.Qd[ 4 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.Qd[ 8 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.Qd[ 12 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.Qd[ 16 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.Qd[ 20 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.Qd[ 24 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.d[ 28 ]), &(acadoWorkspace.Qd[ 28 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.d[ 32 ]), &(acadoWorkspace.Qd[ 32 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.Qd[ 36 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.Qd[ 40 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.d[ 44 ]), &(acadoWorkspace.Qd[ 44 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.Qd[ 48 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.d[ 52 ]), &(acadoWorkspace.Qd[ 52 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.Qd[ 56 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.d[ 64 ]), &(acadoWorkspace.Qd[ 64 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.d[ 68 ]), &(acadoWorkspace.Qd[ 68 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.Qd[ 72 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 76 ]), &(acadoWorkspace.Qd[ 76 ]) );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 12 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 36 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 72 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 120 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 180 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 252 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 336 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 432 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 540 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 660 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 792 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 936 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1092 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1260 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1440 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1632 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1836 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 2052 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 2280 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 84 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 264 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 348 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 444 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 552 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 672 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 804 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 948 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1104 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1272 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1452 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1644 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1848 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2064 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2292 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 204 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 276 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 360 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 456 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 564 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 684 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 816 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 960 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1116 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1284 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1464 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1656 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1860 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2076 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2304 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 372 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 468 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 576 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 696 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 828 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 972 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1128 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1296 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1476 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1668 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1872 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2088 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2316 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 168 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 228 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 588 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 708 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 840 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 984 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1140 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1308 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1488 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1680 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1884 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2100 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2328 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 312 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 396 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 492 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 600 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 720 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 852 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 996 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1152 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1320 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1500 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1692 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1896 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2112 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2340 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 324 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 408 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 504 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 612 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 732 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 864 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1008 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1164 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1332 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1512 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1704 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1908 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2124 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2352 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 420 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 516 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 624 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 744 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 876 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1020 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1176 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1344 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1524 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1716 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1920 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2136 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2364 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 528 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 636 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 756 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 888 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1032 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1188 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1356 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1536 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1728 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1932 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2148 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2376 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 648 ]), &(acadoWorkspace.g[ 27 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 768 ]), &(acadoWorkspace.g[ 27 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 900 ]), &(acadoWorkspace.g[ 27 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1044 ]), &(acadoWorkspace.g[ 27 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1200 ]), &(acadoWorkspace.g[ 27 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1368 ]), &(acadoWorkspace.g[ 27 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1548 ]), &(acadoWorkspace.g[ 27 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1740 ]), &(acadoWorkspace.g[ 27 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1944 ]), &(acadoWorkspace.g[ 27 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2160 ]), &(acadoWorkspace.g[ 27 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2388 ]), &(acadoWorkspace.g[ 27 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 780 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 912 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1056 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1212 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1380 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1560 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1752 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1956 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2172 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2400 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 924 ]), &(acadoWorkspace.g[ 33 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1068 ]), &(acadoWorkspace.g[ 33 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1224 ]), &(acadoWorkspace.g[ 33 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1392 ]), &(acadoWorkspace.g[ 33 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1572 ]), &(acadoWorkspace.g[ 33 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1764 ]), &(acadoWorkspace.g[ 33 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1968 ]), &(acadoWorkspace.g[ 33 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2184 ]), &(acadoWorkspace.g[ 33 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2412 ]), &(acadoWorkspace.g[ 33 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1080 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1236 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1404 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1584 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1776 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1980 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2196 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2424 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1248 ]), &(acadoWorkspace.g[ 39 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1416 ]), &(acadoWorkspace.g[ 39 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1596 ]), &(acadoWorkspace.g[ 39 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1788 ]), &(acadoWorkspace.g[ 39 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1992 ]), &(acadoWorkspace.g[ 39 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2208 ]), &(acadoWorkspace.g[ 39 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2436 ]), &(acadoWorkspace.g[ 39 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1428 ]), &(acadoWorkspace.g[ 42 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1608 ]), &(acadoWorkspace.g[ 42 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1800 ]), &(acadoWorkspace.g[ 42 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2004 ]), &(acadoWorkspace.g[ 42 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2220 ]), &(acadoWorkspace.g[ 42 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2448 ]), &(acadoWorkspace.g[ 42 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1620 ]), &(acadoWorkspace.g[ 45 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1812 ]), &(acadoWorkspace.g[ 45 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2016 ]), &(acadoWorkspace.g[ 45 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2232 ]), &(acadoWorkspace.g[ 45 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2460 ]), &(acadoWorkspace.g[ 45 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1824 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2028 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2244 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2472 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2040 ]), &(acadoWorkspace.g[ 51 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2256 ]), &(acadoWorkspace.g[ 51 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2484 ]), &(acadoWorkspace.g[ 51 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2268 ]), &(acadoWorkspace.g[ 54 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2496 ]), &(acadoWorkspace.g[ 54 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2508 ]), &(acadoWorkspace.g[ 57 ]) );
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 4;
lRun4 = ((lRun3) / (4)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = (((((lRun4) * (lRun4-1)) / (2)) + (lRun2)) * (4)) + ((lRun3) % (4));
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 3)] = acadoWorkspace.E[lRun5 * 3];
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 3 + 1)] = acadoWorkspace.E[lRun5 * 3 + 1];
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 3 + 2)] = acadoWorkspace.E[lRun5 * 3 + 2];
}
}

}

void acado_condenseFdb(  )
{
int lRun1;
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];

for (lRun1 = 0; lRun1 < 140; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 21 ]), &(acadoWorkspace.Dy[ 7 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 42 ]), &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 63 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 84 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 105 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 126 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 147 ]), &(acadoWorkspace.Dy[ 49 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 168 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 189 ]), &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 210 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 231 ]), &(acadoWorkspace.Dy[ 77 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 252 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 273 ]), &(acadoWorkspace.Dy[ 91 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 294 ]), &(acadoWorkspace.Dy[ 98 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 315 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 336 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 357 ]), &(acadoWorkspace.Dy[ 119 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 378 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 399 ]), &(acadoWorkspace.Dy[ 133 ]), &(acadoWorkspace.g[ 57 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 28 ]), &(acadoWorkspace.Dy[ 7 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 56 ]), &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 84 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 112 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 140 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 168 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 196 ]), &(acadoWorkspace.Dy[ 49 ]), &(acadoWorkspace.QDy[ 28 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 224 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.QDy[ 32 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 252 ]), &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 280 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 308 ]), &(acadoWorkspace.Dy[ 77 ]), &(acadoWorkspace.QDy[ 44 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 336 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 364 ]), &(acadoWorkspace.Dy[ 91 ]), &(acadoWorkspace.QDy[ 52 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 392 ]), &(acadoWorkspace.Dy[ 98 ]), &(acadoWorkspace.QDy[ 56 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 420 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 448 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.QDy[ 64 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 476 ]), &(acadoWorkspace.Dy[ 119 ]), &(acadoWorkspace.QDy[ 68 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 504 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 532 ]), &(acadoWorkspace.Dy[ 133 ]), &(acadoWorkspace.QDy[ 76 ]) );

acadoWorkspace.QDy[80] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[81] = + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[82] = + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[83] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[3];

acadoWorkspace.QDy[4] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[5] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[6] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[7] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[8] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[9] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[10] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[22] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[23] += acadoWorkspace.Qd[19];
acadoWorkspace.QDy[24] += acadoWorkspace.Qd[20];
acadoWorkspace.QDy[25] += acadoWorkspace.Qd[21];
acadoWorkspace.QDy[26] += acadoWorkspace.Qd[22];
acadoWorkspace.QDy[27] += acadoWorkspace.Qd[23];
acadoWorkspace.QDy[28] += acadoWorkspace.Qd[24];
acadoWorkspace.QDy[29] += acadoWorkspace.Qd[25];
acadoWorkspace.QDy[30] += acadoWorkspace.Qd[26];
acadoWorkspace.QDy[31] += acadoWorkspace.Qd[27];
acadoWorkspace.QDy[32] += acadoWorkspace.Qd[28];
acadoWorkspace.QDy[33] += acadoWorkspace.Qd[29];
acadoWorkspace.QDy[34] += acadoWorkspace.Qd[30];
acadoWorkspace.QDy[35] += acadoWorkspace.Qd[31];
acadoWorkspace.QDy[36] += acadoWorkspace.Qd[32];
acadoWorkspace.QDy[37] += acadoWorkspace.Qd[33];
acadoWorkspace.QDy[38] += acadoWorkspace.Qd[34];
acadoWorkspace.QDy[39] += acadoWorkspace.Qd[35];
acadoWorkspace.QDy[40] += acadoWorkspace.Qd[36];
acadoWorkspace.QDy[41] += acadoWorkspace.Qd[37];
acadoWorkspace.QDy[42] += acadoWorkspace.Qd[38];
acadoWorkspace.QDy[43] += acadoWorkspace.Qd[39];
acadoWorkspace.QDy[44] += acadoWorkspace.Qd[40];
acadoWorkspace.QDy[45] += acadoWorkspace.Qd[41];
acadoWorkspace.QDy[46] += acadoWorkspace.Qd[42];
acadoWorkspace.QDy[47] += acadoWorkspace.Qd[43];
acadoWorkspace.QDy[48] += acadoWorkspace.Qd[44];
acadoWorkspace.QDy[49] += acadoWorkspace.Qd[45];
acadoWorkspace.QDy[50] += acadoWorkspace.Qd[46];
acadoWorkspace.QDy[51] += acadoWorkspace.Qd[47];
acadoWorkspace.QDy[52] += acadoWorkspace.Qd[48];
acadoWorkspace.QDy[53] += acadoWorkspace.Qd[49];
acadoWorkspace.QDy[54] += acadoWorkspace.Qd[50];
acadoWorkspace.QDy[55] += acadoWorkspace.Qd[51];
acadoWorkspace.QDy[56] += acadoWorkspace.Qd[52];
acadoWorkspace.QDy[57] += acadoWorkspace.Qd[53];
acadoWorkspace.QDy[58] += acadoWorkspace.Qd[54];
acadoWorkspace.QDy[59] += acadoWorkspace.Qd[55];
acadoWorkspace.QDy[60] += acadoWorkspace.Qd[56];
acadoWorkspace.QDy[61] += acadoWorkspace.Qd[57];
acadoWorkspace.QDy[62] += acadoWorkspace.Qd[58];
acadoWorkspace.QDy[63] += acadoWorkspace.Qd[59];
acadoWorkspace.QDy[64] += acadoWorkspace.Qd[60];
acadoWorkspace.QDy[65] += acadoWorkspace.Qd[61];
acadoWorkspace.QDy[66] += acadoWorkspace.Qd[62];
acadoWorkspace.QDy[67] += acadoWorkspace.Qd[63];
acadoWorkspace.QDy[68] += acadoWorkspace.Qd[64];
acadoWorkspace.QDy[69] += acadoWorkspace.Qd[65];
acadoWorkspace.QDy[70] += acadoWorkspace.Qd[66];
acadoWorkspace.QDy[71] += acadoWorkspace.Qd[67];
acadoWorkspace.QDy[72] += acadoWorkspace.Qd[68];
acadoWorkspace.QDy[73] += acadoWorkspace.Qd[69];
acadoWorkspace.QDy[74] += acadoWorkspace.Qd[70];
acadoWorkspace.QDy[75] += acadoWorkspace.Qd[71];
acadoWorkspace.QDy[76] += acadoWorkspace.Qd[72];
acadoWorkspace.QDy[77] += acadoWorkspace.Qd[73];
acadoWorkspace.QDy[78] += acadoWorkspace.Qd[74];
acadoWorkspace.QDy[79] += acadoWorkspace.Qd[75];
acadoWorkspace.QDy[80] += acadoWorkspace.Qd[76];
acadoWorkspace.QDy[81] += acadoWorkspace.Qd[77];
acadoWorkspace.QDy[82] += acadoWorkspace.Qd[78];
acadoWorkspace.QDy[83] += acadoWorkspace.Qd[79];

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 4 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QDy[ 28 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QDy[ 32 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QDy[ 44 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QDy[ 52 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QDy[ 56 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QDy[ 64 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QDy[ 68 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QDy[ 76 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QDy[ 80 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QDy[ 8 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 924 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1068 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1236 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1596 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1428 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1800 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1620 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1812 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2028 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2244 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2472 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2040 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2484 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2268 ]), &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2508 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 57 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[1] += + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[2] += + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[3] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[4] += + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[5] += + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[6] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[7] += + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[8] += + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[9] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[10] += + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[11] += + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[12] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[13] += + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[14] += + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[15] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[16] += + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[17] += + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[18] += + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[19] += + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[20] += + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[21] += + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[22] += + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[23] += + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[24] += + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[25] += + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[26] += + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[27] += + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[28] += + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[29] += + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[30] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[31] += + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[32] += + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[33] += + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[34] += + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[35] += + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[36] += + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[37] += + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[38] += + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[39] += + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[40] += + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[41] += + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[42] += + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[43] += + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[44] += + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[45] += + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[46] += + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[47] += + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[48] += + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[49] += + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[50] += + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[51] += + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[52] += + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[211]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[53] += + acadoWorkspace.H10[212]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[213]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[214]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[215]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[54] += + acadoWorkspace.H10[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[219]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[55] += + acadoWorkspace.H10[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[223]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[56] += + acadoWorkspace.H10[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[227]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[57] += + acadoWorkspace.H10[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[231]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[58] += + acadoWorkspace.H10[232]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[233]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[234]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[235]*acadoWorkspace.Dx0[3];
acadoWorkspace.g[59] += + acadoWorkspace.H10[236]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[237]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[238]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[239]*acadoWorkspace.Dx0[3];

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.lb[50] = acadoVariables.lbValues[50] - acadoVariables.u[50];
acadoWorkspace.lb[51] = acadoVariables.lbValues[51] - acadoVariables.u[51];
acadoWorkspace.lb[52] = acadoVariables.lbValues[52] - acadoVariables.u[52];
acadoWorkspace.lb[53] = acadoVariables.lbValues[53] - acadoVariables.u[53];
acadoWorkspace.lb[54] = acadoVariables.lbValues[54] - acadoVariables.u[54];
acadoWorkspace.lb[55] = acadoVariables.lbValues[55] - acadoVariables.u[55];
acadoWorkspace.lb[56] = acadoVariables.lbValues[56] - acadoVariables.u[56];
acadoWorkspace.lb[57] = acadoVariables.lbValues[57] - acadoVariables.u[57];
acadoWorkspace.lb[58] = acadoVariables.lbValues[58] - acadoVariables.u[58];
acadoWorkspace.lb[59] = acadoVariables.lbValues[59] - acadoVariables.u[59];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[50] = acadoVariables.ubValues[50] - acadoVariables.u[50];
acadoWorkspace.ub[51] = acadoVariables.ubValues[51] - acadoVariables.u[51];
acadoWorkspace.ub[52] = acadoVariables.ubValues[52] - acadoVariables.u[52];
acadoWorkspace.ub[53] = acadoVariables.ubValues[53] - acadoVariables.u[53];
acadoWorkspace.ub[54] = acadoVariables.ubValues[54] - acadoVariables.u[54];
acadoWorkspace.ub[55] = acadoVariables.ubValues[55] - acadoVariables.u[55];
acadoWorkspace.ub[56] = acadoVariables.ubValues[56] - acadoVariables.u[56];
acadoWorkspace.ub[57] = acadoVariables.ubValues[57] - acadoVariables.u[57];
acadoWorkspace.ub[58] = acadoVariables.ubValues[58] - acadoVariables.u[58];
acadoWorkspace.ub[59] = acadoVariables.ubValues[59] - acadoVariables.u[59];

tmp = + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[3] + acadoVariables.x[7];
tmp += acadoWorkspace.d[3];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - tmp;
acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - tmp;
tmp = + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[3] + acadoVariables.x[11];
tmp += acadoWorkspace.d[7];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - tmp;
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - tmp;
tmp = + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[3] + acadoVariables.x[15];
tmp += acadoWorkspace.d[11];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - tmp;
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - tmp;
tmp = + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoVariables.x[19];
tmp += acadoWorkspace.d[15];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - tmp;
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - tmp;
tmp = + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[3] + acadoVariables.x[23];
tmp += acadoWorkspace.d[19];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - tmp;
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - tmp;
tmp = + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[3] + acadoVariables.x[27];
tmp += acadoWorkspace.d[23];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - tmp;
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - tmp;
tmp = + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[3] + acadoVariables.x[31];
tmp += acadoWorkspace.d[27];
acadoWorkspace.lbA[6] = acadoVariables.lbAValues[6] - tmp;
acadoWorkspace.ubA[6] = acadoVariables.ubAValues[6] - tmp;
tmp = + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[3] + acadoVariables.x[35];
tmp += acadoWorkspace.d[31];
acadoWorkspace.lbA[7] = acadoVariables.lbAValues[7] - tmp;
acadoWorkspace.ubA[7] = acadoVariables.ubAValues[7] - tmp;
tmp = + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[3] + acadoVariables.x[39];
tmp += acadoWorkspace.d[35];
acadoWorkspace.lbA[8] = acadoVariables.lbAValues[8] - tmp;
acadoWorkspace.ubA[8] = acadoVariables.ubAValues[8] - tmp;
tmp = + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[3] + acadoVariables.x[43];
tmp += acadoWorkspace.d[39];
acadoWorkspace.lbA[9] = acadoVariables.lbAValues[9] - tmp;
acadoWorkspace.ubA[9] = acadoVariables.ubAValues[9] - tmp;
tmp = + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[3] + acadoVariables.x[47];
tmp += acadoWorkspace.d[43];
acadoWorkspace.lbA[10] = acadoVariables.lbAValues[10] - tmp;
acadoWorkspace.ubA[10] = acadoVariables.ubAValues[10] - tmp;
tmp = + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[3] + acadoVariables.x[51];
tmp += acadoWorkspace.d[47];
acadoWorkspace.lbA[11] = acadoVariables.lbAValues[11] - tmp;
acadoWorkspace.ubA[11] = acadoVariables.ubAValues[11] - tmp;
tmp = + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[3] + acadoVariables.x[55];
tmp += acadoWorkspace.d[51];
acadoWorkspace.lbA[12] = acadoVariables.lbAValues[12] - tmp;
acadoWorkspace.ubA[12] = acadoVariables.ubAValues[12] - tmp;
tmp = + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3] + acadoVariables.x[59];
tmp += acadoWorkspace.d[55];
acadoWorkspace.lbA[13] = acadoVariables.lbAValues[13] - tmp;
acadoWorkspace.ubA[13] = acadoVariables.ubAValues[13] - tmp;
tmp = + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[3] + acadoVariables.x[63];
tmp += acadoWorkspace.d[59];
acadoWorkspace.lbA[14] = acadoVariables.lbAValues[14] - tmp;
acadoWorkspace.ubA[14] = acadoVariables.ubAValues[14] - tmp;
tmp = + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[3] + acadoVariables.x[67];
tmp += acadoWorkspace.d[63];
acadoWorkspace.lbA[15] = acadoVariables.lbAValues[15] - tmp;
acadoWorkspace.ubA[15] = acadoVariables.ubAValues[15] - tmp;
tmp = + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[3] + acadoVariables.x[71];
tmp += acadoWorkspace.d[67];
acadoWorkspace.lbA[16] = acadoVariables.lbAValues[16] - tmp;
acadoWorkspace.ubA[16] = acadoVariables.ubAValues[16] - tmp;
tmp = + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[3] + acadoVariables.x[75];
tmp += acadoWorkspace.d[71];
acadoWorkspace.lbA[17] = acadoVariables.lbAValues[17] - tmp;
acadoWorkspace.ubA[17] = acadoVariables.ubAValues[17] - tmp;
tmp = + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3] + acadoVariables.x[79];
tmp += acadoWorkspace.d[75];
acadoWorkspace.lbA[18] = acadoVariables.lbAValues[18] - tmp;
acadoWorkspace.ubA[18] = acadoVariables.ubAValues[18] - tmp;
tmp = + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[3] + acadoVariables.x[83];
tmp += acadoWorkspace.d[79];
acadoWorkspace.lbA[19] = acadoVariables.lbAValues[19] - tmp;
acadoWorkspace.ubA[19] = acadoVariables.ubAValues[19] - tmp;

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];

acadoVariables.x[4] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[0];
acadoVariables.x[5] += + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[1];
acadoVariables.x[6] += + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[2];
acadoVariables.x[7] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[3];
acadoVariables.x[8] += + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[4];
acadoVariables.x[9] += + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[5];
acadoVariables.x[10] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[6];
acadoVariables.x[11] += + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[7];
acadoVariables.x[12] += + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[8];
acadoVariables.x[13] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[9];
acadoVariables.x[14] += + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[10];
acadoVariables.x[15] += + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[11];
acadoVariables.x[16] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[12];
acadoVariables.x[17] += + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[13];
acadoVariables.x[18] += + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[14];
acadoVariables.x[19] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[15];
acadoVariables.x[20] += + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[16];
acadoVariables.x[21] += + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[17];
acadoVariables.x[22] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[18];
acadoVariables.x[23] += + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[19];
acadoVariables.x[24] += + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[20];
acadoVariables.x[25] += + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[21];
acadoVariables.x[26] += + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[22];
acadoVariables.x[27] += + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[23];
acadoVariables.x[28] += + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[24];
acadoVariables.x[29] += + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[25];
acadoVariables.x[30] += + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[26];
acadoVariables.x[31] += + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[27];
acadoVariables.x[32] += + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[28];
acadoVariables.x[33] += + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[29];
acadoVariables.x[34] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[30];
acadoVariables.x[35] += + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[31];
acadoVariables.x[36] += + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[32];
acadoVariables.x[37] += + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[33];
acadoVariables.x[38] += + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[34];
acadoVariables.x[39] += + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[35];
acadoVariables.x[40] += + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[36];
acadoVariables.x[41] += + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[37];
acadoVariables.x[42] += + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[38];
acadoVariables.x[43] += + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[39];
acadoVariables.x[44] += + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[40];
acadoVariables.x[45] += + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[41];
acadoVariables.x[46] += + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[42];
acadoVariables.x[47] += + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[43];
acadoVariables.x[48] += + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[44];
acadoVariables.x[49] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[45];
acadoVariables.x[50] += + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[46];
acadoVariables.x[51] += + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[47];
acadoVariables.x[52] += + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[48];
acadoVariables.x[53] += + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[49];
acadoVariables.x[54] += + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[50];
acadoVariables.x[55] += + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[51];
acadoVariables.x[56] += + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[52];
acadoVariables.x[57] += + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[53];
acadoVariables.x[58] += + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[54];
acadoVariables.x[59] += + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[55];
acadoVariables.x[60] += + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[56];
acadoVariables.x[61] += + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[57];
acadoVariables.x[62] += + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[58];
acadoVariables.x[63] += + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[59];
acadoVariables.x[64] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[60];
acadoVariables.x[65] += + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[61];
acadoVariables.x[66] += + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[62];
acadoVariables.x[67] += + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[63];
acadoVariables.x[68] += + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[64];
acadoVariables.x[69] += + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[65];
acadoVariables.x[70] += + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[66];
acadoVariables.x[71] += + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[67];
acadoVariables.x[72] += + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[68];
acadoVariables.x[73] += + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[69];
acadoVariables.x[74] += + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[70];
acadoVariables.x[75] += + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[71];
acadoVariables.x[76] += + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[72];
acadoVariables.x[77] += + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[73];
acadoVariables.x[78] += + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[74];
acadoVariables.x[79] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[75];
acadoVariables.x[80] += + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[76];
acadoVariables.x[81] += + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[77];
acadoVariables.x[82] += + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[78];
acadoVariables.x[83] += + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[3] + acadoWorkspace.d[79];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 4 ]) );
acado_multEDu( &(acadoWorkspace.E[ 12 ]), acadoWorkspace.x, &(acadoVariables.x[ 8 ]) );
acado_multEDu( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 8 ]) );
acado_multEDu( &(acadoWorkspace.E[ 36 ]), acadoWorkspace.x, &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 72 ]), acadoWorkspace.x, &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 120 ]), acadoWorkspace.x, &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 180 ]), acadoWorkspace.x, &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 252 ]), acadoWorkspace.x, &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 28 ]) );
acado_multEDu( &(acadoWorkspace.E[ 336 ]), acadoWorkspace.x, &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 432 ]), acadoWorkspace.x, &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 540 ]), acadoWorkspace.x, &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.x[ 27 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 660 ]), acadoWorkspace.x, &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.x[ 27 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 44 ]) );
acado_multEDu( &(acadoWorkspace.E[ 792 ]), acadoWorkspace.x, &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.x[ 27 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 924 ]), &(acadoWorkspace.x[ 33 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 936 ]), acadoWorkspace.x, &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.x[ 27 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1068 ]), &(acadoWorkspace.x[ 33 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 52 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1092 ]), acadoWorkspace.x, &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.x[ 27 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.x[ 33 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1236 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.x[ 39 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1260 ]), acadoWorkspace.x, &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.x[ 27 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.x[ 33 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.x[ 39 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1428 ]), &(acadoWorkspace.x[ 42 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1440 ]), acadoWorkspace.x, &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.x[ 27 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.x[ 33 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1596 ]), &(acadoWorkspace.x[ 39 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.x[ 42 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1620 ]), &(acadoWorkspace.x[ 45 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1632 ]), acadoWorkspace.x, &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.x[ 27 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.x[ 33 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.x[ 39 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1800 ]), &(acadoWorkspace.x[ 42 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1812 ]), &(acadoWorkspace.x[ 45 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.x[ 48 ]), &(acadoVariables.x[ 68 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1836 ]), acadoWorkspace.x, &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.x[ 27 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.x[ 33 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.x[ 39 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.x[ 42 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.x[ 45 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2028 ]), &(acadoWorkspace.x[ 48 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2040 ]), &(acadoWorkspace.x[ 51 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2052 ]), acadoWorkspace.x, &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.x[ 27 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.x[ 33 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.x[ 39 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.x[ 42 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.x[ 45 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2244 ]), &(acadoWorkspace.x[ 48 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.x[ 51 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2268 ]), &(acadoWorkspace.x[ 54 ]), &(acadoVariables.x[ 76 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2280 ]), acadoWorkspace.x, &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.x[ 27 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.x[ 33 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.x[ 39 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.x[ 42 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.x[ 45 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2472 ]), &(acadoWorkspace.x[ 48 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2484 ]), &(acadoWorkspace.x[ 51 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.x[ 54 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2508 ]), &(acadoWorkspace.x[ 57 ]), &(acadoVariables.x[ 80 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -1.0000000000000000e+00;
acadoVariables.lbValues[1] = 0.0000000000000000e+00;
acadoVariables.lbValues[2] = -2.6179938779914941e-01;
acadoVariables.lbValues[3] = -1.0000000000000000e+00;
acadoVariables.lbValues[4] = 0.0000000000000000e+00;
acadoVariables.lbValues[5] = -2.6179938779914941e-01;
acadoVariables.lbValues[6] = -1.0000000000000000e+00;
acadoVariables.lbValues[7] = 0.0000000000000000e+00;
acadoVariables.lbValues[8] = -2.6179938779914941e-01;
acadoVariables.lbValues[9] = -1.0000000000000000e+00;
acadoVariables.lbValues[10] = 0.0000000000000000e+00;
acadoVariables.lbValues[11] = -2.6179938779914941e-01;
acadoVariables.lbValues[12] = -1.0000000000000000e+00;
acadoVariables.lbValues[13] = 0.0000000000000000e+00;
acadoVariables.lbValues[14] = -2.6179938779914941e-01;
acadoVariables.lbValues[15] = -1.0000000000000000e+00;
acadoVariables.lbValues[16] = 0.0000000000000000e+00;
acadoVariables.lbValues[17] = -2.6179938779914941e-01;
acadoVariables.lbValues[18] = -1.0000000000000000e+00;
acadoVariables.lbValues[19] = 0.0000000000000000e+00;
acadoVariables.lbValues[20] = -2.6179938779914941e-01;
acadoVariables.lbValues[21] = -1.0000000000000000e+00;
acadoVariables.lbValues[22] = 0.0000000000000000e+00;
acadoVariables.lbValues[23] = -2.6179938779914941e-01;
acadoVariables.lbValues[24] = -1.0000000000000000e+00;
acadoVariables.lbValues[25] = 0.0000000000000000e+00;
acadoVariables.lbValues[26] = -2.6179938779914941e-01;
acadoVariables.lbValues[27] = -1.0000000000000000e+00;
acadoVariables.lbValues[28] = 0.0000000000000000e+00;
acadoVariables.lbValues[29] = -2.6179938779914941e-01;
acadoVariables.lbValues[30] = -1.0000000000000000e+00;
acadoVariables.lbValues[31] = 0.0000000000000000e+00;
acadoVariables.lbValues[32] = -2.6179938779914941e-01;
acadoVariables.lbValues[33] = -1.0000000000000000e+00;
acadoVariables.lbValues[34] = 0.0000000000000000e+00;
acadoVariables.lbValues[35] = -2.6179938779914941e-01;
acadoVariables.lbValues[36] = -1.0000000000000000e+00;
acadoVariables.lbValues[37] = 0.0000000000000000e+00;
acadoVariables.lbValues[38] = -2.6179938779914941e-01;
acadoVariables.lbValues[39] = -1.0000000000000000e+00;
acadoVariables.lbValues[40] = 0.0000000000000000e+00;
acadoVariables.lbValues[41] = -2.6179938779914941e-01;
acadoVariables.lbValues[42] = -1.0000000000000000e+00;
acadoVariables.lbValues[43] = 0.0000000000000000e+00;
acadoVariables.lbValues[44] = -2.6179938779914941e-01;
acadoVariables.lbValues[45] = -1.0000000000000000e+00;
acadoVariables.lbValues[46] = 0.0000000000000000e+00;
acadoVariables.lbValues[47] = -2.6179938779914941e-01;
acadoVariables.lbValues[48] = -1.0000000000000000e+00;
acadoVariables.lbValues[49] = 0.0000000000000000e+00;
acadoVariables.lbValues[50] = -2.6179938779914941e-01;
acadoVariables.lbValues[51] = -1.0000000000000000e+00;
acadoVariables.lbValues[52] = 0.0000000000000000e+00;
acadoVariables.lbValues[53] = -2.6179938779914941e-01;
acadoVariables.lbValues[54] = -1.0000000000000000e+00;
acadoVariables.lbValues[55] = 0.0000000000000000e+00;
acadoVariables.lbValues[56] = -2.6179938779914941e-01;
acadoVariables.lbValues[57] = -1.0000000000000000e+00;
acadoVariables.lbValues[58] = 0.0000000000000000e+00;
acadoVariables.lbValues[59] = -2.6179938779914941e-01;
acadoVariables.ubValues[0] = 1.0000000000000000e+00;
acadoVariables.ubValues[1] = 3.0000000000000000e+01;
acadoVariables.ubValues[2] = 4.3633231299858238e-01;
acadoVariables.ubValues[3] = 1.0000000000000000e+00;
acadoVariables.ubValues[4] = 3.0000000000000000e+01;
acadoVariables.ubValues[5] = 4.3633231299858238e-01;
acadoVariables.ubValues[6] = 1.0000000000000000e+00;
acadoVariables.ubValues[7] = 3.0000000000000000e+01;
acadoVariables.ubValues[8] = 4.3633231299858238e-01;
acadoVariables.ubValues[9] = 1.0000000000000000e+00;
acadoVariables.ubValues[10] = 3.0000000000000000e+01;
acadoVariables.ubValues[11] = 4.3633231299858238e-01;
acadoVariables.ubValues[12] = 1.0000000000000000e+00;
acadoVariables.ubValues[13] = 3.0000000000000000e+01;
acadoVariables.ubValues[14] = 4.3633231299858238e-01;
acadoVariables.ubValues[15] = 1.0000000000000000e+00;
acadoVariables.ubValues[16] = 3.0000000000000000e+01;
acadoVariables.ubValues[17] = 4.3633231299858238e-01;
acadoVariables.ubValues[18] = 1.0000000000000000e+00;
acadoVariables.ubValues[19] = 3.0000000000000000e+01;
acadoVariables.ubValues[20] = 4.3633231299858238e-01;
acadoVariables.ubValues[21] = 1.0000000000000000e+00;
acadoVariables.ubValues[22] = 3.0000000000000000e+01;
acadoVariables.ubValues[23] = 4.3633231299858238e-01;
acadoVariables.ubValues[24] = 1.0000000000000000e+00;
acadoVariables.ubValues[25] = 3.0000000000000000e+01;
acadoVariables.ubValues[26] = 4.3633231299858238e-01;
acadoVariables.ubValues[27] = 1.0000000000000000e+00;
acadoVariables.ubValues[28] = 3.0000000000000000e+01;
acadoVariables.ubValues[29] = 4.3633231299858238e-01;
acadoVariables.ubValues[30] = 1.0000000000000000e+00;
acadoVariables.ubValues[31] = 3.0000000000000000e+01;
acadoVariables.ubValues[32] = 4.3633231299858238e-01;
acadoVariables.ubValues[33] = 1.0000000000000000e+00;
acadoVariables.ubValues[34] = 3.0000000000000000e+01;
acadoVariables.ubValues[35] = 4.3633231299858238e-01;
acadoVariables.ubValues[36] = 1.0000000000000000e+00;
acadoVariables.ubValues[37] = 3.0000000000000000e+01;
acadoVariables.ubValues[38] = 4.3633231299858238e-01;
acadoVariables.ubValues[39] = 1.0000000000000000e+00;
acadoVariables.ubValues[40] = 3.0000000000000000e+01;
acadoVariables.ubValues[41] = 4.3633231299858238e-01;
acadoVariables.ubValues[42] = 1.0000000000000000e+00;
acadoVariables.ubValues[43] = 3.0000000000000000e+01;
acadoVariables.ubValues[44] = 4.3633231299858238e-01;
acadoVariables.ubValues[45] = 1.0000000000000000e+00;
acadoVariables.ubValues[46] = 3.0000000000000000e+01;
acadoVariables.ubValues[47] = 4.3633231299858238e-01;
acadoVariables.ubValues[48] = 1.0000000000000000e+00;
acadoVariables.ubValues[49] = 3.0000000000000000e+01;
acadoVariables.ubValues[50] = 4.3633231299858238e-01;
acadoVariables.ubValues[51] = 1.0000000000000000e+00;
acadoVariables.ubValues[52] = 3.0000000000000000e+01;
acadoVariables.ubValues[53] = 4.3633231299858238e-01;
acadoVariables.ubValues[54] = 1.0000000000000000e+00;
acadoVariables.ubValues[55] = 3.0000000000000000e+01;
acadoVariables.ubValues[56] = 4.3633231299858238e-01;
acadoVariables.ubValues[57] = 1.0000000000000000e+00;
acadoVariables.ubValues[58] = 3.0000000000000000e+01;
acadoVariables.ubValues[59] = 4.3633231299858238e-01;
{ int lCopy; for (lCopy = 0; lCopy < 20; lCopy++) acadoVariables.lbAValues[ lCopy ] = 0; }
acadoVariables.ubAValues[0] = 1.5707963267948966e+00;
acadoVariables.ubAValues[1] = 1.5707963267948966e+00;
acadoVariables.ubAValues[2] = 1.5707963267948966e+00;
acadoVariables.ubAValues[3] = 1.5707963267948966e+00;
acadoVariables.ubAValues[4] = 1.5707963267948966e+00;
acadoVariables.ubAValues[5] = 1.5707963267948966e+00;
acadoVariables.ubAValues[6] = 1.5707963267948966e+00;
acadoVariables.ubAValues[7] = 1.5707963267948966e+00;
acadoVariables.ubAValues[8] = 1.5707963267948966e+00;
acadoVariables.ubAValues[9] = 1.5707963267948966e+00;
acadoVariables.ubAValues[10] = 1.5707963267948966e+00;
acadoVariables.ubAValues[11] = 1.5707963267948966e+00;
acadoVariables.ubAValues[12] = 1.5707963267948966e+00;
acadoVariables.ubAValues[13] = 1.5707963267948966e+00;
acadoVariables.ubAValues[14] = 1.5707963267948966e+00;
acadoVariables.ubAValues[15] = 1.5707963267948966e+00;
acadoVariables.ubAValues[16] = 1.5707963267948966e+00;
acadoVariables.ubAValues[17] = 1.5707963267948966e+00;
acadoVariables.ubAValues[18] = 1.5707963267948966e+00;
acadoVariables.ubAValues[19] = 1.5707963267948966e+00;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 4];
acadoWorkspace.state[1] = acadoVariables.x[index * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 4 + 3];
acadoWorkspace.state[32] = acadoVariables.u[index * 3];
acadoWorkspace.state[33] = acadoVariables.u[index * 3 + 1];
acadoWorkspace.state[34] = acadoVariables.u[index * 3 + 2];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 4 + 4] = acadoWorkspace.state[0];
acadoVariables.x[index * 4 + 5] = acadoWorkspace.state[1];
acadoVariables.x[index * 4 + 6] = acadoWorkspace.state[2];
acadoVariables.x[index * 4 + 7] = acadoWorkspace.state[3];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.x[index * 4] = acadoVariables.x[index * 4 + 4];
acadoVariables.x[index * 4 + 1] = acadoVariables.x[index * 4 + 5];
acadoVariables.x[index * 4 + 2] = acadoVariables.x[index * 4 + 6];
acadoVariables.x[index * 4 + 3] = acadoVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[80] = xEnd[0];
acadoVariables.x[81] = xEnd[1];
acadoVariables.x[82] = xEnd[2];
acadoVariables.x[83] = xEnd[3];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[80];
acadoWorkspace.state[1] = acadoVariables.x[81];
acadoWorkspace.state[2] = acadoVariables.x[82];
acadoWorkspace.state[3] = acadoVariables.x[83];
if (uEnd != 0)
{
acadoWorkspace.state[32] = uEnd[0];
acadoWorkspace.state[33] = uEnd[1];
acadoWorkspace.state[34] = uEnd[2];
}
else
{
acadoWorkspace.state[32] = acadoVariables.u[57];
acadoWorkspace.state[33] = acadoVariables.u[58];
acadoWorkspace.state[34] = acadoVariables.u[59];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[80] = acadoWorkspace.state[0];
acadoVariables.x[81] = acadoWorkspace.state[1];
acadoVariables.x[82] = acadoWorkspace.state[2];
acadoVariables.x[83] = acadoWorkspace.state[3];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 19; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[57] = uEnd[0];
acadoVariables.u[58] = uEnd[1];
acadoVariables.u[59] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59];
kkt = fabs( kkt );
for (index = 0; index < 60; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index + 60];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 7 */
real_t tmpDy[ 7 ];

/** Row vector of size: 4 */
real_t tmpDyN[ 4 ];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 7] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 7];
acadoWorkspace.Dy[lRun1 * 7 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 7 + 1];
acadoWorkspace.Dy[lRun1 * 7 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 7 + 2];
acadoWorkspace.Dy[lRun1 * 7 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 7 + 3];
acadoWorkspace.Dy[lRun1 * 7 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 7 + 4];
acadoWorkspace.Dy[lRun1 * 7 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 7 + 5];
acadoWorkspace.Dy[lRun1 * 7 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 7 + 6];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[80];
acadoWorkspace.objValueIn[1] = acadoVariables.x[81];
acadoWorkspace.objValueIn[2] = acadoVariables.x[82];
acadoWorkspace.objValueIn[3] = acadoVariables.x[83];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 7] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 14] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 21] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 28] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 35] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 42];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 1] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 8] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 15] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 22] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 29] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 36] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 43];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 2] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 9] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 16] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 23] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 30] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 37] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 44];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 3] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 10] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 17] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 24] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 31] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 38] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 45];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 4] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 11] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 18] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 25] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 32] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 39] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 46];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 5] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 12] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 19] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 26] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 33] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 40] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 47];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 7]*acadoVariables.W[lRun1 * 49 + 6] + acadoWorkspace.Dy[lRun1 * 7 + 1]*acadoVariables.W[lRun1 * 49 + 13] + acadoWorkspace.Dy[lRun1 * 7 + 2]*acadoVariables.W[lRun1 * 49 + 20] + acadoWorkspace.Dy[lRun1 * 7 + 3]*acadoVariables.W[lRun1 * 49 + 27] + acadoWorkspace.Dy[lRun1 * 7 + 4]*acadoVariables.W[lRun1 * 49 + 34] + acadoWorkspace.Dy[lRun1 * 7 + 5]*acadoVariables.W[lRun1 * 49 + 41] + acadoWorkspace.Dy[lRun1 * 7 + 6]*acadoVariables.W[lRun1 * 49 + 48];
objVal += + acadoWorkspace.Dy[lRun1 * 7]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 7 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 7 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 7 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 7 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 7 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 7 + 6]*tmpDy[6];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[5];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[10];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[15];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3];

objVal *= 0.5;
return objVal;
}

