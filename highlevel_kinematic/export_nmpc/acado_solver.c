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
for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 7];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 7 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 7 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 7 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 7 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 7 + 5];
acadoWorkspace.state[6] = acadoVariables.x[lRun1 * 7 + 6];

acadoWorkspace.state[77] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.state[78] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.state[79] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.state[80] = acadoVariables.od[lRun1 * 9];
acadoWorkspace.state[81] = acadoVariables.od[lRun1 * 9 + 1];
acadoWorkspace.state[82] = acadoVariables.od[lRun1 * 9 + 2];
acadoWorkspace.state[83] = acadoVariables.od[lRun1 * 9 + 3];
acadoWorkspace.state[84] = acadoVariables.od[lRun1 * 9 + 4];
acadoWorkspace.state[85] = acadoVariables.od[lRun1 * 9 + 5];
acadoWorkspace.state[86] = acadoVariables.od[lRun1 * 9 + 6];
acadoWorkspace.state[87] = acadoVariables.od[lRun1 * 9 + 7];
acadoWorkspace.state[88] = acadoVariables.od[lRun1 * 9 + 8];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 7] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 7 + 7];
acadoWorkspace.d[lRun1 * 7 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 7 + 8];
acadoWorkspace.d[lRun1 * 7 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 7 + 9];
acadoWorkspace.d[lRun1 * 7 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 7 + 10];
acadoWorkspace.d[lRun1 * 7 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 7 + 11];
acadoWorkspace.d[lRun1 * 7 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 7 + 12];
acadoWorkspace.d[lRun1 * 7 + 6] = acadoWorkspace.state[6] - acadoVariables.x[lRun1 * 7 + 13];

acadoWorkspace.evGx[lRun1 * 49] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 49 + 1] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 49 + 2] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 49 + 3] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 49 + 4] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 49 + 5] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 49 + 6] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 49 + 7] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 49 + 8] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 49 + 9] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 49 + 10] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 49 + 11] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 49 + 12] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 49 + 13] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 49 + 14] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 49 + 15] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 49 + 16] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 49 + 17] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 49 + 18] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 49 + 19] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 49 + 20] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 49 + 21] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 49 + 22] = acadoWorkspace.state[29];
acadoWorkspace.evGx[lRun1 * 49 + 23] = acadoWorkspace.state[30];
acadoWorkspace.evGx[lRun1 * 49 + 24] = acadoWorkspace.state[31];
acadoWorkspace.evGx[lRun1 * 49 + 25] = acadoWorkspace.state[32];
acadoWorkspace.evGx[lRun1 * 49 + 26] = acadoWorkspace.state[33];
acadoWorkspace.evGx[lRun1 * 49 + 27] = acadoWorkspace.state[34];
acadoWorkspace.evGx[lRun1 * 49 + 28] = acadoWorkspace.state[35];
acadoWorkspace.evGx[lRun1 * 49 + 29] = acadoWorkspace.state[36];
acadoWorkspace.evGx[lRun1 * 49 + 30] = acadoWorkspace.state[37];
acadoWorkspace.evGx[lRun1 * 49 + 31] = acadoWorkspace.state[38];
acadoWorkspace.evGx[lRun1 * 49 + 32] = acadoWorkspace.state[39];
acadoWorkspace.evGx[lRun1 * 49 + 33] = acadoWorkspace.state[40];
acadoWorkspace.evGx[lRun1 * 49 + 34] = acadoWorkspace.state[41];
acadoWorkspace.evGx[lRun1 * 49 + 35] = acadoWorkspace.state[42];
acadoWorkspace.evGx[lRun1 * 49 + 36] = acadoWorkspace.state[43];
acadoWorkspace.evGx[lRun1 * 49 + 37] = acadoWorkspace.state[44];
acadoWorkspace.evGx[lRun1 * 49 + 38] = acadoWorkspace.state[45];
acadoWorkspace.evGx[lRun1 * 49 + 39] = acadoWorkspace.state[46];
acadoWorkspace.evGx[lRun1 * 49 + 40] = acadoWorkspace.state[47];
acadoWorkspace.evGx[lRun1 * 49 + 41] = acadoWorkspace.state[48];
acadoWorkspace.evGx[lRun1 * 49 + 42] = acadoWorkspace.state[49];
acadoWorkspace.evGx[lRun1 * 49 + 43] = acadoWorkspace.state[50];
acadoWorkspace.evGx[lRun1 * 49 + 44] = acadoWorkspace.state[51];
acadoWorkspace.evGx[lRun1 * 49 + 45] = acadoWorkspace.state[52];
acadoWorkspace.evGx[lRun1 * 49 + 46] = acadoWorkspace.state[53];
acadoWorkspace.evGx[lRun1 * 49 + 47] = acadoWorkspace.state[54];
acadoWorkspace.evGx[lRun1 * 49 + 48] = acadoWorkspace.state[55];

acadoWorkspace.evGu[lRun1 * 21] = acadoWorkspace.state[56];
acadoWorkspace.evGu[lRun1 * 21 + 1] = acadoWorkspace.state[57];
acadoWorkspace.evGu[lRun1 * 21 + 2] = acadoWorkspace.state[58];
acadoWorkspace.evGu[lRun1 * 21 + 3] = acadoWorkspace.state[59];
acadoWorkspace.evGu[lRun1 * 21 + 4] = acadoWorkspace.state[60];
acadoWorkspace.evGu[lRun1 * 21 + 5] = acadoWorkspace.state[61];
acadoWorkspace.evGu[lRun1 * 21 + 6] = acadoWorkspace.state[62];
acadoWorkspace.evGu[lRun1 * 21 + 7] = acadoWorkspace.state[63];
acadoWorkspace.evGu[lRun1 * 21 + 8] = acadoWorkspace.state[64];
acadoWorkspace.evGu[lRun1 * 21 + 9] = acadoWorkspace.state[65];
acadoWorkspace.evGu[lRun1 * 21 + 10] = acadoWorkspace.state[66];
acadoWorkspace.evGu[lRun1 * 21 + 11] = acadoWorkspace.state[67];
acadoWorkspace.evGu[lRun1 * 21 + 12] = acadoWorkspace.state[68];
acadoWorkspace.evGu[lRun1 * 21 + 13] = acadoWorkspace.state[69];
acadoWorkspace.evGu[lRun1 * 21 + 14] = acadoWorkspace.state[70];
acadoWorkspace.evGu[lRun1 * 21 + 15] = acadoWorkspace.state[71];
acadoWorkspace.evGu[lRun1 * 21 + 16] = acadoWorkspace.state[72];
acadoWorkspace.evGu[lRun1 * 21 + 17] = acadoWorkspace.state[73];
acadoWorkspace.evGu[lRun1 * 21 + 18] = acadoWorkspace.state[74];
acadoWorkspace.evGu[lRun1 * 21 + 19] = acadoWorkspace.state[75];
acadoWorkspace.evGu[lRun1 * 21 + 20] = acadoWorkspace.state[76];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 7;
const real_t* od = in + 10;

/* Compute outputs: */
out[0] = (((((real_t)(0.0000000000000000e+00)-xd[1])+od[4])*od[6])-((od[3]-xd[0])*od[7]));
out[1] = ((((real_t)(0.0000000000000000e+00)-xd[2])+od[5])*od[8]);
out[2] = u[0];
out[3] = u[1];
out[4] = u[2];
out[5] = ((real_t)(0.0000000000000000e+00)-(((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*od[7]));
out[6] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*od[6]);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*od[8]);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 7;

/* Compute outputs: */
out[0] = (((((real_t)(0.0000000000000000e+00)-xd[1])+od[4])*od[6])-((od[3]-xd[0])*od[7]));
out[1] = ((((real_t)(0.0000000000000000e+00)-xd[2])+od[5])*od[8]);
out[2] = ((real_t)(0.0000000000000000e+00)-(((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*od[7]));
out[3] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*od[6]);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*od[8]);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
}

void acado_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[7]*tmpObjS[5] + tmpFx[14]*tmpObjS[10] + tmpFx[21]*tmpObjS[15] + tmpFx[28]*tmpObjS[20];
tmpQ2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[7]*tmpObjS[6] + tmpFx[14]*tmpObjS[11] + tmpFx[21]*tmpObjS[16] + tmpFx[28]*tmpObjS[21];
tmpQ2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[7]*tmpObjS[7] + tmpFx[14]*tmpObjS[12] + tmpFx[21]*tmpObjS[17] + tmpFx[28]*tmpObjS[22];
tmpQ2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[7]*tmpObjS[8] + tmpFx[14]*tmpObjS[13] + tmpFx[21]*tmpObjS[18] + tmpFx[28]*tmpObjS[23];
tmpQ2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[7]*tmpObjS[9] + tmpFx[14]*tmpObjS[14] + tmpFx[21]*tmpObjS[19] + tmpFx[28]*tmpObjS[24];
tmpQ2[5] = + tmpFx[1]*tmpObjS[0] + tmpFx[8]*tmpObjS[5] + tmpFx[15]*tmpObjS[10] + tmpFx[22]*tmpObjS[15] + tmpFx[29]*tmpObjS[20];
tmpQ2[6] = + tmpFx[1]*tmpObjS[1] + tmpFx[8]*tmpObjS[6] + tmpFx[15]*tmpObjS[11] + tmpFx[22]*tmpObjS[16] + tmpFx[29]*tmpObjS[21];
tmpQ2[7] = + tmpFx[1]*tmpObjS[2] + tmpFx[8]*tmpObjS[7] + tmpFx[15]*tmpObjS[12] + tmpFx[22]*tmpObjS[17] + tmpFx[29]*tmpObjS[22];
tmpQ2[8] = + tmpFx[1]*tmpObjS[3] + tmpFx[8]*tmpObjS[8] + tmpFx[15]*tmpObjS[13] + tmpFx[22]*tmpObjS[18] + tmpFx[29]*tmpObjS[23];
tmpQ2[9] = + tmpFx[1]*tmpObjS[4] + tmpFx[8]*tmpObjS[9] + tmpFx[15]*tmpObjS[14] + tmpFx[22]*tmpObjS[19] + tmpFx[29]*tmpObjS[24];
tmpQ2[10] = + tmpFx[2]*tmpObjS[0] + tmpFx[9]*tmpObjS[5] + tmpFx[16]*tmpObjS[10] + tmpFx[23]*tmpObjS[15] + tmpFx[30]*tmpObjS[20];
tmpQ2[11] = + tmpFx[2]*tmpObjS[1] + tmpFx[9]*tmpObjS[6] + tmpFx[16]*tmpObjS[11] + tmpFx[23]*tmpObjS[16] + tmpFx[30]*tmpObjS[21];
tmpQ2[12] = + tmpFx[2]*tmpObjS[2] + tmpFx[9]*tmpObjS[7] + tmpFx[16]*tmpObjS[12] + tmpFx[23]*tmpObjS[17] + tmpFx[30]*tmpObjS[22];
tmpQ2[13] = + tmpFx[2]*tmpObjS[3] + tmpFx[9]*tmpObjS[8] + tmpFx[16]*tmpObjS[13] + tmpFx[23]*tmpObjS[18] + tmpFx[30]*tmpObjS[23];
tmpQ2[14] = + tmpFx[2]*tmpObjS[4] + tmpFx[9]*tmpObjS[9] + tmpFx[16]*tmpObjS[14] + tmpFx[23]*tmpObjS[19] + tmpFx[30]*tmpObjS[24];
tmpQ2[15] = + tmpFx[3]*tmpObjS[0] + tmpFx[10]*tmpObjS[5] + tmpFx[17]*tmpObjS[10] + tmpFx[24]*tmpObjS[15] + tmpFx[31]*tmpObjS[20];
tmpQ2[16] = + tmpFx[3]*tmpObjS[1] + tmpFx[10]*tmpObjS[6] + tmpFx[17]*tmpObjS[11] + tmpFx[24]*tmpObjS[16] + tmpFx[31]*tmpObjS[21];
tmpQ2[17] = + tmpFx[3]*tmpObjS[2] + tmpFx[10]*tmpObjS[7] + tmpFx[17]*tmpObjS[12] + tmpFx[24]*tmpObjS[17] + tmpFx[31]*tmpObjS[22];
tmpQ2[18] = + tmpFx[3]*tmpObjS[3] + tmpFx[10]*tmpObjS[8] + tmpFx[17]*tmpObjS[13] + tmpFx[24]*tmpObjS[18] + tmpFx[31]*tmpObjS[23];
tmpQ2[19] = + tmpFx[3]*tmpObjS[4] + tmpFx[10]*tmpObjS[9] + tmpFx[17]*tmpObjS[14] + tmpFx[24]*tmpObjS[19] + tmpFx[31]*tmpObjS[24];
tmpQ2[20] = + tmpFx[4]*tmpObjS[0] + tmpFx[11]*tmpObjS[5] + tmpFx[18]*tmpObjS[10] + tmpFx[25]*tmpObjS[15] + tmpFx[32]*tmpObjS[20];
tmpQ2[21] = + tmpFx[4]*tmpObjS[1] + tmpFx[11]*tmpObjS[6] + tmpFx[18]*tmpObjS[11] + tmpFx[25]*tmpObjS[16] + tmpFx[32]*tmpObjS[21];
tmpQ2[22] = + tmpFx[4]*tmpObjS[2] + tmpFx[11]*tmpObjS[7] + tmpFx[18]*tmpObjS[12] + tmpFx[25]*tmpObjS[17] + tmpFx[32]*tmpObjS[22];
tmpQ2[23] = + tmpFx[4]*tmpObjS[3] + tmpFx[11]*tmpObjS[8] + tmpFx[18]*tmpObjS[13] + tmpFx[25]*tmpObjS[18] + tmpFx[32]*tmpObjS[23];
tmpQ2[24] = + tmpFx[4]*tmpObjS[4] + tmpFx[11]*tmpObjS[9] + tmpFx[18]*tmpObjS[14] + tmpFx[25]*tmpObjS[19] + tmpFx[32]*tmpObjS[24];
tmpQ2[25] = + tmpFx[5]*tmpObjS[0] + tmpFx[12]*tmpObjS[5] + tmpFx[19]*tmpObjS[10] + tmpFx[26]*tmpObjS[15] + tmpFx[33]*tmpObjS[20];
tmpQ2[26] = + tmpFx[5]*tmpObjS[1] + tmpFx[12]*tmpObjS[6] + tmpFx[19]*tmpObjS[11] + tmpFx[26]*tmpObjS[16] + tmpFx[33]*tmpObjS[21];
tmpQ2[27] = + tmpFx[5]*tmpObjS[2] + tmpFx[12]*tmpObjS[7] + tmpFx[19]*tmpObjS[12] + tmpFx[26]*tmpObjS[17] + tmpFx[33]*tmpObjS[22];
tmpQ2[28] = + tmpFx[5]*tmpObjS[3] + tmpFx[12]*tmpObjS[8] + tmpFx[19]*tmpObjS[13] + tmpFx[26]*tmpObjS[18] + tmpFx[33]*tmpObjS[23];
tmpQ2[29] = + tmpFx[5]*tmpObjS[4] + tmpFx[12]*tmpObjS[9] + tmpFx[19]*tmpObjS[14] + tmpFx[26]*tmpObjS[19] + tmpFx[33]*tmpObjS[24];
tmpQ2[30] = + tmpFx[6]*tmpObjS[0] + tmpFx[13]*tmpObjS[5] + tmpFx[20]*tmpObjS[10] + tmpFx[27]*tmpObjS[15] + tmpFx[34]*tmpObjS[20];
tmpQ2[31] = + tmpFx[6]*tmpObjS[1] + tmpFx[13]*tmpObjS[6] + tmpFx[20]*tmpObjS[11] + tmpFx[27]*tmpObjS[16] + tmpFx[34]*tmpObjS[21];
tmpQ2[32] = + tmpFx[6]*tmpObjS[2] + tmpFx[13]*tmpObjS[7] + tmpFx[20]*tmpObjS[12] + tmpFx[27]*tmpObjS[17] + tmpFx[34]*tmpObjS[22];
tmpQ2[33] = + tmpFx[6]*tmpObjS[3] + tmpFx[13]*tmpObjS[8] + tmpFx[20]*tmpObjS[13] + tmpFx[27]*tmpObjS[18] + tmpFx[34]*tmpObjS[23];
tmpQ2[34] = + tmpFx[6]*tmpObjS[4] + tmpFx[13]*tmpObjS[9] + tmpFx[20]*tmpObjS[14] + tmpFx[27]*tmpObjS[19] + tmpFx[34]*tmpObjS[24];
tmpQ1[0] = + tmpQ2[0]*tmpFx[0] + tmpQ2[1]*tmpFx[7] + tmpQ2[2]*tmpFx[14] + tmpQ2[3]*tmpFx[21] + tmpQ2[4]*tmpFx[28];
tmpQ1[1] = + tmpQ2[0]*tmpFx[1] + tmpQ2[1]*tmpFx[8] + tmpQ2[2]*tmpFx[15] + tmpQ2[3]*tmpFx[22] + tmpQ2[4]*tmpFx[29];
tmpQ1[2] = + tmpQ2[0]*tmpFx[2] + tmpQ2[1]*tmpFx[9] + tmpQ2[2]*tmpFx[16] + tmpQ2[3]*tmpFx[23] + tmpQ2[4]*tmpFx[30];
tmpQ1[3] = + tmpQ2[0]*tmpFx[3] + tmpQ2[1]*tmpFx[10] + tmpQ2[2]*tmpFx[17] + tmpQ2[3]*tmpFx[24] + tmpQ2[4]*tmpFx[31];
tmpQ1[4] = + tmpQ2[0]*tmpFx[4] + tmpQ2[1]*tmpFx[11] + tmpQ2[2]*tmpFx[18] + tmpQ2[3]*tmpFx[25] + tmpQ2[4]*tmpFx[32];
tmpQ1[5] = + tmpQ2[0]*tmpFx[5] + tmpQ2[1]*tmpFx[12] + tmpQ2[2]*tmpFx[19] + tmpQ2[3]*tmpFx[26] + tmpQ2[4]*tmpFx[33];
tmpQ1[6] = + tmpQ2[0]*tmpFx[6] + tmpQ2[1]*tmpFx[13] + tmpQ2[2]*tmpFx[20] + tmpQ2[3]*tmpFx[27] + tmpQ2[4]*tmpFx[34];
tmpQ1[7] = + tmpQ2[5]*tmpFx[0] + tmpQ2[6]*tmpFx[7] + tmpQ2[7]*tmpFx[14] + tmpQ2[8]*tmpFx[21] + tmpQ2[9]*tmpFx[28];
tmpQ1[8] = + tmpQ2[5]*tmpFx[1] + tmpQ2[6]*tmpFx[8] + tmpQ2[7]*tmpFx[15] + tmpQ2[8]*tmpFx[22] + tmpQ2[9]*tmpFx[29];
tmpQ1[9] = + tmpQ2[5]*tmpFx[2] + tmpQ2[6]*tmpFx[9] + tmpQ2[7]*tmpFx[16] + tmpQ2[8]*tmpFx[23] + tmpQ2[9]*tmpFx[30];
tmpQ1[10] = + tmpQ2[5]*tmpFx[3] + tmpQ2[6]*tmpFx[10] + tmpQ2[7]*tmpFx[17] + tmpQ2[8]*tmpFx[24] + tmpQ2[9]*tmpFx[31];
tmpQ1[11] = + tmpQ2[5]*tmpFx[4] + tmpQ2[6]*tmpFx[11] + tmpQ2[7]*tmpFx[18] + tmpQ2[8]*tmpFx[25] + tmpQ2[9]*tmpFx[32];
tmpQ1[12] = + tmpQ2[5]*tmpFx[5] + tmpQ2[6]*tmpFx[12] + tmpQ2[7]*tmpFx[19] + tmpQ2[8]*tmpFx[26] + tmpQ2[9]*tmpFx[33];
tmpQ1[13] = + tmpQ2[5]*tmpFx[6] + tmpQ2[6]*tmpFx[13] + tmpQ2[7]*tmpFx[20] + tmpQ2[8]*tmpFx[27] + tmpQ2[9]*tmpFx[34];
tmpQ1[14] = + tmpQ2[10]*tmpFx[0] + tmpQ2[11]*tmpFx[7] + tmpQ2[12]*tmpFx[14] + tmpQ2[13]*tmpFx[21] + tmpQ2[14]*tmpFx[28];
tmpQ1[15] = + tmpQ2[10]*tmpFx[1] + tmpQ2[11]*tmpFx[8] + tmpQ2[12]*tmpFx[15] + tmpQ2[13]*tmpFx[22] + tmpQ2[14]*tmpFx[29];
tmpQ1[16] = + tmpQ2[10]*tmpFx[2] + tmpQ2[11]*tmpFx[9] + tmpQ2[12]*tmpFx[16] + tmpQ2[13]*tmpFx[23] + tmpQ2[14]*tmpFx[30];
tmpQ1[17] = + tmpQ2[10]*tmpFx[3] + tmpQ2[11]*tmpFx[10] + tmpQ2[12]*tmpFx[17] + tmpQ2[13]*tmpFx[24] + tmpQ2[14]*tmpFx[31];
tmpQ1[18] = + tmpQ2[10]*tmpFx[4] + tmpQ2[11]*tmpFx[11] + tmpQ2[12]*tmpFx[18] + tmpQ2[13]*tmpFx[25] + tmpQ2[14]*tmpFx[32];
tmpQ1[19] = + tmpQ2[10]*tmpFx[5] + tmpQ2[11]*tmpFx[12] + tmpQ2[12]*tmpFx[19] + tmpQ2[13]*tmpFx[26] + tmpQ2[14]*tmpFx[33];
tmpQ1[20] = + tmpQ2[10]*tmpFx[6] + tmpQ2[11]*tmpFx[13] + tmpQ2[12]*tmpFx[20] + tmpQ2[13]*tmpFx[27] + tmpQ2[14]*tmpFx[34];
tmpQ1[21] = + tmpQ2[15]*tmpFx[0] + tmpQ2[16]*tmpFx[7] + tmpQ2[17]*tmpFx[14] + tmpQ2[18]*tmpFx[21] + tmpQ2[19]*tmpFx[28];
tmpQ1[22] = + tmpQ2[15]*tmpFx[1] + tmpQ2[16]*tmpFx[8] + tmpQ2[17]*tmpFx[15] + tmpQ2[18]*tmpFx[22] + tmpQ2[19]*tmpFx[29];
tmpQ1[23] = + tmpQ2[15]*tmpFx[2] + tmpQ2[16]*tmpFx[9] + tmpQ2[17]*tmpFx[16] + tmpQ2[18]*tmpFx[23] + tmpQ2[19]*tmpFx[30];
tmpQ1[24] = + tmpQ2[15]*tmpFx[3] + tmpQ2[16]*tmpFx[10] + tmpQ2[17]*tmpFx[17] + tmpQ2[18]*tmpFx[24] + tmpQ2[19]*tmpFx[31];
tmpQ1[25] = + tmpQ2[15]*tmpFx[4] + tmpQ2[16]*tmpFx[11] + tmpQ2[17]*tmpFx[18] + tmpQ2[18]*tmpFx[25] + tmpQ2[19]*tmpFx[32];
tmpQ1[26] = + tmpQ2[15]*tmpFx[5] + tmpQ2[16]*tmpFx[12] + tmpQ2[17]*tmpFx[19] + tmpQ2[18]*tmpFx[26] + tmpQ2[19]*tmpFx[33];
tmpQ1[27] = + tmpQ2[15]*tmpFx[6] + tmpQ2[16]*tmpFx[13] + tmpQ2[17]*tmpFx[20] + tmpQ2[18]*tmpFx[27] + tmpQ2[19]*tmpFx[34];
tmpQ1[28] = + tmpQ2[20]*tmpFx[0] + tmpQ2[21]*tmpFx[7] + tmpQ2[22]*tmpFx[14] + tmpQ2[23]*tmpFx[21] + tmpQ2[24]*tmpFx[28];
tmpQ1[29] = + tmpQ2[20]*tmpFx[1] + tmpQ2[21]*tmpFx[8] + tmpQ2[22]*tmpFx[15] + tmpQ2[23]*tmpFx[22] + tmpQ2[24]*tmpFx[29];
tmpQ1[30] = + tmpQ2[20]*tmpFx[2] + tmpQ2[21]*tmpFx[9] + tmpQ2[22]*tmpFx[16] + tmpQ2[23]*tmpFx[23] + tmpQ2[24]*tmpFx[30];
tmpQ1[31] = + tmpQ2[20]*tmpFx[3] + tmpQ2[21]*tmpFx[10] + tmpQ2[22]*tmpFx[17] + tmpQ2[23]*tmpFx[24] + tmpQ2[24]*tmpFx[31];
tmpQ1[32] = + tmpQ2[20]*tmpFx[4] + tmpQ2[21]*tmpFx[11] + tmpQ2[22]*tmpFx[18] + tmpQ2[23]*tmpFx[25] + tmpQ2[24]*tmpFx[32];
tmpQ1[33] = + tmpQ2[20]*tmpFx[5] + tmpQ2[21]*tmpFx[12] + tmpQ2[22]*tmpFx[19] + tmpQ2[23]*tmpFx[26] + tmpQ2[24]*tmpFx[33];
tmpQ1[34] = + tmpQ2[20]*tmpFx[6] + tmpQ2[21]*tmpFx[13] + tmpQ2[22]*tmpFx[20] + tmpQ2[23]*tmpFx[27] + tmpQ2[24]*tmpFx[34];
tmpQ1[35] = + tmpQ2[25]*tmpFx[0] + tmpQ2[26]*tmpFx[7] + tmpQ2[27]*tmpFx[14] + tmpQ2[28]*tmpFx[21] + tmpQ2[29]*tmpFx[28];
tmpQ1[36] = + tmpQ2[25]*tmpFx[1] + tmpQ2[26]*tmpFx[8] + tmpQ2[27]*tmpFx[15] + tmpQ2[28]*tmpFx[22] + tmpQ2[29]*tmpFx[29];
tmpQ1[37] = + tmpQ2[25]*tmpFx[2] + tmpQ2[26]*tmpFx[9] + tmpQ2[27]*tmpFx[16] + tmpQ2[28]*tmpFx[23] + tmpQ2[29]*tmpFx[30];
tmpQ1[38] = + tmpQ2[25]*tmpFx[3] + tmpQ2[26]*tmpFx[10] + tmpQ2[27]*tmpFx[17] + tmpQ2[28]*tmpFx[24] + tmpQ2[29]*tmpFx[31];
tmpQ1[39] = + tmpQ2[25]*tmpFx[4] + tmpQ2[26]*tmpFx[11] + tmpQ2[27]*tmpFx[18] + tmpQ2[28]*tmpFx[25] + tmpQ2[29]*tmpFx[32];
tmpQ1[40] = + tmpQ2[25]*tmpFx[5] + tmpQ2[26]*tmpFx[12] + tmpQ2[27]*tmpFx[19] + tmpQ2[28]*tmpFx[26] + tmpQ2[29]*tmpFx[33];
tmpQ1[41] = + tmpQ2[25]*tmpFx[6] + tmpQ2[26]*tmpFx[13] + tmpQ2[27]*tmpFx[20] + tmpQ2[28]*tmpFx[27] + tmpQ2[29]*tmpFx[34];
tmpQ1[42] = + tmpQ2[30]*tmpFx[0] + tmpQ2[31]*tmpFx[7] + tmpQ2[32]*tmpFx[14] + tmpQ2[33]*tmpFx[21] + tmpQ2[34]*tmpFx[28];
tmpQ1[43] = + tmpQ2[30]*tmpFx[1] + tmpQ2[31]*tmpFx[8] + tmpQ2[32]*tmpFx[15] + tmpQ2[33]*tmpFx[22] + tmpQ2[34]*tmpFx[29];
tmpQ1[44] = + tmpQ2[30]*tmpFx[2] + tmpQ2[31]*tmpFx[9] + tmpQ2[32]*tmpFx[16] + tmpQ2[33]*tmpFx[23] + tmpQ2[34]*tmpFx[30];
tmpQ1[45] = + tmpQ2[30]*tmpFx[3] + tmpQ2[31]*tmpFx[10] + tmpQ2[32]*tmpFx[17] + tmpQ2[33]*tmpFx[24] + tmpQ2[34]*tmpFx[31];
tmpQ1[46] = + tmpQ2[30]*tmpFx[4] + tmpQ2[31]*tmpFx[11] + tmpQ2[32]*tmpFx[18] + tmpQ2[33]*tmpFx[25] + tmpQ2[34]*tmpFx[32];
tmpQ1[47] = + tmpQ2[30]*tmpFx[5] + tmpQ2[31]*tmpFx[12] + tmpQ2[32]*tmpFx[19] + tmpQ2[33]*tmpFx[26] + tmpQ2[34]*tmpFx[33];
tmpQ1[48] = + tmpQ2[30]*tmpFx[6] + tmpQ2[31]*tmpFx[13] + tmpQ2[32]*tmpFx[20] + tmpQ2[33]*tmpFx[27] + tmpQ2[34]*tmpFx[34];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[10];
tmpR2[1] = +tmpObjS[11];
tmpR2[2] = +tmpObjS[12];
tmpR2[3] = +tmpObjS[13];
tmpR2[4] = +tmpObjS[14];
tmpR2[5] = +tmpObjS[15];
tmpR2[6] = +tmpObjS[16];
tmpR2[7] = +tmpObjS[17];
tmpR2[8] = +tmpObjS[18];
tmpR2[9] = +tmpObjS[19];
tmpR2[10] = +tmpObjS[20];
tmpR2[11] = +tmpObjS[21];
tmpR2[12] = +tmpObjS[22];
tmpR2[13] = +tmpObjS[23];
tmpR2[14] = +tmpObjS[24];
tmpR1[0] = + tmpR2[2];
tmpR1[1] = + tmpR2[3];
tmpR1[2] = + tmpR2[4];
tmpR1[3] = + tmpR2[7];
tmpR1[4] = + tmpR2[8];
tmpR1[5] = + tmpR2[9];
tmpR1[6] = + tmpR2[12];
tmpR1[7] = + tmpR2[13];
tmpR1[8] = + tmpR2[14];
}

void acado_setObjQN1QN2( real_t* const tmpFx, real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = + tmpFx[0]*tmpObjSEndTerm[0] + tmpFx[7]*tmpObjSEndTerm[2];
tmpQN2[1] = + tmpFx[0]*tmpObjSEndTerm[1] + tmpFx[7]*tmpObjSEndTerm[3];
tmpQN2[2] = + tmpFx[1]*tmpObjSEndTerm[0] + tmpFx[8]*tmpObjSEndTerm[2];
tmpQN2[3] = + tmpFx[1]*tmpObjSEndTerm[1] + tmpFx[8]*tmpObjSEndTerm[3];
tmpQN2[4] = + tmpFx[2]*tmpObjSEndTerm[0] + tmpFx[9]*tmpObjSEndTerm[2];
tmpQN2[5] = + tmpFx[2]*tmpObjSEndTerm[1] + tmpFx[9]*tmpObjSEndTerm[3];
tmpQN2[6] = + tmpFx[3]*tmpObjSEndTerm[0] + tmpFx[10]*tmpObjSEndTerm[2];
tmpQN2[7] = + tmpFx[3]*tmpObjSEndTerm[1] + tmpFx[10]*tmpObjSEndTerm[3];
tmpQN2[8] = + tmpFx[4]*tmpObjSEndTerm[0] + tmpFx[11]*tmpObjSEndTerm[2];
tmpQN2[9] = + tmpFx[4]*tmpObjSEndTerm[1] + tmpFx[11]*tmpObjSEndTerm[3];
tmpQN2[10] = + tmpFx[5]*tmpObjSEndTerm[0] + tmpFx[12]*tmpObjSEndTerm[2];
tmpQN2[11] = + tmpFx[5]*tmpObjSEndTerm[1] + tmpFx[12]*tmpObjSEndTerm[3];
tmpQN2[12] = + tmpFx[6]*tmpObjSEndTerm[0] + tmpFx[13]*tmpObjSEndTerm[2];
tmpQN2[13] = + tmpFx[6]*tmpObjSEndTerm[1] + tmpFx[13]*tmpObjSEndTerm[3];
tmpQN1[0] = + tmpQN2[0]*tmpFx[0] + tmpQN2[1]*tmpFx[7];
tmpQN1[1] = + tmpQN2[0]*tmpFx[1] + tmpQN2[1]*tmpFx[8];
tmpQN1[2] = + tmpQN2[0]*tmpFx[2] + tmpQN2[1]*tmpFx[9];
tmpQN1[3] = + tmpQN2[0]*tmpFx[3] + tmpQN2[1]*tmpFx[10];
tmpQN1[4] = + tmpQN2[0]*tmpFx[4] + tmpQN2[1]*tmpFx[11];
tmpQN1[5] = + tmpQN2[0]*tmpFx[5] + tmpQN2[1]*tmpFx[12];
tmpQN1[6] = + tmpQN2[0]*tmpFx[6] + tmpQN2[1]*tmpFx[13];
tmpQN1[7] = + tmpQN2[2]*tmpFx[0] + tmpQN2[3]*tmpFx[7];
tmpQN1[8] = + tmpQN2[2]*tmpFx[1] + tmpQN2[3]*tmpFx[8];
tmpQN1[9] = + tmpQN2[2]*tmpFx[2] + tmpQN2[3]*tmpFx[9];
tmpQN1[10] = + tmpQN2[2]*tmpFx[3] + tmpQN2[3]*tmpFx[10];
tmpQN1[11] = + tmpQN2[2]*tmpFx[4] + tmpQN2[3]*tmpFx[11];
tmpQN1[12] = + tmpQN2[2]*tmpFx[5] + tmpQN2[3]*tmpFx[12];
tmpQN1[13] = + tmpQN2[2]*tmpFx[6] + tmpQN2[3]*tmpFx[13];
tmpQN1[14] = + tmpQN2[4]*tmpFx[0] + tmpQN2[5]*tmpFx[7];
tmpQN1[15] = + tmpQN2[4]*tmpFx[1] + tmpQN2[5]*tmpFx[8];
tmpQN1[16] = + tmpQN2[4]*tmpFx[2] + tmpQN2[5]*tmpFx[9];
tmpQN1[17] = + tmpQN2[4]*tmpFx[3] + tmpQN2[5]*tmpFx[10];
tmpQN1[18] = + tmpQN2[4]*tmpFx[4] + tmpQN2[5]*tmpFx[11];
tmpQN1[19] = + tmpQN2[4]*tmpFx[5] + tmpQN2[5]*tmpFx[12];
tmpQN1[20] = + tmpQN2[4]*tmpFx[6] + tmpQN2[5]*tmpFx[13];
tmpQN1[21] = + tmpQN2[6]*tmpFx[0] + tmpQN2[7]*tmpFx[7];
tmpQN1[22] = + tmpQN2[6]*tmpFx[1] + tmpQN2[7]*tmpFx[8];
tmpQN1[23] = + tmpQN2[6]*tmpFx[2] + tmpQN2[7]*tmpFx[9];
tmpQN1[24] = + tmpQN2[6]*tmpFx[3] + tmpQN2[7]*tmpFx[10];
tmpQN1[25] = + tmpQN2[6]*tmpFx[4] + tmpQN2[7]*tmpFx[11];
tmpQN1[26] = + tmpQN2[6]*tmpFx[5] + tmpQN2[7]*tmpFx[12];
tmpQN1[27] = + tmpQN2[6]*tmpFx[6] + tmpQN2[7]*tmpFx[13];
tmpQN1[28] = + tmpQN2[8]*tmpFx[0] + tmpQN2[9]*tmpFx[7];
tmpQN1[29] = + tmpQN2[8]*tmpFx[1] + tmpQN2[9]*tmpFx[8];
tmpQN1[30] = + tmpQN2[8]*tmpFx[2] + tmpQN2[9]*tmpFx[9];
tmpQN1[31] = + tmpQN2[8]*tmpFx[3] + tmpQN2[9]*tmpFx[10];
tmpQN1[32] = + tmpQN2[8]*tmpFx[4] + tmpQN2[9]*tmpFx[11];
tmpQN1[33] = + tmpQN2[8]*tmpFx[5] + tmpQN2[9]*tmpFx[12];
tmpQN1[34] = + tmpQN2[8]*tmpFx[6] + tmpQN2[9]*tmpFx[13];
tmpQN1[35] = + tmpQN2[10]*tmpFx[0] + tmpQN2[11]*tmpFx[7];
tmpQN1[36] = + tmpQN2[10]*tmpFx[1] + tmpQN2[11]*tmpFx[8];
tmpQN1[37] = + tmpQN2[10]*tmpFx[2] + tmpQN2[11]*tmpFx[9];
tmpQN1[38] = + tmpQN2[10]*tmpFx[3] + tmpQN2[11]*tmpFx[10];
tmpQN1[39] = + tmpQN2[10]*tmpFx[4] + tmpQN2[11]*tmpFx[11];
tmpQN1[40] = + tmpQN2[10]*tmpFx[5] + tmpQN2[11]*tmpFx[12];
tmpQN1[41] = + tmpQN2[10]*tmpFx[6] + tmpQN2[11]*tmpFx[13];
tmpQN1[42] = + tmpQN2[12]*tmpFx[0] + tmpQN2[13]*tmpFx[7];
tmpQN1[43] = + tmpQN2[12]*tmpFx[1] + tmpQN2[13]*tmpFx[8];
tmpQN1[44] = + tmpQN2[12]*tmpFx[2] + tmpQN2[13]*tmpFx[9];
tmpQN1[45] = + tmpQN2[12]*tmpFx[3] + tmpQN2[13]*tmpFx[10];
tmpQN1[46] = + tmpQN2[12]*tmpFx[4] + tmpQN2[13]*tmpFx[11];
tmpQN1[47] = + tmpQN2[12]*tmpFx[5] + tmpQN2[13]*tmpFx[12];
tmpQN1[48] = + tmpQN2[12]*tmpFx[6] + tmpQN2[13]*tmpFx[13];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 100; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 7];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 7 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 7 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 7 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 7 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 7 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 7 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.u[runObj * 3];
acadoWorkspace.objValueIn[8] = acadoVariables.u[runObj * 3 + 1];
acadoWorkspace.objValueIn[9] = acadoVariables.u[runObj * 3 + 2];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 9];
acadoWorkspace.objValueIn[11] = acadoVariables.od[runObj * 9 + 1];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 9 + 2];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 9 + 3];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 9 + 4];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 9 + 5];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 9 + 6];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 9 + 7];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 9 + 8];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 5] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 5 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 5 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 5 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 5 + 4] = acadoWorkspace.objValueOut[4];

acado_setObjQ1Q2( &(acadoWorkspace.objValueOut[ 5 ]), &(acadoVariables.W[ runObj * 25 ]), &(acadoWorkspace.Q1[ runObj * 49 ]), &(acadoWorkspace.Q2[ runObj * 35 ]) );

acado_setObjR1R2( &(acadoVariables.W[ runObj * 25 ]), &(acadoWorkspace.R1[ runObj * 9 ]), &(acadoWorkspace.R2[ runObj * 15 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[700];
acadoWorkspace.objValueIn[1] = acadoVariables.x[701];
acadoWorkspace.objValueIn[2] = acadoVariables.x[702];
acadoWorkspace.objValueIn[3] = acadoVariables.x[703];
acadoWorkspace.objValueIn[4] = acadoVariables.x[704];
acadoWorkspace.objValueIn[5] = acadoVariables.x[705];
acadoWorkspace.objValueIn[6] = acadoVariables.x[706];
acadoWorkspace.objValueIn[7] = acadoVariables.od[900];
acadoWorkspace.objValueIn[8] = acadoVariables.od[901];
acadoWorkspace.objValueIn[9] = acadoVariables.od[902];
acadoWorkspace.objValueIn[10] = acadoVariables.od[903];
acadoWorkspace.objValueIn[11] = acadoVariables.od[904];
acadoWorkspace.objValueIn[12] = acadoVariables.od[905];
acadoWorkspace.objValueIn[13] = acadoVariables.od[906];
acadoWorkspace.objValueIn[14] = acadoVariables.od[907];
acadoWorkspace.objValueIn[15] = acadoVariables.od[908];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];

acado_setObjQN1QN2( &(acadoWorkspace.objValueOut[ 2 ]), acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6];
dNew[1] += + Gx1[7]*dOld[0] + Gx1[8]*dOld[1] + Gx1[9]*dOld[2] + Gx1[10]*dOld[3] + Gx1[11]*dOld[4] + Gx1[12]*dOld[5] + Gx1[13]*dOld[6];
dNew[2] += + Gx1[14]*dOld[0] + Gx1[15]*dOld[1] + Gx1[16]*dOld[2] + Gx1[17]*dOld[3] + Gx1[18]*dOld[4] + Gx1[19]*dOld[5] + Gx1[20]*dOld[6];
dNew[3] += + Gx1[21]*dOld[0] + Gx1[22]*dOld[1] + Gx1[23]*dOld[2] + Gx1[24]*dOld[3] + Gx1[25]*dOld[4] + Gx1[26]*dOld[5] + Gx1[27]*dOld[6];
dNew[4] += + Gx1[28]*dOld[0] + Gx1[29]*dOld[1] + Gx1[30]*dOld[2] + Gx1[31]*dOld[3] + Gx1[32]*dOld[4] + Gx1[33]*dOld[5] + Gx1[34]*dOld[6];
dNew[5] += + Gx1[35]*dOld[0] + Gx1[36]*dOld[1] + Gx1[37]*dOld[2] + Gx1[38]*dOld[3] + Gx1[39]*dOld[4] + Gx1[40]*dOld[5] + Gx1[41]*dOld[6];
dNew[6] += + Gx1[42]*dOld[0] + Gx1[43]*dOld[1] + Gx1[44]*dOld[2] + Gx1[45]*dOld[3] + Gx1[46]*dOld[4] + Gx1[47]*dOld[5] + Gx1[48]*dOld[6];
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
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
Gx2[25] = Gx1[25];
Gx2[26] = Gx1[26];
Gx2[27] = Gx1[27];
Gx2[28] = Gx1[28];
Gx2[29] = Gx1[29];
Gx2[30] = Gx1[30];
Gx2[31] = Gx1[31];
Gx2[32] = Gx1[32];
Gx2[33] = Gx1[33];
Gx2[34] = Gx1[34];
Gx2[35] = Gx1[35];
Gx2[36] = Gx1[36];
Gx2[37] = Gx1[37];
Gx2[38] = Gx1[38];
Gx2[39] = Gx1[39];
Gx2[40] = Gx1[40];
Gx2[41] = Gx1[41];
Gx2[42] = Gx1[42];
Gx2[43] = Gx1[43];
Gx2[44] = Gx1[44];
Gx2[45] = Gx1[45];
Gx2[46] = Gx1[46];
Gx2[47] = Gx1[47];
Gx2[48] = Gx1[48];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[14] + Gx1[3]*Gx2[21] + Gx1[4]*Gx2[28] + Gx1[5]*Gx2[35] + Gx1[6]*Gx2[42];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[15] + Gx1[3]*Gx2[22] + Gx1[4]*Gx2[29] + Gx1[5]*Gx2[36] + Gx1[6]*Gx2[43];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[16] + Gx1[3]*Gx2[23] + Gx1[4]*Gx2[30] + Gx1[5]*Gx2[37] + Gx1[6]*Gx2[44];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[17] + Gx1[3]*Gx2[24] + Gx1[4]*Gx2[31] + Gx1[5]*Gx2[38] + Gx1[6]*Gx2[45];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[18] + Gx1[3]*Gx2[25] + Gx1[4]*Gx2[32] + Gx1[5]*Gx2[39] + Gx1[6]*Gx2[46];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[12] + Gx1[2]*Gx2[19] + Gx1[3]*Gx2[26] + Gx1[4]*Gx2[33] + Gx1[5]*Gx2[40] + Gx1[6]*Gx2[47];
Gx3[6] = + Gx1[0]*Gx2[6] + Gx1[1]*Gx2[13] + Gx1[2]*Gx2[20] + Gx1[3]*Gx2[27] + Gx1[4]*Gx2[34] + Gx1[5]*Gx2[41] + Gx1[6]*Gx2[48];
Gx3[7] = + Gx1[7]*Gx2[0] + Gx1[8]*Gx2[7] + Gx1[9]*Gx2[14] + Gx1[10]*Gx2[21] + Gx1[11]*Gx2[28] + Gx1[12]*Gx2[35] + Gx1[13]*Gx2[42];
Gx3[8] = + Gx1[7]*Gx2[1] + Gx1[8]*Gx2[8] + Gx1[9]*Gx2[15] + Gx1[10]*Gx2[22] + Gx1[11]*Gx2[29] + Gx1[12]*Gx2[36] + Gx1[13]*Gx2[43];
Gx3[9] = + Gx1[7]*Gx2[2] + Gx1[8]*Gx2[9] + Gx1[9]*Gx2[16] + Gx1[10]*Gx2[23] + Gx1[11]*Gx2[30] + Gx1[12]*Gx2[37] + Gx1[13]*Gx2[44];
Gx3[10] = + Gx1[7]*Gx2[3] + Gx1[8]*Gx2[10] + Gx1[9]*Gx2[17] + Gx1[10]*Gx2[24] + Gx1[11]*Gx2[31] + Gx1[12]*Gx2[38] + Gx1[13]*Gx2[45];
Gx3[11] = + Gx1[7]*Gx2[4] + Gx1[8]*Gx2[11] + Gx1[9]*Gx2[18] + Gx1[10]*Gx2[25] + Gx1[11]*Gx2[32] + Gx1[12]*Gx2[39] + Gx1[13]*Gx2[46];
Gx3[12] = + Gx1[7]*Gx2[5] + Gx1[8]*Gx2[12] + Gx1[9]*Gx2[19] + Gx1[10]*Gx2[26] + Gx1[11]*Gx2[33] + Gx1[12]*Gx2[40] + Gx1[13]*Gx2[47];
Gx3[13] = + Gx1[7]*Gx2[6] + Gx1[8]*Gx2[13] + Gx1[9]*Gx2[20] + Gx1[10]*Gx2[27] + Gx1[11]*Gx2[34] + Gx1[12]*Gx2[41] + Gx1[13]*Gx2[48];
Gx3[14] = + Gx1[14]*Gx2[0] + Gx1[15]*Gx2[7] + Gx1[16]*Gx2[14] + Gx1[17]*Gx2[21] + Gx1[18]*Gx2[28] + Gx1[19]*Gx2[35] + Gx1[20]*Gx2[42];
Gx3[15] = + Gx1[14]*Gx2[1] + Gx1[15]*Gx2[8] + Gx1[16]*Gx2[15] + Gx1[17]*Gx2[22] + Gx1[18]*Gx2[29] + Gx1[19]*Gx2[36] + Gx1[20]*Gx2[43];
Gx3[16] = + Gx1[14]*Gx2[2] + Gx1[15]*Gx2[9] + Gx1[16]*Gx2[16] + Gx1[17]*Gx2[23] + Gx1[18]*Gx2[30] + Gx1[19]*Gx2[37] + Gx1[20]*Gx2[44];
Gx3[17] = + Gx1[14]*Gx2[3] + Gx1[15]*Gx2[10] + Gx1[16]*Gx2[17] + Gx1[17]*Gx2[24] + Gx1[18]*Gx2[31] + Gx1[19]*Gx2[38] + Gx1[20]*Gx2[45];
Gx3[18] = + Gx1[14]*Gx2[4] + Gx1[15]*Gx2[11] + Gx1[16]*Gx2[18] + Gx1[17]*Gx2[25] + Gx1[18]*Gx2[32] + Gx1[19]*Gx2[39] + Gx1[20]*Gx2[46];
Gx3[19] = + Gx1[14]*Gx2[5] + Gx1[15]*Gx2[12] + Gx1[16]*Gx2[19] + Gx1[17]*Gx2[26] + Gx1[18]*Gx2[33] + Gx1[19]*Gx2[40] + Gx1[20]*Gx2[47];
Gx3[20] = + Gx1[14]*Gx2[6] + Gx1[15]*Gx2[13] + Gx1[16]*Gx2[20] + Gx1[17]*Gx2[27] + Gx1[18]*Gx2[34] + Gx1[19]*Gx2[41] + Gx1[20]*Gx2[48];
Gx3[21] = + Gx1[21]*Gx2[0] + Gx1[22]*Gx2[7] + Gx1[23]*Gx2[14] + Gx1[24]*Gx2[21] + Gx1[25]*Gx2[28] + Gx1[26]*Gx2[35] + Gx1[27]*Gx2[42];
Gx3[22] = + Gx1[21]*Gx2[1] + Gx1[22]*Gx2[8] + Gx1[23]*Gx2[15] + Gx1[24]*Gx2[22] + Gx1[25]*Gx2[29] + Gx1[26]*Gx2[36] + Gx1[27]*Gx2[43];
Gx3[23] = + Gx1[21]*Gx2[2] + Gx1[22]*Gx2[9] + Gx1[23]*Gx2[16] + Gx1[24]*Gx2[23] + Gx1[25]*Gx2[30] + Gx1[26]*Gx2[37] + Gx1[27]*Gx2[44];
Gx3[24] = + Gx1[21]*Gx2[3] + Gx1[22]*Gx2[10] + Gx1[23]*Gx2[17] + Gx1[24]*Gx2[24] + Gx1[25]*Gx2[31] + Gx1[26]*Gx2[38] + Gx1[27]*Gx2[45];
Gx3[25] = + Gx1[21]*Gx2[4] + Gx1[22]*Gx2[11] + Gx1[23]*Gx2[18] + Gx1[24]*Gx2[25] + Gx1[25]*Gx2[32] + Gx1[26]*Gx2[39] + Gx1[27]*Gx2[46];
Gx3[26] = + Gx1[21]*Gx2[5] + Gx1[22]*Gx2[12] + Gx1[23]*Gx2[19] + Gx1[24]*Gx2[26] + Gx1[25]*Gx2[33] + Gx1[26]*Gx2[40] + Gx1[27]*Gx2[47];
Gx3[27] = + Gx1[21]*Gx2[6] + Gx1[22]*Gx2[13] + Gx1[23]*Gx2[20] + Gx1[24]*Gx2[27] + Gx1[25]*Gx2[34] + Gx1[26]*Gx2[41] + Gx1[27]*Gx2[48];
Gx3[28] = + Gx1[28]*Gx2[0] + Gx1[29]*Gx2[7] + Gx1[30]*Gx2[14] + Gx1[31]*Gx2[21] + Gx1[32]*Gx2[28] + Gx1[33]*Gx2[35] + Gx1[34]*Gx2[42];
Gx3[29] = + Gx1[28]*Gx2[1] + Gx1[29]*Gx2[8] + Gx1[30]*Gx2[15] + Gx1[31]*Gx2[22] + Gx1[32]*Gx2[29] + Gx1[33]*Gx2[36] + Gx1[34]*Gx2[43];
Gx3[30] = + Gx1[28]*Gx2[2] + Gx1[29]*Gx2[9] + Gx1[30]*Gx2[16] + Gx1[31]*Gx2[23] + Gx1[32]*Gx2[30] + Gx1[33]*Gx2[37] + Gx1[34]*Gx2[44];
Gx3[31] = + Gx1[28]*Gx2[3] + Gx1[29]*Gx2[10] + Gx1[30]*Gx2[17] + Gx1[31]*Gx2[24] + Gx1[32]*Gx2[31] + Gx1[33]*Gx2[38] + Gx1[34]*Gx2[45];
Gx3[32] = + Gx1[28]*Gx2[4] + Gx1[29]*Gx2[11] + Gx1[30]*Gx2[18] + Gx1[31]*Gx2[25] + Gx1[32]*Gx2[32] + Gx1[33]*Gx2[39] + Gx1[34]*Gx2[46];
Gx3[33] = + Gx1[28]*Gx2[5] + Gx1[29]*Gx2[12] + Gx1[30]*Gx2[19] + Gx1[31]*Gx2[26] + Gx1[32]*Gx2[33] + Gx1[33]*Gx2[40] + Gx1[34]*Gx2[47];
Gx3[34] = + Gx1[28]*Gx2[6] + Gx1[29]*Gx2[13] + Gx1[30]*Gx2[20] + Gx1[31]*Gx2[27] + Gx1[32]*Gx2[34] + Gx1[33]*Gx2[41] + Gx1[34]*Gx2[48];
Gx3[35] = + Gx1[35]*Gx2[0] + Gx1[36]*Gx2[7] + Gx1[37]*Gx2[14] + Gx1[38]*Gx2[21] + Gx1[39]*Gx2[28] + Gx1[40]*Gx2[35] + Gx1[41]*Gx2[42];
Gx3[36] = + Gx1[35]*Gx2[1] + Gx1[36]*Gx2[8] + Gx1[37]*Gx2[15] + Gx1[38]*Gx2[22] + Gx1[39]*Gx2[29] + Gx1[40]*Gx2[36] + Gx1[41]*Gx2[43];
Gx3[37] = + Gx1[35]*Gx2[2] + Gx1[36]*Gx2[9] + Gx1[37]*Gx2[16] + Gx1[38]*Gx2[23] + Gx1[39]*Gx2[30] + Gx1[40]*Gx2[37] + Gx1[41]*Gx2[44];
Gx3[38] = + Gx1[35]*Gx2[3] + Gx1[36]*Gx2[10] + Gx1[37]*Gx2[17] + Gx1[38]*Gx2[24] + Gx1[39]*Gx2[31] + Gx1[40]*Gx2[38] + Gx1[41]*Gx2[45];
Gx3[39] = + Gx1[35]*Gx2[4] + Gx1[36]*Gx2[11] + Gx1[37]*Gx2[18] + Gx1[38]*Gx2[25] + Gx1[39]*Gx2[32] + Gx1[40]*Gx2[39] + Gx1[41]*Gx2[46];
Gx3[40] = + Gx1[35]*Gx2[5] + Gx1[36]*Gx2[12] + Gx1[37]*Gx2[19] + Gx1[38]*Gx2[26] + Gx1[39]*Gx2[33] + Gx1[40]*Gx2[40] + Gx1[41]*Gx2[47];
Gx3[41] = + Gx1[35]*Gx2[6] + Gx1[36]*Gx2[13] + Gx1[37]*Gx2[20] + Gx1[38]*Gx2[27] + Gx1[39]*Gx2[34] + Gx1[40]*Gx2[41] + Gx1[41]*Gx2[48];
Gx3[42] = + Gx1[42]*Gx2[0] + Gx1[43]*Gx2[7] + Gx1[44]*Gx2[14] + Gx1[45]*Gx2[21] + Gx1[46]*Gx2[28] + Gx1[47]*Gx2[35] + Gx1[48]*Gx2[42];
Gx3[43] = + Gx1[42]*Gx2[1] + Gx1[43]*Gx2[8] + Gx1[44]*Gx2[15] + Gx1[45]*Gx2[22] + Gx1[46]*Gx2[29] + Gx1[47]*Gx2[36] + Gx1[48]*Gx2[43];
Gx3[44] = + Gx1[42]*Gx2[2] + Gx1[43]*Gx2[9] + Gx1[44]*Gx2[16] + Gx1[45]*Gx2[23] + Gx1[46]*Gx2[30] + Gx1[47]*Gx2[37] + Gx1[48]*Gx2[44];
Gx3[45] = + Gx1[42]*Gx2[3] + Gx1[43]*Gx2[10] + Gx1[44]*Gx2[17] + Gx1[45]*Gx2[24] + Gx1[46]*Gx2[31] + Gx1[47]*Gx2[38] + Gx1[48]*Gx2[45];
Gx3[46] = + Gx1[42]*Gx2[4] + Gx1[43]*Gx2[11] + Gx1[44]*Gx2[18] + Gx1[45]*Gx2[25] + Gx1[46]*Gx2[32] + Gx1[47]*Gx2[39] + Gx1[48]*Gx2[46];
Gx3[47] = + Gx1[42]*Gx2[5] + Gx1[43]*Gx2[12] + Gx1[44]*Gx2[19] + Gx1[45]*Gx2[26] + Gx1[46]*Gx2[33] + Gx1[47]*Gx2[40] + Gx1[48]*Gx2[47];
Gx3[48] = + Gx1[42]*Gx2[6] + Gx1[43]*Gx2[13] + Gx1[44]*Gx2[20] + Gx1[45]*Gx2[27] + Gx1[46]*Gx2[34] + Gx1[47]*Gx2[41] + Gx1[48]*Gx2[48];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[6] + Gx1[3]*Gu1[9] + Gx1[4]*Gu1[12] + Gx1[5]*Gu1[15] + Gx1[6]*Gu1[18];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[7] + Gx1[3]*Gu1[10] + Gx1[4]*Gu1[13] + Gx1[5]*Gu1[16] + Gx1[6]*Gu1[19];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[11] + Gx1[4]*Gu1[14] + Gx1[5]*Gu1[17] + Gx1[6]*Gu1[20];
Gu2[3] = + Gx1[7]*Gu1[0] + Gx1[8]*Gu1[3] + Gx1[9]*Gu1[6] + Gx1[10]*Gu1[9] + Gx1[11]*Gu1[12] + Gx1[12]*Gu1[15] + Gx1[13]*Gu1[18];
Gu2[4] = + Gx1[7]*Gu1[1] + Gx1[8]*Gu1[4] + Gx1[9]*Gu1[7] + Gx1[10]*Gu1[10] + Gx1[11]*Gu1[13] + Gx1[12]*Gu1[16] + Gx1[13]*Gu1[19];
Gu2[5] = + Gx1[7]*Gu1[2] + Gx1[8]*Gu1[5] + Gx1[9]*Gu1[8] + Gx1[10]*Gu1[11] + Gx1[11]*Gu1[14] + Gx1[12]*Gu1[17] + Gx1[13]*Gu1[20];
Gu2[6] = + Gx1[14]*Gu1[0] + Gx1[15]*Gu1[3] + Gx1[16]*Gu1[6] + Gx1[17]*Gu1[9] + Gx1[18]*Gu1[12] + Gx1[19]*Gu1[15] + Gx1[20]*Gu1[18];
Gu2[7] = + Gx1[14]*Gu1[1] + Gx1[15]*Gu1[4] + Gx1[16]*Gu1[7] + Gx1[17]*Gu1[10] + Gx1[18]*Gu1[13] + Gx1[19]*Gu1[16] + Gx1[20]*Gu1[19];
Gu2[8] = + Gx1[14]*Gu1[2] + Gx1[15]*Gu1[5] + Gx1[16]*Gu1[8] + Gx1[17]*Gu1[11] + Gx1[18]*Gu1[14] + Gx1[19]*Gu1[17] + Gx1[20]*Gu1[20];
Gu2[9] = + Gx1[21]*Gu1[0] + Gx1[22]*Gu1[3] + Gx1[23]*Gu1[6] + Gx1[24]*Gu1[9] + Gx1[25]*Gu1[12] + Gx1[26]*Gu1[15] + Gx1[27]*Gu1[18];
Gu2[10] = + Gx1[21]*Gu1[1] + Gx1[22]*Gu1[4] + Gx1[23]*Gu1[7] + Gx1[24]*Gu1[10] + Gx1[25]*Gu1[13] + Gx1[26]*Gu1[16] + Gx1[27]*Gu1[19];
Gu2[11] = + Gx1[21]*Gu1[2] + Gx1[22]*Gu1[5] + Gx1[23]*Gu1[8] + Gx1[24]*Gu1[11] + Gx1[25]*Gu1[14] + Gx1[26]*Gu1[17] + Gx1[27]*Gu1[20];
Gu2[12] = + Gx1[28]*Gu1[0] + Gx1[29]*Gu1[3] + Gx1[30]*Gu1[6] + Gx1[31]*Gu1[9] + Gx1[32]*Gu1[12] + Gx1[33]*Gu1[15] + Gx1[34]*Gu1[18];
Gu2[13] = + Gx1[28]*Gu1[1] + Gx1[29]*Gu1[4] + Gx1[30]*Gu1[7] + Gx1[31]*Gu1[10] + Gx1[32]*Gu1[13] + Gx1[33]*Gu1[16] + Gx1[34]*Gu1[19];
Gu2[14] = + Gx1[28]*Gu1[2] + Gx1[29]*Gu1[5] + Gx1[30]*Gu1[8] + Gx1[31]*Gu1[11] + Gx1[32]*Gu1[14] + Gx1[33]*Gu1[17] + Gx1[34]*Gu1[20];
Gu2[15] = + Gx1[35]*Gu1[0] + Gx1[36]*Gu1[3] + Gx1[37]*Gu1[6] + Gx1[38]*Gu1[9] + Gx1[39]*Gu1[12] + Gx1[40]*Gu1[15] + Gx1[41]*Gu1[18];
Gu2[16] = + Gx1[35]*Gu1[1] + Gx1[36]*Gu1[4] + Gx1[37]*Gu1[7] + Gx1[38]*Gu1[10] + Gx1[39]*Gu1[13] + Gx1[40]*Gu1[16] + Gx1[41]*Gu1[19];
Gu2[17] = + Gx1[35]*Gu1[2] + Gx1[36]*Gu1[5] + Gx1[37]*Gu1[8] + Gx1[38]*Gu1[11] + Gx1[39]*Gu1[14] + Gx1[40]*Gu1[17] + Gx1[41]*Gu1[20];
Gu2[18] = + Gx1[42]*Gu1[0] + Gx1[43]*Gu1[3] + Gx1[44]*Gu1[6] + Gx1[45]*Gu1[9] + Gx1[46]*Gu1[12] + Gx1[47]*Gu1[15] + Gx1[48]*Gu1[18];
Gu2[19] = + Gx1[42]*Gu1[1] + Gx1[43]*Gu1[4] + Gx1[44]*Gu1[7] + Gx1[45]*Gu1[10] + Gx1[46]*Gu1[13] + Gx1[47]*Gu1[16] + Gx1[48]*Gu1[19];
Gu2[20] = + Gx1[42]*Gu1[2] + Gx1[43]*Gu1[5] + Gx1[44]*Gu1[8] + Gx1[45]*Gu1[11] + Gx1[46]*Gu1[14] + Gx1[47]*Gu1[17] + Gx1[48]*Gu1[20];
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
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 900) + (iCol * 3)] += + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18];
acadoWorkspace.H[(iRow * 900) + (iCol * 3 + 1)] += + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19];
acadoWorkspace.H[(iRow * 900) + (iCol * 3 + 2)] += + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20];
acadoWorkspace.H[(iRow * 900 + 300) + (iCol * 3)] += + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18];
acadoWorkspace.H[(iRow * 900 + 300) + (iCol * 3 + 1)] += + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19];
acadoWorkspace.H[(iRow * 900 + 300) + (iCol * 3 + 2)] += + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20];
acadoWorkspace.H[(iRow * 900 + 600) + (iCol * 3)] += + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18];
acadoWorkspace.H[(iRow * 900 + 600) + (iCol * 3 + 1)] += + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19];
acadoWorkspace.H[(iRow * 900 + 600) + (iCol * 3 + 2)] += + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 900) + (iCol * 3)] = R11[0] + (real_t)1.0000000000000000e-10;
acadoWorkspace.H[(iRow * 900) + (iCol * 3 + 1)] = R11[1];
acadoWorkspace.H[(iRow * 900) + (iCol * 3 + 2)] = R11[2];
acadoWorkspace.H[(iRow * 900 + 300) + (iCol * 3)] = R11[3];
acadoWorkspace.H[(iRow * 900 + 300) + (iCol * 3 + 1)] = R11[4] + (real_t)1.0000000000000000e-10;
acadoWorkspace.H[(iRow * 900 + 300) + (iCol * 3 + 2)] = R11[5];
acadoWorkspace.H[(iRow * 900 + 600) + (iCol * 3)] = R11[6];
acadoWorkspace.H[(iRow * 900 + 600) + (iCol * 3 + 1)] = R11[7];
acadoWorkspace.H[(iRow * 900 + 600) + (iCol * 3 + 2)] = R11[8] + (real_t)1.0000000000000000e-10;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 900) + (iCol * 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 900) + (iCol * 3 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 900) + (iCol * 3 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 900 + 300) + (iCol * 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 900 + 300) + (iCol * 3 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 900 + 300) + (iCol * 3 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 900 + 600) + (iCol * 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 900 + 600) + (iCol * 3 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 900 + 600) + (iCol * 3 + 2)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 900) + (iCol * 3)] = acadoWorkspace.H[(iCol * 900) + (iRow * 3)];
acadoWorkspace.H[(iRow * 900) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 900 + 300) + (iRow * 3)];
acadoWorkspace.H[(iRow * 900) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 900 + 600) + (iRow * 3)];
acadoWorkspace.H[(iRow * 900 + 300) + (iCol * 3)] = acadoWorkspace.H[(iCol * 900) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 900 + 300) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 900 + 300) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 900 + 300) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 900 + 600) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 900 + 600) + (iCol * 3)] = acadoWorkspace.H[(iCol * 900) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 900 + 600) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 900 + 300) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 900 + 600) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 900 + 600) + (iRow * 3 + 2)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6];
dNew[1] = + Gx1[7]*dOld[0] + Gx1[8]*dOld[1] + Gx1[9]*dOld[2] + Gx1[10]*dOld[3] + Gx1[11]*dOld[4] + Gx1[12]*dOld[5] + Gx1[13]*dOld[6];
dNew[2] = + Gx1[14]*dOld[0] + Gx1[15]*dOld[1] + Gx1[16]*dOld[2] + Gx1[17]*dOld[3] + Gx1[18]*dOld[4] + Gx1[19]*dOld[5] + Gx1[20]*dOld[6];
dNew[3] = + Gx1[21]*dOld[0] + Gx1[22]*dOld[1] + Gx1[23]*dOld[2] + Gx1[24]*dOld[3] + Gx1[25]*dOld[4] + Gx1[26]*dOld[5] + Gx1[27]*dOld[6];
dNew[4] = + Gx1[28]*dOld[0] + Gx1[29]*dOld[1] + Gx1[30]*dOld[2] + Gx1[31]*dOld[3] + Gx1[32]*dOld[4] + Gx1[33]*dOld[5] + Gx1[34]*dOld[6];
dNew[5] = + Gx1[35]*dOld[0] + Gx1[36]*dOld[1] + Gx1[37]*dOld[2] + Gx1[38]*dOld[3] + Gx1[39]*dOld[4] + Gx1[40]*dOld[5] + Gx1[41]*dOld[6];
dNew[6] = + Gx1[42]*dOld[0] + Gx1[43]*dOld[1] + Gx1[44]*dOld[2] + Gx1[45]*dOld[3] + Gx1[46]*dOld[4] + Gx1[47]*dOld[5] + Gx1[48]*dOld[6];
}

void acado_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + acadoWorkspace.QN1[0]*dOld[0] + acadoWorkspace.QN1[1]*dOld[1] + acadoWorkspace.QN1[2]*dOld[2] + acadoWorkspace.QN1[3]*dOld[3] + acadoWorkspace.QN1[4]*dOld[4] + acadoWorkspace.QN1[5]*dOld[5] + acadoWorkspace.QN1[6]*dOld[6];
dNew[1] = + acadoWorkspace.QN1[7]*dOld[0] + acadoWorkspace.QN1[8]*dOld[1] + acadoWorkspace.QN1[9]*dOld[2] + acadoWorkspace.QN1[10]*dOld[3] + acadoWorkspace.QN1[11]*dOld[4] + acadoWorkspace.QN1[12]*dOld[5] + acadoWorkspace.QN1[13]*dOld[6];
dNew[2] = + acadoWorkspace.QN1[14]*dOld[0] + acadoWorkspace.QN1[15]*dOld[1] + acadoWorkspace.QN1[16]*dOld[2] + acadoWorkspace.QN1[17]*dOld[3] + acadoWorkspace.QN1[18]*dOld[4] + acadoWorkspace.QN1[19]*dOld[5] + acadoWorkspace.QN1[20]*dOld[6];
dNew[3] = + acadoWorkspace.QN1[21]*dOld[0] + acadoWorkspace.QN1[22]*dOld[1] + acadoWorkspace.QN1[23]*dOld[2] + acadoWorkspace.QN1[24]*dOld[3] + acadoWorkspace.QN1[25]*dOld[4] + acadoWorkspace.QN1[26]*dOld[5] + acadoWorkspace.QN1[27]*dOld[6];
dNew[4] = + acadoWorkspace.QN1[28]*dOld[0] + acadoWorkspace.QN1[29]*dOld[1] + acadoWorkspace.QN1[30]*dOld[2] + acadoWorkspace.QN1[31]*dOld[3] + acadoWorkspace.QN1[32]*dOld[4] + acadoWorkspace.QN1[33]*dOld[5] + acadoWorkspace.QN1[34]*dOld[6];
dNew[5] = + acadoWorkspace.QN1[35]*dOld[0] + acadoWorkspace.QN1[36]*dOld[1] + acadoWorkspace.QN1[37]*dOld[2] + acadoWorkspace.QN1[38]*dOld[3] + acadoWorkspace.QN1[39]*dOld[4] + acadoWorkspace.QN1[40]*dOld[5] + acadoWorkspace.QN1[41]*dOld[6];
dNew[6] = + acadoWorkspace.QN1[42]*dOld[0] + acadoWorkspace.QN1[43]*dOld[1] + acadoWorkspace.QN1[44]*dOld[2] + acadoWorkspace.QN1[45]*dOld[3] + acadoWorkspace.QN1[46]*dOld[4] + acadoWorkspace.QN1[47]*dOld[5] + acadoWorkspace.QN1[48]*dOld[6];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4];
RDy1[1] = + R2[5]*Dy1[0] + R2[6]*Dy1[1] + R2[7]*Dy1[2] + R2[8]*Dy1[3] + R2[9]*Dy1[4];
RDy1[2] = + R2[10]*Dy1[0] + R2[11]*Dy1[1] + R2[12]*Dy1[2] + R2[13]*Dy1[3] + R2[14]*Dy1[4];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4];
QDy1[1] = + Q2[5]*Dy1[0] + Q2[6]*Dy1[1] + Q2[7]*Dy1[2] + Q2[8]*Dy1[3] + Q2[9]*Dy1[4];
QDy1[2] = + Q2[10]*Dy1[0] + Q2[11]*Dy1[1] + Q2[12]*Dy1[2] + Q2[13]*Dy1[3] + Q2[14]*Dy1[4];
QDy1[3] = + Q2[15]*Dy1[0] + Q2[16]*Dy1[1] + Q2[17]*Dy1[2] + Q2[18]*Dy1[3] + Q2[19]*Dy1[4];
QDy1[4] = + Q2[20]*Dy1[0] + Q2[21]*Dy1[1] + Q2[22]*Dy1[2] + Q2[23]*Dy1[3] + Q2[24]*Dy1[4];
QDy1[5] = + Q2[25]*Dy1[0] + Q2[26]*Dy1[1] + Q2[27]*Dy1[2] + Q2[28]*Dy1[3] + Q2[29]*Dy1[4];
QDy1[6] = + Q2[30]*Dy1[0] + Q2[31]*Dy1[1] + Q2[32]*Dy1[2] + Q2[33]*Dy1[3] + Q2[34]*Dy1[4];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[3]*QDy1[1] + E1[6]*QDy1[2] + E1[9]*QDy1[3] + E1[12]*QDy1[4] + E1[15]*QDy1[5] + E1[18]*QDy1[6];
U1[1] += + E1[1]*QDy1[0] + E1[4]*QDy1[1] + E1[7]*QDy1[2] + E1[10]*QDy1[3] + E1[13]*QDy1[4] + E1[16]*QDy1[5] + E1[19]*QDy1[6];
U1[2] += + E1[2]*QDy1[0] + E1[5]*QDy1[1] + E1[8]*QDy1[2] + E1[11]*QDy1[3] + E1[14]*QDy1[4] + E1[17]*QDy1[5] + E1[20]*QDy1[6];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[3]*Gx1[7] + E1[6]*Gx1[14] + E1[9]*Gx1[21] + E1[12]*Gx1[28] + E1[15]*Gx1[35] + E1[18]*Gx1[42];
H101[1] += + E1[0]*Gx1[1] + E1[3]*Gx1[8] + E1[6]*Gx1[15] + E1[9]*Gx1[22] + E1[12]*Gx1[29] + E1[15]*Gx1[36] + E1[18]*Gx1[43];
H101[2] += + E1[0]*Gx1[2] + E1[3]*Gx1[9] + E1[6]*Gx1[16] + E1[9]*Gx1[23] + E1[12]*Gx1[30] + E1[15]*Gx1[37] + E1[18]*Gx1[44];
H101[3] += + E1[0]*Gx1[3] + E1[3]*Gx1[10] + E1[6]*Gx1[17] + E1[9]*Gx1[24] + E1[12]*Gx1[31] + E1[15]*Gx1[38] + E1[18]*Gx1[45];
H101[4] += + E1[0]*Gx1[4] + E1[3]*Gx1[11] + E1[6]*Gx1[18] + E1[9]*Gx1[25] + E1[12]*Gx1[32] + E1[15]*Gx1[39] + E1[18]*Gx1[46];
H101[5] += + E1[0]*Gx1[5] + E1[3]*Gx1[12] + E1[6]*Gx1[19] + E1[9]*Gx1[26] + E1[12]*Gx1[33] + E1[15]*Gx1[40] + E1[18]*Gx1[47];
H101[6] += + E1[0]*Gx1[6] + E1[3]*Gx1[13] + E1[6]*Gx1[20] + E1[9]*Gx1[27] + E1[12]*Gx1[34] + E1[15]*Gx1[41] + E1[18]*Gx1[48];
H101[7] += + E1[1]*Gx1[0] + E1[4]*Gx1[7] + E1[7]*Gx1[14] + E1[10]*Gx1[21] + E1[13]*Gx1[28] + E1[16]*Gx1[35] + E1[19]*Gx1[42];
H101[8] += + E1[1]*Gx1[1] + E1[4]*Gx1[8] + E1[7]*Gx1[15] + E1[10]*Gx1[22] + E1[13]*Gx1[29] + E1[16]*Gx1[36] + E1[19]*Gx1[43];
H101[9] += + E1[1]*Gx1[2] + E1[4]*Gx1[9] + E1[7]*Gx1[16] + E1[10]*Gx1[23] + E1[13]*Gx1[30] + E1[16]*Gx1[37] + E1[19]*Gx1[44];
H101[10] += + E1[1]*Gx1[3] + E1[4]*Gx1[10] + E1[7]*Gx1[17] + E1[10]*Gx1[24] + E1[13]*Gx1[31] + E1[16]*Gx1[38] + E1[19]*Gx1[45];
H101[11] += + E1[1]*Gx1[4] + E1[4]*Gx1[11] + E1[7]*Gx1[18] + E1[10]*Gx1[25] + E1[13]*Gx1[32] + E1[16]*Gx1[39] + E1[19]*Gx1[46];
H101[12] += + E1[1]*Gx1[5] + E1[4]*Gx1[12] + E1[7]*Gx1[19] + E1[10]*Gx1[26] + E1[13]*Gx1[33] + E1[16]*Gx1[40] + E1[19]*Gx1[47];
H101[13] += + E1[1]*Gx1[6] + E1[4]*Gx1[13] + E1[7]*Gx1[20] + E1[10]*Gx1[27] + E1[13]*Gx1[34] + E1[16]*Gx1[41] + E1[19]*Gx1[48];
H101[14] += + E1[2]*Gx1[0] + E1[5]*Gx1[7] + E1[8]*Gx1[14] + E1[11]*Gx1[21] + E1[14]*Gx1[28] + E1[17]*Gx1[35] + E1[20]*Gx1[42];
H101[15] += + E1[2]*Gx1[1] + E1[5]*Gx1[8] + E1[8]*Gx1[15] + E1[11]*Gx1[22] + E1[14]*Gx1[29] + E1[17]*Gx1[36] + E1[20]*Gx1[43];
H101[16] += + E1[2]*Gx1[2] + E1[5]*Gx1[9] + E1[8]*Gx1[16] + E1[11]*Gx1[23] + E1[14]*Gx1[30] + E1[17]*Gx1[37] + E1[20]*Gx1[44];
H101[17] += + E1[2]*Gx1[3] + E1[5]*Gx1[10] + E1[8]*Gx1[17] + E1[11]*Gx1[24] + E1[14]*Gx1[31] + E1[17]*Gx1[38] + E1[20]*Gx1[45];
H101[18] += + E1[2]*Gx1[4] + E1[5]*Gx1[11] + E1[8]*Gx1[18] + E1[11]*Gx1[25] + E1[14]*Gx1[32] + E1[17]*Gx1[39] + E1[20]*Gx1[46];
H101[19] += + E1[2]*Gx1[5] + E1[5]*Gx1[12] + E1[8]*Gx1[19] + E1[11]*Gx1[26] + E1[14]*Gx1[33] + E1[17]*Gx1[40] + E1[20]*Gx1[47];
H101[20] += + E1[2]*Gx1[6] + E1[5]*Gx1[13] + E1[8]*Gx1[20] + E1[11]*Gx1[27] + E1[14]*Gx1[34] + E1[17]*Gx1[41] + E1[20]*Gx1[48];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 21; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1] + E1[2]*U1[2];
dNew[1] += + E1[3]*U1[0] + E1[4]*U1[1] + E1[5]*U1[2];
dNew[2] += + E1[6]*U1[0] + E1[7]*U1[1] + E1[8]*U1[2];
dNew[3] += + E1[9]*U1[0] + E1[10]*U1[1] + E1[11]*U1[2];
dNew[4] += + E1[12]*U1[0] + E1[13]*U1[1] + E1[14]*U1[2];
dNew[5] += + E1[15]*U1[0] + E1[16]*U1[1] + E1[17]*U1[2];
dNew[6] += + E1[18]*U1[0] + E1[19]*U1[1] + E1[20]*U1[2];
}

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[7] + Hx[2]*Gx[14] + Hx[3]*Gx[21] + Hx[4]*Gx[28] + Hx[5]*Gx[35] + Hx[6]*Gx[42];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[8] + Hx[2]*Gx[15] + Hx[3]*Gx[22] + Hx[4]*Gx[29] + Hx[5]*Gx[36] + Hx[6]*Gx[43];
A01[2] = + Hx[0]*Gx[2] + Hx[1]*Gx[9] + Hx[2]*Gx[16] + Hx[3]*Gx[23] + Hx[4]*Gx[30] + Hx[5]*Gx[37] + Hx[6]*Gx[44];
A01[3] = + Hx[0]*Gx[3] + Hx[1]*Gx[10] + Hx[2]*Gx[17] + Hx[3]*Gx[24] + Hx[4]*Gx[31] + Hx[5]*Gx[38] + Hx[6]*Gx[45];
A01[4] = + Hx[0]*Gx[4] + Hx[1]*Gx[11] + Hx[2]*Gx[18] + Hx[3]*Gx[25] + Hx[4]*Gx[32] + Hx[5]*Gx[39] + Hx[6]*Gx[46];
A01[5] = + Hx[0]*Gx[5] + Hx[1]*Gx[12] + Hx[2]*Gx[19] + Hx[3]*Gx[26] + Hx[4]*Gx[33] + Hx[5]*Gx[40] + Hx[6]*Gx[47];
A01[6] = + Hx[0]*Gx[6] + Hx[1]*Gx[13] + Hx[2]*Gx[20] + Hx[3]*Gx[27] + Hx[4]*Gx[34] + Hx[5]*Gx[41] + Hx[6]*Gx[48];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 300) + (col * 3)] = + Hx[0]*E[0] + Hx[1]*E[3] + Hx[2]*E[6] + Hx[3]*E[9] + Hx[4]*E[12] + Hx[5]*E[15] + Hx[6]*E[18];
acadoWorkspace.A[(row * 300) + (col * 3 + 1)] = + Hx[0]*E[1] + Hx[1]*E[4] + Hx[2]*E[7] + Hx[3]*E[10] + Hx[4]*E[13] + Hx[5]*E[16] + Hx[6]*E[19];
acadoWorkspace.A[(row * 300) + (col * 3 + 2)] = + Hx[0]*E[2] + Hx[1]*E[5] + Hx[2]*E[8] + Hx[3]*E[11] + Hx[4]*E[14] + Hx[5]*E[17] + Hx[6]*E[20];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3] + Hx[4]*tmpd[4] + Hx[5]*tmpd[5] + Hx[6]*tmpd[6];
lbA[0] -= acadoWorkspace.evHxd[0];
ubA[0] -= acadoWorkspace.evHxd[0];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 7;
/* Vector of auxiliary variables; number of elements: 12. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (sin(u[1]));
a[1] = (real_t)(0.0000000000000000e+00);
a[2] = (real_t)(0.0000000000000000e+00);
a[3] = (real_t)(0.0000000000000000e+00);
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = (real_t)(0.0000000000000000e+00);
a[6] = (real_t)(0.0000000000000000e+00);
a[7] = (real_t)(0.0000000000000000e+00);
a[8] = (sin(u[1]));
a[9] = (cos(u[1]));
a[10] = (a[9]*u[0]);
a[11] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = (a[0]*u[0]);
out[1] = a[1];
out[2] = a[2];
out[3] = a[3];
out[4] = a[4];
out[5] = a[5];
out[6] = a[6];
out[7] = a[7];
out[8] = a[8];
out[9] = a[10];
out[10] = a[11];
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
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
for (lRun1 = 1; lRun1 < 100; ++lRun1)
{
acado_moveGxT( &(acadoWorkspace.evGx[ lRun1 * 49 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ lRun1 * 7-7 ]), &(acadoWorkspace.evGx[ lRun1 * 49 ]), &(acadoWorkspace.d[ lRun1 * 7 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ lRun1 * 49-49 ]), &(acadoWorkspace.evGx[ lRun1 * 49 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ lRun4 * 21 ]), &(acadoWorkspace.E[ lRun3 * 21 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun1 * 21 ]), &(acadoWorkspace.E[ lRun3 * 21 ]) );
}

for (lRun1 = 0; lRun1 < 99; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( &(acadoWorkspace.Q1[ lRun1 * 49 + 49 ]), &(acadoWorkspace.E[ lRun3 * 21 ]), &(acadoWorkspace.QE[ lRun3 * 21 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ lRun3 * 21 ]), &(acadoWorkspace.QE[ lRun3 * 21 ]) );
}

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
acado_zeroBlockH10( &(acadoWorkspace.H10[ lRun1 * 21 ]) );
for (lRun2 = lRun1; lRun2 < 100; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multQETGx( &(acadoWorkspace.QE[ lRun3 * 21 ]), &(acadoWorkspace.evGx[ lRun2 * 49 ]), &(acadoWorkspace.H10[ lRun1 * 21 ]) );
}
}

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
acado_setBlockH11_R1( lRun1, lRun1, &(acadoWorkspace.R1[ lRun1 * 9 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 100; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 21 ]), &(acadoWorkspace.QE[ lRun5 * 21 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 100; ++lRun2)
{
acado_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 100; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 21 ]), &(acadoWorkspace.QE[ lRun5 * 21 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun1, lRun2 );
}
}

acado_multQ1d( &(acadoWorkspace.Q1[ 49 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 98 ]), &(acadoWorkspace.d[ 7 ]), &(acadoWorkspace.Qd[ 7 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 147 ]), &(acadoWorkspace.d[ 14 ]), &(acadoWorkspace.Qd[ 14 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 196 ]), &(acadoWorkspace.d[ 21 ]), &(acadoWorkspace.Qd[ 21 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 245 ]), &(acadoWorkspace.d[ 28 ]), &(acadoWorkspace.Qd[ 28 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 294 ]), &(acadoWorkspace.d[ 35 ]), &(acadoWorkspace.Qd[ 35 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 343 ]), &(acadoWorkspace.d[ 42 ]), &(acadoWorkspace.Qd[ 42 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 392 ]), &(acadoWorkspace.d[ 49 ]), &(acadoWorkspace.Qd[ 49 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 441 ]), &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.Qd[ 56 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 490 ]), &(acadoWorkspace.d[ 63 ]), &(acadoWorkspace.Qd[ 63 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 539 ]), &(acadoWorkspace.d[ 70 ]), &(acadoWorkspace.Qd[ 70 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 588 ]), &(acadoWorkspace.d[ 77 ]), &(acadoWorkspace.Qd[ 77 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 637 ]), &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.Qd[ 84 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 686 ]), &(acadoWorkspace.d[ 91 ]), &(acadoWorkspace.Qd[ 91 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 735 ]), &(acadoWorkspace.d[ 98 ]), &(acadoWorkspace.Qd[ 98 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 784 ]), &(acadoWorkspace.d[ 105 ]), &(acadoWorkspace.Qd[ 105 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 833 ]), &(acadoWorkspace.d[ 112 ]), &(acadoWorkspace.Qd[ 112 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 882 ]), &(acadoWorkspace.d[ 119 ]), &(acadoWorkspace.Qd[ 119 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 931 ]), &(acadoWorkspace.d[ 126 ]), &(acadoWorkspace.Qd[ 126 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 980 ]), &(acadoWorkspace.d[ 133 ]), &(acadoWorkspace.Qd[ 133 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1029 ]), &(acadoWorkspace.d[ 140 ]), &(acadoWorkspace.Qd[ 140 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1078 ]), &(acadoWorkspace.d[ 147 ]), &(acadoWorkspace.Qd[ 147 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1127 ]), &(acadoWorkspace.d[ 154 ]), &(acadoWorkspace.Qd[ 154 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1176 ]), &(acadoWorkspace.d[ 161 ]), &(acadoWorkspace.Qd[ 161 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1225 ]), &(acadoWorkspace.d[ 168 ]), &(acadoWorkspace.Qd[ 168 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1274 ]), &(acadoWorkspace.d[ 175 ]), &(acadoWorkspace.Qd[ 175 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1323 ]), &(acadoWorkspace.d[ 182 ]), &(acadoWorkspace.Qd[ 182 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1372 ]), &(acadoWorkspace.d[ 189 ]), &(acadoWorkspace.Qd[ 189 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1421 ]), &(acadoWorkspace.d[ 196 ]), &(acadoWorkspace.Qd[ 196 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1470 ]), &(acadoWorkspace.d[ 203 ]), &(acadoWorkspace.Qd[ 203 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1519 ]), &(acadoWorkspace.d[ 210 ]), &(acadoWorkspace.Qd[ 210 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1568 ]), &(acadoWorkspace.d[ 217 ]), &(acadoWorkspace.Qd[ 217 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1617 ]), &(acadoWorkspace.d[ 224 ]), &(acadoWorkspace.Qd[ 224 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1666 ]), &(acadoWorkspace.d[ 231 ]), &(acadoWorkspace.Qd[ 231 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1715 ]), &(acadoWorkspace.d[ 238 ]), &(acadoWorkspace.Qd[ 238 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1764 ]), &(acadoWorkspace.d[ 245 ]), &(acadoWorkspace.Qd[ 245 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1813 ]), &(acadoWorkspace.d[ 252 ]), &(acadoWorkspace.Qd[ 252 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1862 ]), &(acadoWorkspace.d[ 259 ]), &(acadoWorkspace.Qd[ 259 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1911 ]), &(acadoWorkspace.d[ 266 ]), &(acadoWorkspace.Qd[ 266 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1960 ]), &(acadoWorkspace.d[ 273 ]), &(acadoWorkspace.Qd[ 273 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2009 ]), &(acadoWorkspace.d[ 280 ]), &(acadoWorkspace.Qd[ 280 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2058 ]), &(acadoWorkspace.d[ 287 ]), &(acadoWorkspace.Qd[ 287 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2107 ]), &(acadoWorkspace.d[ 294 ]), &(acadoWorkspace.Qd[ 294 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2156 ]), &(acadoWorkspace.d[ 301 ]), &(acadoWorkspace.Qd[ 301 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2205 ]), &(acadoWorkspace.d[ 308 ]), &(acadoWorkspace.Qd[ 308 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2254 ]), &(acadoWorkspace.d[ 315 ]), &(acadoWorkspace.Qd[ 315 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2303 ]), &(acadoWorkspace.d[ 322 ]), &(acadoWorkspace.Qd[ 322 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2352 ]), &(acadoWorkspace.d[ 329 ]), &(acadoWorkspace.Qd[ 329 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2401 ]), &(acadoWorkspace.d[ 336 ]), &(acadoWorkspace.Qd[ 336 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2450 ]), &(acadoWorkspace.d[ 343 ]), &(acadoWorkspace.Qd[ 343 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2499 ]), &(acadoWorkspace.d[ 350 ]), &(acadoWorkspace.Qd[ 350 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2548 ]), &(acadoWorkspace.d[ 357 ]), &(acadoWorkspace.Qd[ 357 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2597 ]), &(acadoWorkspace.d[ 364 ]), &(acadoWorkspace.Qd[ 364 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2646 ]), &(acadoWorkspace.d[ 371 ]), &(acadoWorkspace.Qd[ 371 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2695 ]), &(acadoWorkspace.d[ 378 ]), &(acadoWorkspace.Qd[ 378 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2744 ]), &(acadoWorkspace.d[ 385 ]), &(acadoWorkspace.Qd[ 385 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2793 ]), &(acadoWorkspace.d[ 392 ]), &(acadoWorkspace.Qd[ 392 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2842 ]), &(acadoWorkspace.d[ 399 ]), &(acadoWorkspace.Qd[ 399 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2891 ]), &(acadoWorkspace.d[ 406 ]), &(acadoWorkspace.Qd[ 406 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2940 ]), &(acadoWorkspace.d[ 413 ]), &(acadoWorkspace.Qd[ 413 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2989 ]), &(acadoWorkspace.d[ 420 ]), &(acadoWorkspace.Qd[ 420 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3038 ]), &(acadoWorkspace.d[ 427 ]), &(acadoWorkspace.Qd[ 427 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3087 ]), &(acadoWorkspace.d[ 434 ]), &(acadoWorkspace.Qd[ 434 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3136 ]), &(acadoWorkspace.d[ 441 ]), &(acadoWorkspace.Qd[ 441 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3185 ]), &(acadoWorkspace.d[ 448 ]), &(acadoWorkspace.Qd[ 448 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3234 ]), &(acadoWorkspace.d[ 455 ]), &(acadoWorkspace.Qd[ 455 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3283 ]), &(acadoWorkspace.d[ 462 ]), &(acadoWorkspace.Qd[ 462 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3332 ]), &(acadoWorkspace.d[ 469 ]), &(acadoWorkspace.Qd[ 469 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3381 ]), &(acadoWorkspace.d[ 476 ]), &(acadoWorkspace.Qd[ 476 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3430 ]), &(acadoWorkspace.d[ 483 ]), &(acadoWorkspace.Qd[ 483 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3479 ]), &(acadoWorkspace.d[ 490 ]), &(acadoWorkspace.Qd[ 490 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3528 ]), &(acadoWorkspace.d[ 497 ]), &(acadoWorkspace.Qd[ 497 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3577 ]), &(acadoWorkspace.d[ 504 ]), &(acadoWorkspace.Qd[ 504 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3626 ]), &(acadoWorkspace.d[ 511 ]), &(acadoWorkspace.Qd[ 511 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3675 ]), &(acadoWorkspace.d[ 518 ]), &(acadoWorkspace.Qd[ 518 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3724 ]), &(acadoWorkspace.d[ 525 ]), &(acadoWorkspace.Qd[ 525 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3773 ]), &(acadoWorkspace.d[ 532 ]), &(acadoWorkspace.Qd[ 532 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3822 ]), &(acadoWorkspace.d[ 539 ]), &(acadoWorkspace.Qd[ 539 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3871 ]), &(acadoWorkspace.d[ 546 ]), &(acadoWorkspace.Qd[ 546 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3920 ]), &(acadoWorkspace.d[ 553 ]), &(acadoWorkspace.Qd[ 553 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3969 ]), &(acadoWorkspace.d[ 560 ]), &(acadoWorkspace.Qd[ 560 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4018 ]), &(acadoWorkspace.d[ 567 ]), &(acadoWorkspace.Qd[ 567 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4067 ]), &(acadoWorkspace.d[ 574 ]), &(acadoWorkspace.Qd[ 574 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4116 ]), &(acadoWorkspace.d[ 581 ]), &(acadoWorkspace.Qd[ 581 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4165 ]), &(acadoWorkspace.d[ 588 ]), &(acadoWorkspace.Qd[ 588 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4214 ]), &(acadoWorkspace.d[ 595 ]), &(acadoWorkspace.Qd[ 595 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4263 ]), &(acadoWorkspace.d[ 602 ]), &(acadoWorkspace.Qd[ 602 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4312 ]), &(acadoWorkspace.d[ 609 ]), &(acadoWorkspace.Qd[ 609 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4361 ]), &(acadoWorkspace.d[ 616 ]), &(acadoWorkspace.Qd[ 616 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4410 ]), &(acadoWorkspace.d[ 623 ]), &(acadoWorkspace.Qd[ 623 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4459 ]), &(acadoWorkspace.d[ 630 ]), &(acadoWorkspace.Qd[ 630 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4508 ]), &(acadoWorkspace.d[ 637 ]), &(acadoWorkspace.Qd[ 637 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4557 ]), &(acadoWorkspace.d[ 644 ]), &(acadoWorkspace.Qd[ 644 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4606 ]), &(acadoWorkspace.d[ 651 ]), &(acadoWorkspace.Qd[ 651 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4655 ]), &(acadoWorkspace.d[ 658 ]), &(acadoWorkspace.Qd[ 658 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4704 ]), &(acadoWorkspace.d[ 665 ]), &(acadoWorkspace.Qd[ 665 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4753 ]), &(acadoWorkspace.d[ 672 ]), &(acadoWorkspace.Qd[ 672 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4802 ]), &(acadoWorkspace.d[ 679 ]), &(acadoWorkspace.Qd[ 679 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4851 ]), &(acadoWorkspace.d[ 686 ]), &(acadoWorkspace.Qd[ 686 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 693 ]), &(acadoWorkspace.Qd[ 693 ]) );

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 100; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_macETSlu( &(acadoWorkspace.QE[ lRun3 * 21 ]), &(acadoWorkspace.g[ lRun1 * 3 ]) );
}
}
acadoWorkspace.lb[0] = (real_t)8.0000000000000000e+00 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)8.0000000000000000e+00 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)8.0000000000000000e+00 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)8.0000000000000000e+00 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)8.0000000000000000e+00 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)8.0000000000000000e+00 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)8.0000000000000000e+00 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)8.0000000000000000e+00 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)8.0000000000000000e+00 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)8.0000000000000000e+00 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)8.0000000000000000e+00 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)8.0000000000000000e+00 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)8.0000000000000000e+00 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)8.0000000000000000e+00 - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[41];
acadoWorkspace.lb[42] = (real_t)8.0000000000000000e+00 - acadoVariables.u[42];
acadoWorkspace.lb[43] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[43];
acadoWorkspace.lb[44] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[44];
acadoWorkspace.lb[45] = (real_t)8.0000000000000000e+00 - acadoVariables.u[45];
acadoWorkspace.lb[46] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[46];
acadoWorkspace.lb[47] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[47];
acadoWorkspace.lb[48] = (real_t)8.0000000000000000e+00 - acadoVariables.u[48];
acadoWorkspace.lb[49] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[49];
acadoWorkspace.lb[50] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[50];
acadoWorkspace.lb[51] = (real_t)8.0000000000000000e+00 - acadoVariables.u[51];
acadoWorkspace.lb[52] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[52];
acadoWorkspace.lb[53] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[53];
acadoWorkspace.lb[54] = (real_t)8.0000000000000000e+00 - acadoVariables.u[54];
acadoWorkspace.lb[55] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[55];
acadoWorkspace.lb[56] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[56];
acadoWorkspace.lb[57] = (real_t)8.0000000000000000e+00 - acadoVariables.u[57];
acadoWorkspace.lb[58] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[58];
acadoWorkspace.lb[59] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[59];
acadoWorkspace.lb[60] = (real_t)8.0000000000000000e+00 - acadoVariables.u[60];
acadoWorkspace.lb[61] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[61];
acadoWorkspace.lb[62] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[62];
acadoWorkspace.lb[63] = (real_t)8.0000000000000000e+00 - acadoVariables.u[63];
acadoWorkspace.lb[64] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[64];
acadoWorkspace.lb[65] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[65];
acadoWorkspace.lb[66] = (real_t)8.0000000000000000e+00 - acadoVariables.u[66];
acadoWorkspace.lb[67] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[67];
acadoWorkspace.lb[68] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[68];
acadoWorkspace.lb[69] = (real_t)8.0000000000000000e+00 - acadoVariables.u[69];
acadoWorkspace.lb[70] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[70];
acadoWorkspace.lb[71] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[71];
acadoWorkspace.lb[72] = (real_t)8.0000000000000000e+00 - acadoVariables.u[72];
acadoWorkspace.lb[73] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[73];
acadoWorkspace.lb[74] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[74];
acadoWorkspace.lb[75] = (real_t)8.0000000000000000e+00 - acadoVariables.u[75];
acadoWorkspace.lb[76] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[76];
acadoWorkspace.lb[77] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[77];
acadoWorkspace.lb[78] = (real_t)8.0000000000000000e+00 - acadoVariables.u[78];
acadoWorkspace.lb[79] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[79];
acadoWorkspace.lb[80] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[80];
acadoWorkspace.lb[81] = (real_t)8.0000000000000000e+00 - acadoVariables.u[81];
acadoWorkspace.lb[82] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[82];
acadoWorkspace.lb[83] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[83];
acadoWorkspace.lb[84] = (real_t)8.0000000000000000e+00 - acadoVariables.u[84];
acadoWorkspace.lb[85] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[85];
acadoWorkspace.lb[86] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[86];
acadoWorkspace.lb[87] = (real_t)8.0000000000000000e+00 - acadoVariables.u[87];
acadoWorkspace.lb[88] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[88];
acadoWorkspace.lb[89] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[89];
acadoWorkspace.lb[90] = (real_t)8.0000000000000000e+00 - acadoVariables.u[90];
acadoWorkspace.lb[91] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[91];
acadoWorkspace.lb[92] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[92];
acadoWorkspace.lb[93] = (real_t)8.0000000000000000e+00 - acadoVariables.u[93];
acadoWorkspace.lb[94] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[94];
acadoWorkspace.lb[95] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[95];
acadoWorkspace.lb[96] = (real_t)8.0000000000000000e+00 - acadoVariables.u[96];
acadoWorkspace.lb[97] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[97];
acadoWorkspace.lb[98] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[98];
acadoWorkspace.lb[99] = (real_t)8.0000000000000000e+00 - acadoVariables.u[99];
acadoWorkspace.lb[100] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[100];
acadoWorkspace.lb[101] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[101];
acadoWorkspace.lb[102] = (real_t)8.0000000000000000e+00 - acadoVariables.u[102];
acadoWorkspace.lb[103] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[103];
acadoWorkspace.lb[104] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[104];
acadoWorkspace.lb[105] = (real_t)8.0000000000000000e+00 - acadoVariables.u[105];
acadoWorkspace.lb[106] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[106];
acadoWorkspace.lb[107] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[107];
acadoWorkspace.lb[108] = (real_t)8.0000000000000000e+00 - acadoVariables.u[108];
acadoWorkspace.lb[109] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[109];
acadoWorkspace.lb[110] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[110];
acadoWorkspace.lb[111] = (real_t)8.0000000000000000e+00 - acadoVariables.u[111];
acadoWorkspace.lb[112] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[112];
acadoWorkspace.lb[113] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[113];
acadoWorkspace.lb[114] = (real_t)8.0000000000000000e+00 - acadoVariables.u[114];
acadoWorkspace.lb[115] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[115];
acadoWorkspace.lb[116] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[116];
acadoWorkspace.lb[117] = (real_t)8.0000000000000000e+00 - acadoVariables.u[117];
acadoWorkspace.lb[118] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[118];
acadoWorkspace.lb[119] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[119];
acadoWorkspace.lb[120] = (real_t)8.0000000000000000e+00 - acadoVariables.u[120];
acadoWorkspace.lb[121] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[121];
acadoWorkspace.lb[122] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[122];
acadoWorkspace.lb[123] = (real_t)8.0000000000000000e+00 - acadoVariables.u[123];
acadoWorkspace.lb[124] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[124];
acadoWorkspace.lb[125] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[125];
acadoWorkspace.lb[126] = (real_t)8.0000000000000000e+00 - acadoVariables.u[126];
acadoWorkspace.lb[127] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[127];
acadoWorkspace.lb[128] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[128];
acadoWorkspace.lb[129] = (real_t)8.0000000000000000e+00 - acadoVariables.u[129];
acadoWorkspace.lb[130] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[130];
acadoWorkspace.lb[131] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[131];
acadoWorkspace.lb[132] = (real_t)8.0000000000000000e+00 - acadoVariables.u[132];
acadoWorkspace.lb[133] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[133];
acadoWorkspace.lb[134] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[134];
acadoWorkspace.lb[135] = (real_t)8.0000000000000000e+00 - acadoVariables.u[135];
acadoWorkspace.lb[136] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[136];
acadoWorkspace.lb[137] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[137];
acadoWorkspace.lb[138] = (real_t)8.0000000000000000e+00 - acadoVariables.u[138];
acadoWorkspace.lb[139] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[139];
acadoWorkspace.lb[140] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[140];
acadoWorkspace.lb[141] = (real_t)8.0000000000000000e+00 - acadoVariables.u[141];
acadoWorkspace.lb[142] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[142];
acadoWorkspace.lb[143] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[143];
acadoWorkspace.lb[144] = (real_t)8.0000000000000000e+00 - acadoVariables.u[144];
acadoWorkspace.lb[145] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[145];
acadoWorkspace.lb[146] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[146];
acadoWorkspace.lb[147] = (real_t)8.0000000000000000e+00 - acadoVariables.u[147];
acadoWorkspace.lb[148] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[148];
acadoWorkspace.lb[149] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[149];
acadoWorkspace.lb[150] = (real_t)8.0000000000000000e+00 - acadoVariables.u[150];
acadoWorkspace.lb[151] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[151];
acadoWorkspace.lb[152] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[152];
acadoWorkspace.lb[153] = (real_t)8.0000000000000000e+00 - acadoVariables.u[153];
acadoWorkspace.lb[154] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[154];
acadoWorkspace.lb[155] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[155];
acadoWorkspace.lb[156] = (real_t)8.0000000000000000e+00 - acadoVariables.u[156];
acadoWorkspace.lb[157] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[157];
acadoWorkspace.lb[158] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[158];
acadoWorkspace.lb[159] = (real_t)8.0000000000000000e+00 - acadoVariables.u[159];
acadoWorkspace.lb[160] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[160];
acadoWorkspace.lb[161] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[161];
acadoWorkspace.lb[162] = (real_t)8.0000000000000000e+00 - acadoVariables.u[162];
acadoWorkspace.lb[163] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[163];
acadoWorkspace.lb[164] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[164];
acadoWorkspace.lb[165] = (real_t)8.0000000000000000e+00 - acadoVariables.u[165];
acadoWorkspace.lb[166] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[166];
acadoWorkspace.lb[167] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[167];
acadoWorkspace.lb[168] = (real_t)8.0000000000000000e+00 - acadoVariables.u[168];
acadoWorkspace.lb[169] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[169];
acadoWorkspace.lb[170] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[170];
acadoWorkspace.lb[171] = (real_t)8.0000000000000000e+00 - acadoVariables.u[171];
acadoWorkspace.lb[172] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[172];
acadoWorkspace.lb[173] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[173];
acadoWorkspace.lb[174] = (real_t)8.0000000000000000e+00 - acadoVariables.u[174];
acadoWorkspace.lb[175] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[175];
acadoWorkspace.lb[176] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[176];
acadoWorkspace.lb[177] = (real_t)8.0000000000000000e+00 - acadoVariables.u[177];
acadoWorkspace.lb[178] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[178];
acadoWorkspace.lb[179] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[179];
acadoWorkspace.lb[180] = (real_t)8.0000000000000000e+00 - acadoVariables.u[180];
acadoWorkspace.lb[181] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[181];
acadoWorkspace.lb[182] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[182];
acadoWorkspace.lb[183] = (real_t)8.0000000000000000e+00 - acadoVariables.u[183];
acadoWorkspace.lb[184] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[184];
acadoWorkspace.lb[185] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[185];
acadoWorkspace.lb[186] = (real_t)8.0000000000000000e+00 - acadoVariables.u[186];
acadoWorkspace.lb[187] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[187];
acadoWorkspace.lb[188] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[188];
acadoWorkspace.lb[189] = (real_t)8.0000000000000000e+00 - acadoVariables.u[189];
acadoWorkspace.lb[190] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[190];
acadoWorkspace.lb[191] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[191];
acadoWorkspace.lb[192] = (real_t)8.0000000000000000e+00 - acadoVariables.u[192];
acadoWorkspace.lb[193] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[193];
acadoWorkspace.lb[194] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[194];
acadoWorkspace.lb[195] = (real_t)8.0000000000000000e+00 - acadoVariables.u[195];
acadoWorkspace.lb[196] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[196];
acadoWorkspace.lb[197] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[197];
acadoWorkspace.lb[198] = (real_t)8.0000000000000000e+00 - acadoVariables.u[198];
acadoWorkspace.lb[199] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[199];
acadoWorkspace.lb[200] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[200];
acadoWorkspace.lb[201] = (real_t)8.0000000000000000e+00 - acadoVariables.u[201];
acadoWorkspace.lb[202] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[202];
acadoWorkspace.lb[203] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[203];
acadoWorkspace.lb[204] = (real_t)8.0000000000000000e+00 - acadoVariables.u[204];
acadoWorkspace.lb[205] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[205];
acadoWorkspace.lb[206] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[206];
acadoWorkspace.lb[207] = (real_t)8.0000000000000000e+00 - acadoVariables.u[207];
acadoWorkspace.lb[208] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[208];
acadoWorkspace.lb[209] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[209];
acadoWorkspace.lb[210] = (real_t)8.0000000000000000e+00 - acadoVariables.u[210];
acadoWorkspace.lb[211] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[211];
acadoWorkspace.lb[212] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[212];
acadoWorkspace.lb[213] = (real_t)8.0000000000000000e+00 - acadoVariables.u[213];
acadoWorkspace.lb[214] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[214];
acadoWorkspace.lb[215] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[215];
acadoWorkspace.lb[216] = (real_t)8.0000000000000000e+00 - acadoVariables.u[216];
acadoWorkspace.lb[217] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[217];
acadoWorkspace.lb[218] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[218];
acadoWorkspace.lb[219] = (real_t)8.0000000000000000e+00 - acadoVariables.u[219];
acadoWorkspace.lb[220] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[220];
acadoWorkspace.lb[221] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[221];
acadoWorkspace.lb[222] = (real_t)8.0000000000000000e+00 - acadoVariables.u[222];
acadoWorkspace.lb[223] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[223];
acadoWorkspace.lb[224] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[224];
acadoWorkspace.lb[225] = (real_t)8.0000000000000000e+00 - acadoVariables.u[225];
acadoWorkspace.lb[226] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[226];
acadoWorkspace.lb[227] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[227];
acadoWorkspace.lb[228] = (real_t)8.0000000000000000e+00 - acadoVariables.u[228];
acadoWorkspace.lb[229] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[229];
acadoWorkspace.lb[230] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[230];
acadoWorkspace.lb[231] = (real_t)8.0000000000000000e+00 - acadoVariables.u[231];
acadoWorkspace.lb[232] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[232];
acadoWorkspace.lb[233] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[233];
acadoWorkspace.lb[234] = (real_t)8.0000000000000000e+00 - acadoVariables.u[234];
acadoWorkspace.lb[235] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[235];
acadoWorkspace.lb[236] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[236];
acadoWorkspace.lb[237] = (real_t)8.0000000000000000e+00 - acadoVariables.u[237];
acadoWorkspace.lb[238] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[238];
acadoWorkspace.lb[239] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[239];
acadoWorkspace.lb[240] = (real_t)8.0000000000000000e+00 - acadoVariables.u[240];
acadoWorkspace.lb[241] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[241];
acadoWorkspace.lb[242] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[242];
acadoWorkspace.lb[243] = (real_t)8.0000000000000000e+00 - acadoVariables.u[243];
acadoWorkspace.lb[244] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[244];
acadoWorkspace.lb[245] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[245];
acadoWorkspace.lb[246] = (real_t)8.0000000000000000e+00 - acadoVariables.u[246];
acadoWorkspace.lb[247] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[247];
acadoWorkspace.lb[248] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[248];
acadoWorkspace.lb[249] = (real_t)8.0000000000000000e+00 - acadoVariables.u[249];
acadoWorkspace.lb[250] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[250];
acadoWorkspace.lb[251] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[251];
acadoWorkspace.lb[252] = (real_t)8.0000000000000000e+00 - acadoVariables.u[252];
acadoWorkspace.lb[253] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[253];
acadoWorkspace.lb[254] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[254];
acadoWorkspace.lb[255] = (real_t)8.0000000000000000e+00 - acadoVariables.u[255];
acadoWorkspace.lb[256] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[256];
acadoWorkspace.lb[257] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[257];
acadoWorkspace.lb[258] = (real_t)8.0000000000000000e+00 - acadoVariables.u[258];
acadoWorkspace.lb[259] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[259];
acadoWorkspace.lb[260] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[260];
acadoWorkspace.lb[261] = (real_t)8.0000000000000000e+00 - acadoVariables.u[261];
acadoWorkspace.lb[262] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[262];
acadoWorkspace.lb[263] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[263];
acadoWorkspace.lb[264] = (real_t)8.0000000000000000e+00 - acadoVariables.u[264];
acadoWorkspace.lb[265] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[265];
acadoWorkspace.lb[266] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[266];
acadoWorkspace.lb[267] = (real_t)8.0000000000000000e+00 - acadoVariables.u[267];
acadoWorkspace.lb[268] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[268];
acadoWorkspace.lb[269] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[269];
acadoWorkspace.lb[270] = (real_t)8.0000000000000000e+00 - acadoVariables.u[270];
acadoWorkspace.lb[271] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[271];
acadoWorkspace.lb[272] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[272];
acadoWorkspace.lb[273] = (real_t)8.0000000000000000e+00 - acadoVariables.u[273];
acadoWorkspace.lb[274] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[274];
acadoWorkspace.lb[275] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[275];
acadoWorkspace.lb[276] = (real_t)8.0000000000000000e+00 - acadoVariables.u[276];
acadoWorkspace.lb[277] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[277];
acadoWorkspace.lb[278] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[278];
acadoWorkspace.lb[279] = (real_t)8.0000000000000000e+00 - acadoVariables.u[279];
acadoWorkspace.lb[280] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[280];
acadoWorkspace.lb[281] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[281];
acadoWorkspace.lb[282] = (real_t)8.0000000000000000e+00 - acadoVariables.u[282];
acadoWorkspace.lb[283] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[283];
acadoWorkspace.lb[284] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[284];
acadoWorkspace.lb[285] = (real_t)8.0000000000000000e+00 - acadoVariables.u[285];
acadoWorkspace.lb[286] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[286];
acadoWorkspace.lb[287] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[287];
acadoWorkspace.lb[288] = (real_t)8.0000000000000000e+00 - acadoVariables.u[288];
acadoWorkspace.lb[289] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[289];
acadoWorkspace.lb[290] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[290];
acadoWorkspace.lb[291] = (real_t)8.0000000000000000e+00 - acadoVariables.u[291];
acadoWorkspace.lb[292] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[292];
acadoWorkspace.lb[293] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[293];
acadoWorkspace.lb[294] = (real_t)8.0000000000000000e+00 - acadoVariables.u[294];
acadoWorkspace.lb[295] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[295];
acadoWorkspace.lb[296] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[296];
acadoWorkspace.lb[297] = (real_t)8.0000000000000000e+00 - acadoVariables.u[297];
acadoWorkspace.lb[298] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[298];
acadoWorkspace.lb[299] = (real_t)-7.8539816339744828e-01 - acadoVariables.u[299];
acadoWorkspace.ub[0] = (real_t)1.3000000000000000e+01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)1.0000000000000000e+12 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)7.8539816339744828e-01 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)1.3000000000000000e+01 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)1.0000000000000000e+12 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)7.8539816339744828e-01 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.3000000000000000e+01 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)1.0000000000000000e+12 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)7.8539816339744828e-01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)1.3000000000000000e+01 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+12 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)7.8539816339744828e-01 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)1.3000000000000000e+01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)1.0000000000000000e+12 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)7.8539816339744828e-01 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)1.3000000000000000e+01 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)1.0000000000000000e+12 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)7.8539816339744828e-01 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.3000000000000000e+01 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)1.0000000000000000e+12 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)7.8539816339744828e-01 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)1.3000000000000000e+01 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)1.0000000000000000e+12 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)7.8539816339744828e-01 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)1.3000000000000000e+01 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)1.0000000000000000e+12 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)7.8539816339744828e-01 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)1.3000000000000000e+01 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)1.0000000000000000e+12 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)7.8539816339744828e-01 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)1.3000000000000000e+01 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)1.0000000000000000e+12 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)7.8539816339744828e-01 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)1.3000000000000000e+01 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)1.0000000000000000e+12 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)7.8539816339744828e-01 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)1.3000000000000000e+01 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)1.0000000000000000e+12 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)7.8539816339744828e-01 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)1.3000000000000000e+01 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)1.0000000000000000e+12 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)7.8539816339744828e-01 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)1.3000000000000000e+01 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)1.0000000000000000e+12 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)7.8539816339744828e-01 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)1.3000000000000000e+01 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)1.0000000000000000e+12 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)7.8539816339744828e-01 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)1.3000000000000000e+01 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)1.0000000000000000e+12 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)7.8539816339744828e-01 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)1.3000000000000000e+01 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)1.0000000000000000e+12 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)7.8539816339744828e-01 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)1.3000000000000000e+01 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)1.0000000000000000e+12 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)7.8539816339744828e-01 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)1.3000000000000000e+01 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)1.0000000000000000e+12 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)7.8539816339744828e-01 - acadoVariables.u[59];
acadoWorkspace.ub[60] = (real_t)1.3000000000000000e+01 - acadoVariables.u[60];
acadoWorkspace.ub[61] = (real_t)1.0000000000000000e+12 - acadoVariables.u[61];
acadoWorkspace.ub[62] = (real_t)7.8539816339744828e-01 - acadoVariables.u[62];
acadoWorkspace.ub[63] = (real_t)1.3000000000000000e+01 - acadoVariables.u[63];
acadoWorkspace.ub[64] = (real_t)1.0000000000000000e+12 - acadoVariables.u[64];
acadoWorkspace.ub[65] = (real_t)7.8539816339744828e-01 - acadoVariables.u[65];
acadoWorkspace.ub[66] = (real_t)1.3000000000000000e+01 - acadoVariables.u[66];
acadoWorkspace.ub[67] = (real_t)1.0000000000000000e+12 - acadoVariables.u[67];
acadoWorkspace.ub[68] = (real_t)7.8539816339744828e-01 - acadoVariables.u[68];
acadoWorkspace.ub[69] = (real_t)1.3000000000000000e+01 - acadoVariables.u[69];
acadoWorkspace.ub[70] = (real_t)1.0000000000000000e+12 - acadoVariables.u[70];
acadoWorkspace.ub[71] = (real_t)7.8539816339744828e-01 - acadoVariables.u[71];
acadoWorkspace.ub[72] = (real_t)1.3000000000000000e+01 - acadoVariables.u[72];
acadoWorkspace.ub[73] = (real_t)1.0000000000000000e+12 - acadoVariables.u[73];
acadoWorkspace.ub[74] = (real_t)7.8539816339744828e-01 - acadoVariables.u[74];
acadoWorkspace.ub[75] = (real_t)1.3000000000000000e+01 - acadoVariables.u[75];
acadoWorkspace.ub[76] = (real_t)1.0000000000000000e+12 - acadoVariables.u[76];
acadoWorkspace.ub[77] = (real_t)7.8539816339744828e-01 - acadoVariables.u[77];
acadoWorkspace.ub[78] = (real_t)1.3000000000000000e+01 - acadoVariables.u[78];
acadoWorkspace.ub[79] = (real_t)1.0000000000000000e+12 - acadoVariables.u[79];
acadoWorkspace.ub[80] = (real_t)7.8539816339744828e-01 - acadoVariables.u[80];
acadoWorkspace.ub[81] = (real_t)1.3000000000000000e+01 - acadoVariables.u[81];
acadoWorkspace.ub[82] = (real_t)1.0000000000000000e+12 - acadoVariables.u[82];
acadoWorkspace.ub[83] = (real_t)7.8539816339744828e-01 - acadoVariables.u[83];
acadoWorkspace.ub[84] = (real_t)1.3000000000000000e+01 - acadoVariables.u[84];
acadoWorkspace.ub[85] = (real_t)1.0000000000000000e+12 - acadoVariables.u[85];
acadoWorkspace.ub[86] = (real_t)7.8539816339744828e-01 - acadoVariables.u[86];
acadoWorkspace.ub[87] = (real_t)1.3000000000000000e+01 - acadoVariables.u[87];
acadoWorkspace.ub[88] = (real_t)1.0000000000000000e+12 - acadoVariables.u[88];
acadoWorkspace.ub[89] = (real_t)7.8539816339744828e-01 - acadoVariables.u[89];
acadoWorkspace.ub[90] = (real_t)1.3000000000000000e+01 - acadoVariables.u[90];
acadoWorkspace.ub[91] = (real_t)1.0000000000000000e+12 - acadoVariables.u[91];
acadoWorkspace.ub[92] = (real_t)7.8539816339744828e-01 - acadoVariables.u[92];
acadoWorkspace.ub[93] = (real_t)1.3000000000000000e+01 - acadoVariables.u[93];
acadoWorkspace.ub[94] = (real_t)1.0000000000000000e+12 - acadoVariables.u[94];
acadoWorkspace.ub[95] = (real_t)7.8539816339744828e-01 - acadoVariables.u[95];
acadoWorkspace.ub[96] = (real_t)1.3000000000000000e+01 - acadoVariables.u[96];
acadoWorkspace.ub[97] = (real_t)1.0000000000000000e+12 - acadoVariables.u[97];
acadoWorkspace.ub[98] = (real_t)7.8539816339744828e-01 - acadoVariables.u[98];
acadoWorkspace.ub[99] = (real_t)1.3000000000000000e+01 - acadoVariables.u[99];
acadoWorkspace.ub[100] = (real_t)1.0000000000000000e+12 - acadoVariables.u[100];
acadoWorkspace.ub[101] = (real_t)7.8539816339744828e-01 - acadoVariables.u[101];
acadoWorkspace.ub[102] = (real_t)1.3000000000000000e+01 - acadoVariables.u[102];
acadoWorkspace.ub[103] = (real_t)1.0000000000000000e+12 - acadoVariables.u[103];
acadoWorkspace.ub[104] = (real_t)7.8539816339744828e-01 - acadoVariables.u[104];
acadoWorkspace.ub[105] = (real_t)1.3000000000000000e+01 - acadoVariables.u[105];
acadoWorkspace.ub[106] = (real_t)1.0000000000000000e+12 - acadoVariables.u[106];
acadoWorkspace.ub[107] = (real_t)7.8539816339744828e-01 - acadoVariables.u[107];
acadoWorkspace.ub[108] = (real_t)1.3000000000000000e+01 - acadoVariables.u[108];
acadoWorkspace.ub[109] = (real_t)1.0000000000000000e+12 - acadoVariables.u[109];
acadoWorkspace.ub[110] = (real_t)7.8539816339744828e-01 - acadoVariables.u[110];
acadoWorkspace.ub[111] = (real_t)1.3000000000000000e+01 - acadoVariables.u[111];
acadoWorkspace.ub[112] = (real_t)1.0000000000000000e+12 - acadoVariables.u[112];
acadoWorkspace.ub[113] = (real_t)7.8539816339744828e-01 - acadoVariables.u[113];
acadoWorkspace.ub[114] = (real_t)1.3000000000000000e+01 - acadoVariables.u[114];
acadoWorkspace.ub[115] = (real_t)1.0000000000000000e+12 - acadoVariables.u[115];
acadoWorkspace.ub[116] = (real_t)7.8539816339744828e-01 - acadoVariables.u[116];
acadoWorkspace.ub[117] = (real_t)1.3000000000000000e+01 - acadoVariables.u[117];
acadoWorkspace.ub[118] = (real_t)1.0000000000000000e+12 - acadoVariables.u[118];
acadoWorkspace.ub[119] = (real_t)7.8539816339744828e-01 - acadoVariables.u[119];
acadoWorkspace.ub[120] = (real_t)1.3000000000000000e+01 - acadoVariables.u[120];
acadoWorkspace.ub[121] = (real_t)1.0000000000000000e+12 - acadoVariables.u[121];
acadoWorkspace.ub[122] = (real_t)7.8539816339744828e-01 - acadoVariables.u[122];
acadoWorkspace.ub[123] = (real_t)1.3000000000000000e+01 - acadoVariables.u[123];
acadoWorkspace.ub[124] = (real_t)1.0000000000000000e+12 - acadoVariables.u[124];
acadoWorkspace.ub[125] = (real_t)7.8539816339744828e-01 - acadoVariables.u[125];
acadoWorkspace.ub[126] = (real_t)1.3000000000000000e+01 - acadoVariables.u[126];
acadoWorkspace.ub[127] = (real_t)1.0000000000000000e+12 - acadoVariables.u[127];
acadoWorkspace.ub[128] = (real_t)7.8539816339744828e-01 - acadoVariables.u[128];
acadoWorkspace.ub[129] = (real_t)1.3000000000000000e+01 - acadoVariables.u[129];
acadoWorkspace.ub[130] = (real_t)1.0000000000000000e+12 - acadoVariables.u[130];
acadoWorkspace.ub[131] = (real_t)7.8539816339744828e-01 - acadoVariables.u[131];
acadoWorkspace.ub[132] = (real_t)1.3000000000000000e+01 - acadoVariables.u[132];
acadoWorkspace.ub[133] = (real_t)1.0000000000000000e+12 - acadoVariables.u[133];
acadoWorkspace.ub[134] = (real_t)7.8539816339744828e-01 - acadoVariables.u[134];
acadoWorkspace.ub[135] = (real_t)1.3000000000000000e+01 - acadoVariables.u[135];
acadoWorkspace.ub[136] = (real_t)1.0000000000000000e+12 - acadoVariables.u[136];
acadoWorkspace.ub[137] = (real_t)7.8539816339744828e-01 - acadoVariables.u[137];
acadoWorkspace.ub[138] = (real_t)1.3000000000000000e+01 - acadoVariables.u[138];
acadoWorkspace.ub[139] = (real_t)1.0000000000000000e+12 - acadoVariables.u[139];
acadoWorkspace.ub[140] = (real_t)7.8539816339744828e-01 - acadoVariables.u[140];
acadoWorkspace.ub[141] = (real_t)1.3000000000000000e+01 - acadoVariables.u[141];
acadoWorkspace.ub[142] = (real_t)1.0000000000000000e+12 - acadoVariables.u[142];
acadoWorkspace.ub[143] = (real_t)7.8539816339744828e-01 - acadoVariables.u[143];
acadoWorkspace.ub[144] = (real_t)1.3000000000000000e+01 - acadoVariables.u[144];
acadoWorkspace.ub[145] = (real_t)1.0000000000000000e+12 - acadoVariables.u[145];
acadoWorkspace.ub[146] = (real_t)7.8539816339744828e-01 - acadoVariables.u[146];
acadoWorkspace.ub[147] = (real_t)1.3000000000000000e+01 - acadoVariables.u[147];
acadoWorkspace.ub[148] = (real_t)1.0000000000000000e+12 - acadoVariables.u[148];
acadoWorkspace.ub[149] = (real_t)7.8539816339744828e-01 - acadoVariables.u[149];
acadoWorkspace.ub[150] = (real_t)1.3000000000000000e+01 - acadoVariables.u[150];
acadoWorkspace.ub[151] = (real_t)1.0000000000000000e+12 - acadoVariables.u[151];
acadoWorkspace.ub[152] = (real_t)7.8539816339744828e-01 - acadoVariables.u[152];
acadoWorkspace.ub[153] = (real_t)1.3000000000000000e+01 - acadoVariables.u[153];
acadoWorkspace.ub[154] = (real_t)1.0000000000000000e+12 - acadoVariables.u[154];
acadoWorkspace.ub[155] = (real_t)7.8539816339744828e-01 - acadoVariables.u[155];
acadoWorkspace.ub[156] = (real_t)1.3000000000000000e+01 - acadoVariables.u[156];
acadoWorkspace.ub[157] = (real_t)1.0000000000000000e+12 - acadoVariables.u[157];
acadoWorkspace.ub[158] = (real_t)7.8539816339744828e-01 - acadoVariables.u[158];
acadoWorkspace.ub[159] = (real_t)1.3000000000000000e+01 - acadoVariables.u[159];
acadoWorkspace.ub[160] = (real_t)1.0000000000000000e+12 - acadoVariables.u[160];
acadoWorkspace.ub[161] = (real_t)7.8539816339744828e-01 - acadoVariables.u[161];
acadoWorkspace.ub[162] = (real_t)1.3000000000000000e+01 - acadoVariables.u[162];
acadoWorkspace.ub[163] = (real_t)1.0000000000000000e+12 - acadoVariables.u[163];
acadoWorkspace.ub[164] = (real_t)7.8539816339744828e-01 - acadoVariables.u[164];
acadoWorkspace.ub[165] = (real_t)1.3000000000000000e+01 - acadoVariables.u[165];
acadoWorkspace.ub[166] = (real_t)1.0000000000000000e+12 - acadoVariables.u[166];
acadoWorkspace.ub[167] = (real_t)7.8539816339744828e-01 - acadoVariables.u[167];
acadoWorkspace.ub[168] = (real_t)1.3000000000000000e+01 - acadoVariables.u[168];
acadoWorkspace.ub[169] = (real_t)1.0000000000000000e+12 - acadoVariables.u[169];
acadoWorkspace.ub[170] = (real_t)7.8539816339744828e-01 - acadoVariables.u[170];
acadoWorkspace.ub[171] = (real_t)1.3000000000000000e+01 - acadoVariables.u[171];
acadoWorkspace.ub[172] = (real_t)1.0000000000000000e+12 - acadoVariables.u[172];
acadoWorkspace.ub[173] = (real_t)7.8539816339744828e-01 - acadoVariables.u[173];
acadoWorkspace.ub[174] = (real_t)1.3000000000000000e+01 - acadoVariables.u[174];
acadoWorkspace.ub[175] = (real_t)1.0000000000000000e+12 - acadoVariables.u[175];
acadoWorkspace.ub[176] = (real_t)7.8539816339744828e-01 - acadoVariables.u[176];
acadoWorkspace.ub[177] = (real_t)1.3000000000000000e+01 - acadoVariables.u[177];
acadoWorkspace.ub[178] = (real_t)1.0000000000000000e+12 - acadoVariables.u[178];
acadoWorkspace.ub[179] = (real_t)7.8539816339744828e-01 - acadoVariables.u[179];
acadoWorkspace.ub[180] = (real_t)1.3000000000000000e+01 - acadoVariables.u[180];
acadoWorkspace.ub[181] = (real_t)1.0000000000000000e+12 - acadoVariables.u[181];
acadoWorkspace.ub[182] = (real_t)7.8539816339744828e-01 - acadoVariables.u[182];
acadoWorkspace.ub[183] = (real_t)1.3000000000000000e+01 - acadoVariables.u[183];
acadoWorkspace.ub[184] = (real_t)1.0000000000000000e+12 - acadoVariables.u[184];
acadoWorkspace.ub[185] = (real_t)7.8539816339744828e-01 - acadoVariables.u[185];
acadoWorkspace.ub[186] = (real_t)1.3000000000000000e+01 - acadoVariables.u[186];
acadoWorkspace.ub[187] = (real_t)1.0000000000000000e+12 - acadoVariables.u[187];
acadoWorkspace.ub[188] = (real_t)7.8539816339744828e-01 - acadoVariables.u[188];
acadoWorkspace.ub[189] = (real_t)1.3000000000000000e+01 - acadoVariables.u[189];
acadoWorkspace.ub[190] = (real_t)1.0000000000000000e+12 - acadoVariables.u[190];
acadoWorkspace.ub[191] = (real_t)7.8539816339744828e-01 - acadoVariables.u[191];
acadoWorkspace.ub[192] = (real_t)1.3000000000000000e+01 - acadoVariables.u[192];
acadoWorkspace.ub[193] = (real_t)1.0000000000000000e+12 - acadoVariables.u[193];
acadoWorkspace.ub[194] = (real_t)7.8539816339744828e-01 - acadoVariables.u[194];
acadoWorkspace.ub[195] = (real_t)1.3000000000000000e+01 - acadoVariables.u[195];
acadoWorkspace.ub[196] = (real_t)1.0000000000000000e+12 - acadoVariables.u[196];
acadoWorkspace.ub[197] = (real_t)7.8539816339744828e-01 - acadoVariables.u[197];
acadoWorkspace.ub[198] = (real_t)1.3000000000000000e+01 - acadoVariables.u[198];
acadoWorkspace.ub[199] = (real_t)1.0000000000000000e+12 - acadoVariables.u[199];
acadoWorkspace.ub[200] = (real_t)7.8539816339744828e-01 - acadoVariables.u[200];
acadoWorkspace.ub[201] = (real_t)1.3000000000000000e+01 - acadoVariables.u[201];
acadoWorkspace.ub[202] = (real_t)1.0000000000000000e+12 - acadoVariables.u[202];
acadoWorkspace.ub[203] = (real_t)7.8539816339744828e-01 - acadoVariables.u[203];
acadoWorkspace.ub[204] = (real_t)1.3000000000000000e+01 - acadoVariables.u[204];
acadoWorkspace.ub[205] = (real_t)1.0000000000000000e+12 - acadoVariables.u[205];
acadoWorkspace.ub[206] = (real_t)7.8539816339744828e-01 - acadoVariables.u[206];
acadoWorkspace.ub[207] = (real_t)1.3000000000000000e+01 - acadoVariables.u[207];
acadoWorkspace.ub[208] = (real_t)1.0000000000000000e+12 - acadoVariables.u[208];
acadoWorkspace.ub[209] = (real_t)7.8539816339744828e-01 - acadoVariables.u[209];
acadoWorkspace.ub[210] = (real_t)1.3000000000000000e+01 - acadoVariables.u[210];
acadoWorkspace.ub[211] = (real_t)1.0000000000000000e+12 - acadoVariables.u[211];
acadoWorkspace.ub[212] = (real_t)7.8539816339744828e-01 - acadoVariables.u[212];
acadoWorkspace.ub[213] = (real_t)1.3000000000000000e+01 - acadoVariables.u[213];
acadoWorkspace.ub[214] = (real_t)1.0000000000000000e+12 - acadoVariables.u[214];
acadoWorkspace.ub[215] = (real_t)7.8539816339744828e-01 - acadoVariables.u[215];
acadoWorkspace.ub[216] = (real_t)1.3000000000000000e+01 - acadoVariables.u[216];
acadoWorkspace.ub[217] = (real_t)1.0000000000000000e+12 - acadoVariables.u[217];
acadoWorkspace.ub[218] = (real_t)7.8539816339744828e-01 - acadoVariables.u[218];
acadoWorkspace.ub[219] = (real_t)1.3000000000000000e+01 - acadoVariables.u[219];
acadoWorkspace.ub[220] = (real_t)1.0000000000000000e+12 - acadoVariables.u[220];
acadoWorkspace.ub[221] = (real_t)7.8539816339744828e-01 - acadoVariables.u[221];
acadoWorkspace.ub[222] = (real_t)1.3000000000000000e+01 - acadoVariables.u[222];
acadoWorkspace.ub[223] = (real_t)1.0000000000000000e+12 - acadoVariables.u[223];
acadoWorkspace.ub[224] = (real_t)7.8539816339744828e-01 - acadoVariables.u[224];
acadoWorkspace.ub[225] = (real_t)1.3000000000000000e+01 - acadoVariables.u[225];
acadoWorkspace.ub[226] = (real_t)1.0000000000000000e+12 - acadoVariables.u[226];
acadoWorkspace.ub[227] = (real_t)7.8539816339744828e-01 - acadoVariables.u[227];
acadoWorkspace.ub[228] = (real_t)1.3000000000000000e+01 - acadoVariables.u[228];
acadoWorkspace.ub[229] = (real_t)1.0000000000000000e+12 - acadoVariables.u[229];
acadoWorkspace.ub[230] = (real_t)7.8539816339744828e-01 - acadoVariables.u[230];
acadoWorkspace.ub[231] = (real_t)1.3000000000000000e+01 - acadoVariables.u[231];
acadoWorkspace.ub[232] = (real_t)1.0000000000000000e+12 - acadoVariables.u[232];
acadoWorkspace.ub[233] = (real_t)7.8539816339744828e-01 - acadoVariables.u[233];
acadoWorkspace.ub[234] = (real_t)1.3000000000000000e+01 - acadoVariables.u[234];
acadoWorkspace.ub[235] = (real_t)1.0000000000000000e+12 - acadoVariables.u[235];
acadoWorkspace.ub[236] = (real_t)7.8539816339744828e-01 - acadoVariables.u[236];
acadoWorkspace.ub[237] = (real_t)1.3000000000000000e+01 - acadoVariables.u[237];
acadoWorkspace.ub[238] = (real_t)1.0000000000000000e+12 - acadoVariables.u[238];
acadoWorkspace.ub[239] = (real_t)7.8539816339744828e-01 - acadoVariables.u[239];
acadoWorkspace.ub[240] = (real_t)1.3000000000000000e+01 - acadoVariables.u[240];
acadoWorkspace.ub[241] = (real_t)1.0000000000000000e+12 - acadoVariables.u[241];
acadoWorkspace.ub[242] = (real_t)7.8539816339744828e-01 - acadoVariables.u[242];
acadoWorkspace.ub[243] = (real_t)1.3000000000000000e+01 - acadoVariables.u[243];
acadoWorkspace.ub[244] = (real_t)1.0000000000000000e+12 - acadoVariables.u[244];
acadoWorkspace.ub[245] = (real_t)7.8539816339744828e-01 - acadoVariables.u[245];
acadoWorkspace.ub[246] = (real_t)1.3000000000000000e+01 - acadoVariables.u[246];
acadoWorkspace.ub[247] = (real_t)1.0000000000000000e+12 - acadoVariables.u[247];
acadoWorkspace.ub[248] = (real_t)7.8539816339744828e-01 - acadoVariables.u[248];
acadoWorkspace.ub[249] = (real_t)1.3000000000000000e+01 - acadoVariables.u[249];
acadoWorkspace.ub[250] = (real_t)1.0000000000000000e+12 - acadoVariables.u[250];
acadoWorkspace.ub[251] = (real_t)7.8539816339744828e-01 - acadoVariables.u[251];
acadoWorkspace.ub[252] = (real_t)1.3000000000000000e+01 - acadoVariables.u[252];
acadoWorkspace.ub[253] = (real_t)1.0000000000000000e+12 - acadoVariables.u[253];
acadoWorkspace.ub[254] = (real_t)7.8539816339744828e-01 - acadoVariables.u[254];
acadoWorkspace.ub[255] = (real_t)1.3000000000000000e+01 - acadoVariables.u[255];
acadoWorkspace.ub[256] = (real_t)1.0000000000000000e+12 - acadoVariables.u[256];
acadoWorkspace.ub[257] = (real_t)7.8539816339744828e-01 - acadoVariables.u[257];
acadoWorkspace.ub[258] = (real_t)1.3000000000000000e+01 - acadoVariables.u[258];
acadoWorkspace.ub[259] = (real_t)1.0000000000000000e+12 - acadoVariables.u[259];
acadoWorkspace.ub[260] = (real_t)7.8539816339744828e-01 - acadoVariables.u[260];
acadoWorkspace.ub[261] = (real_t)1.3000000000000000e+01 - acadoVariables.u[261];
acadoWorkspace.ub[262] = (real_t)1.0000000000000000e+12 - acadoVariables.u[262];
acadoWorkspace.ub[263] = (real_t)7.8539816339744828e-01 - acadoVariables.u[263];
acadoWorkspace.ub[264] = (real_t)1.3000000000000000e+01 - acadoVariables.u[264];
acadoWorkspace.ub[265] = (real_t)1.0000000000000000e+12 - acadoVariables.u[265];
acadoWorkspace.ub[266] = (real_t)7.8539816339744828e-01 - acadoVariables.u[266];
acadoWorkspace.ub[267] = (real_t)1.3000000000000000e+01 - acadoVariables.u[267];
acadoWorkspace.ub[268] = (real_t)1.0000000000000000e+12 - acadoVariables.u[268];
acadoWorkspace.ub[269] = (real_t)7.8539816339744828e-01 - acadoVariables.u[269];
acadoWorkspace.ub[270] = (real_t)1.3000000000000000e+01 - acadoVariables.u[270];
acadoWorkspace.ub[271] = (real_t)1.0000000000000000e+12 - acadoVariables.u[271];
acadoWorkspace.ub[272] = (real_t)7.8539816339744828e-01 - acadoVariables.u[272];
acadoWorkspace.ub[273] = (real_t)1.3000000000000000e+01 - acadoVariables.u[273];
acadoWorkspace.ub[274] = (real_t)1.0000000000000000e+12 - acadoVariables.u[274];
acadoWorkspace.ub[275] = (real_t)7.8539816339744828e-01 - acadoVariables.u[275];
acadoWorkspace.ub[276] = (real_t)1.3000000000000000e+01 - acadoVariables.u[276];
acadoWorkspace.ub[277] = (real_t)1.0000000000000000e+12 - acadoVariables.u[277];
acadoWorkspace.ub[278] = (real_t)7.8539816339744828e-01 - acadoVariables.u[278];
acadoWorkspace.ub[279] = (real_t)1.3000000000000000e+01 - acadoVariables.u[279];
acadoWorkspace.ub[280] = (real_t)1.0000000000000000e+12 - acadoVariables.u[280];
acadoWorkspace.ub[281] = (real_t)7.8539816339744828e-01 - acadoVariables.u[281];
acadoWorkspace.ub[282] = (real_t)1.3000000000000000e+01 - acadoVariables.u[282];
acadoWorkspace.ub[283] = (real_t)1.0000000000000000e+12 - acadoVariables.u[283];
acadoWorkspace.ub[284] = (real_t)7.8539816339744828e-01 - acadoVariables.u[284];
acadoWorkspace.ub[285] = (real_t)1.3000000000000000e+01 - acadoVariables.u[285];
acadoWorkspace.ub[286] = (real_t)1.0000000000000000e+12 - acadoVariables.u[286];
acadoWorkspace.ub[287] = (real_t)7.8539816339744828e-01 - acadoVariables.u[287];
acadoWorkspace.ub[288] = (real_t)1.3000000000000000e+01 - acadoVariables.u[288];
acadoWorkspace.ub[289] = (real_t)1.0000000000000000e+12 - acadoVariables.u[289];
acadoWorkspace.ub[290] = (real_t)7.8539816339744828e-01 - acadoVariables.u[290];
acadoWorkspace.ub[291] = (real_t)1.3000000000000000e+01 - acadoVariables.u[291];
acadoWorkspace.ub[292] = (real_t)1.0000000000000000e+12 - acadoVariables.u[292];
acadoWorkspace.ub[293] = (real_t)7.8539816339744828e-01 - acadoVariables.u[293];
acadoWorkspace.ub[294] = (real_t)1.3000000000000000e+01 - acadoVariables.u[294];
acadoWorkspace.ub[295] = (real_t)1.0000000000000000e+12 - acadoVariables.u[295];
acadoWorkspace.ub[296] = (real_t)7.8539816339744828e-01 - acadoVariables.u[296];
acadoWorkspace.ub[297] = (real_t)1.3000000000000000e+01 - acadoVariables.u[297];
acadoWorkspace.ub[298] = (real_t)1.0000000000000000e+12 - acadoVariables.u[298];
acadoWorkspace.ub[299] = (real_t)7.8539816339744828e-01 - acadoVariables.u[299];

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 7];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 7 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 7 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 7 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.x[lRun1 * 7 + 4];
acadoWorkspace.conValueIn[5] = acadoVariables.x[lRun1 * 7 + 5];
acadoWorkspace.conValueIn[6] = acadoVariables.x[lRun1 * 7 + 6];
acadoWorkspace.conValueIn[7] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.conValueIn[8] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.conValueIn[9] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.conValueIn[10] = acadoVariables.od[lRun1 * 9];
acadoWorkspace.conValueIn[11] = acadoVariables.od[lRun1 * 9 + 1];
acadoWorkspace.conValueIn[12] = acadoVariables.od[lRun1 * 9 + 2];
acadoWorkspace.conValueIn[13] = acadoVariables.od[lRun1 * 9 + 3];
acadoWorkspace.conValueIn[14] = acadoVariables.od[lRun1 * 9 + 4];
acadoWorkspace.conValueIn[15] = acadoVariables.od[lRun1 * 9 + 5];
acadoWorkspace.conValueIn[16] = acadoVariables.od[lRun1 * 9 + 6];
acadoWorkspace.conValueIn[17] = acadoVariables.od[lRun1 * 9 + 7];
acadoWorkspace.conValueIn[18] = acadoVariables.od[lRun1 * 9 + 8];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1] = acadoWorkspace.conValueOut[0];

acadoWorkspace.evHx[lRun1 * 7] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evHx[lRun1 * 7 + 1] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun1 * 7 + 2] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun1 * 7 + 3] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 7 + 4] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 7 + 5] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 7 + 6] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHu[lRun1 * 3] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHu[lRun1 * 3 + 1] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHu[lRun1 * 3 + 2] = acadoWorkspace.conValueOut[10];
}

acadoWorkspace.A01[0] = acadoWorkspace.evHx[0];
acadoWorkspace.A01[1] = acadoWorkspace.evHx[1];
acadoWorkspace.A01[2] = acadoWorkspace.evHx[2];
acadoWorkspace.A01[3] = acadoWorkspace.evHx[3];
acadoWorkspace.A01[4] = acadoWorkspace.evHx[4];
acadoWorkspace.A01[5] = acadoWorkspace.evHx[5];
acadoWorkspace.A01[6] = acadoWorkspace.evHx[6];

acado_multHxC( &(acadoWorkspace.evHx[ 7 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 7 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 14 ]), &(acadoWorkspace.evGx[ 49 ]), &(acadoWorkspace.A01[ 14 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 21 ]), &(acadoWorkspace.evGx[ 98 ]), &(acadoWorkspace.A01[ 21 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 28 ]), &(acadoWorkspace.evGx[ 147 ]), &(acadoWorkspace.A01[ 28 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 35 ]), &(acadoWorkspace.evGx[ 196 ]), &(acadoWorkspace.A01[ 35 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.evGx[ 245 ]), &(acadoWorkspace.A01[ 42 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 49 ]), &(acadoWorkspace.evGx[ 294 ]), &(acadoWorkspace.A01[ 49 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.evGx[ 343 ]), &(acadoWorkspace.A01[ 56 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 63 ]), &(acadoWorkspace.evGx[ 392 ]), &(acadoWorkspace.A01[ 63 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 70 ]), &(acadoWorkspace.evGx[ 441 ]), &(acadoWorkspace.A01[ 70 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 77 ]), &(acadoWorkspace.evGx[ 490 ]), &(acadoWorkspace.A01[ 77 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.evGx[ 539 ]), &(acadoWorkspace.A01[ 84 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 91 ]), &(acadoWorkspace.evGx[ 588 ]), &(acadoWorkspace.A01[ 91 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 98 ]), &(acadoWorkspace.evGx[ 637 ]), &(acadoWorkspace.A01[ 98 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 105 ]), &(acadoWorkspace.evGx[ 686 ]), &(acadoWorkspace.A01[ 105 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 112 ]), &(acadoWorkspace.evGx[ 735 ]), &(acadoWorkspace.A01[ 112 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 119 ]), &(acadoWorkspace.evGx[ 784 ]), &(acadoWorkspace.A01[ 119 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 126 ]), &(acadoWorkspace.evGx[ 833 ]), &(acadoWorkspace.A01[ 126 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 133 ]), &(acadoWorkspace.evGx[ 882 ]), &(acadoWorkspace.A01[ 133 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.evGx[ 931 ]), &(acadoWorkspace.A01[ 140 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 147 ]), &(acadoWorkspace.evGx[ 980 ]), &(acadoWorkspace.A01[ 147 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 154 ]), &(acadoWorkspace.evGx[ 1029 ]), &(acadoWorkspace.A01[ 154 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 161 ]), &(acadoWorkspace.evGx[ 1078 ]), &(acadoWorkspace.A01[ 161 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.evGx[ 1127 ]), &(acadoWorkspace.A01[ 168 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 175 ]), &(acadoWorkspace.evGx[ 1176 ]), &(acadoWorkspace.A01[ 175 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 182 ]), &(acadoWorkspace.evGx[ 1225 ]), &(acadoWorkspace.A01[ 182 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 189 ]), &(acadoWorkspace.evGx[ 1274 ]), &(acadoWorkspace.A01[ 189 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 196 ]), &(acadoWorkspace.evGx[ 1323 ]), &(acadoWorkspace.A01[ 196 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 203 ]), &(acadoWorkspace.evGx[ 1372 ]), &(acadoWorkspace.A01[ 203 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.evGx[ 1421 ]), &(acadoWorkspace.A01[ 210 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 217 ]), &(acadoWorkspace.evGx[ 1470 ]), &(acadoWorkspace.A01[ 217 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.evGx[ 1519 ]), &(acadoWorkspace.A01[ 224 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 231 ]), &(acadoWorkspace.evGx[ 1568 ]), &(acadoWorkspace.A01[ 231 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 238 ]), &(acadoWorkspace.evGx[ 1617 ]), &(acadoWorkspace.A01[ 238 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 245 ]), &(acadoWorkspace.evGx[ 1666 ]), &(acadoWorkspace.A01[ 245 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 252 ]), &(acadoWorkspace.evGx[ 1715 ]), &(acadoWorkspace.A01[ 252 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 259 ]), &(acadoWorkspace.evGx[ 1764 ]), &(acadoWorkspace.A01[ 259 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 266 ]), &(acadoWorkspace.evGx[ 1813 ]), &(acadoWorkspace.A01[ 266 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 273 ]), &(acadoWorkspace.evGx[ 1862 ]), &(acadoWorkspace.A01[ 273 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 280 ]), &(acadoWorkspace.evGx[ 1911 ]), &(acadoWorkspace.A01[ 280 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 287 ]), &(acadoWorkspace.evGx[ 1960 ]), &(acadoWorkspace.A01[ 287 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 294 ]), &(acadoWorkspace.evGx[ 2009 ]), &(acadoWorkspace.A01[ 294 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 301 ]), &(acadoWorkspace.evGx[ 2058 ]), &(acadoWorkspace.A01[ 301 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 308 ]), &(acadoWorkspace.evGx[ 2107 ]), &(acadoWorkspace.A01[ 308 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 315 ]), &(acadoWorkspace.evGx[ 2156 ]), &(acadoWorkspace.A01[ 315 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 322 ]), &(acadoWorkspace.evGx[ 2205 ]), &(acadoWorkspace.A01[ 322 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 329 ]), &(acadoWorkspace.evGx[ 2254 ]), &(acadoWorkspace.A01[ 329 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 336 ]), &(acadoWorkspace.evGx[ 2303 ]), &(acadoWorkspace.A01[ 336 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 343 ]), &(acadoWorkspace.evGx[ 2352 ]), &(acadoWorkspace.A01[ 343 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 350 ]), &(acadoWorkspace.evGx[ 2401 ]), &(acadoWorkspace.A01[ 350 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 357 ]), &(acadoWorkspace.evGx[ 2450 ]), &(acadoWorkspace.A01[ 357 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 364 ]), &(acadoWorkspace.evGx[ 2499 ]), &(acadoWorkspace.A01[ 364 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 371 ]), &(acadoWorkspace.evGx[ 2548 ]), &(acadoWorkspace.A01[ 371 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 378 ]), &(acadoWorkspace.evGx[ 2597 ]), &(acadoWorkspace.A01[ 378 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 385 ]), &(acadoWorkspace.evGx[ 2646 ]), &(acadoWorkspace.A01[ 385 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 392 ]), &(acadoWorkspace.evGx[ 2695 ]), &(acadoWorkspace.A01[ 392 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 399 ]), &(acadoWorkspace.evGx[ 2744 ]), &(acadoWorkspace.A01[ 399 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 406 ]), &(acadoWorkspace.evGx[ 2793 ]), &(acadoWorkspace.A01[ 406 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 413 ]), &(acadoWorkspace.evGx[ 2842 ]), &(acadoWorkspace.A01[ 413 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 420 ]), &(acadoWorkspace.evGx[ 2891 ]), &(acadoWorkspace.A01[ 420 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 427 ]), &(acadoWorkspace.evGx[ 2940 ]), &(acadoWorkspace.A01[ 427 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 434 ]), &(acadoWorkspace.evGx[ 2989 ]), &(acadoWorkspace.A01[ 434 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 441 ]), &(acadoWorkspace.evGx[ 3038 ]), &(acadoWorkspace.A01[ 441 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.evGx[ 3087 ]), &(acadoWorkspace.A01[ 448 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 455 ]), &(acadoWorkspace.evGx[ 3136 ]), &(acadoWorkspace.A01[ 455 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 462 ]), &(acadoWorkspace.evGx[ 3185 ]), &(acadoWorkspace.A01[ 462 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 469 ]), &(acadoWorkspace.evGx[ 3234 ]), &(acadoWorkspace.A01[ 469 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 476 ]), &(acadoWorkspace.evGx[ 3283 ]), &(acadoWorkspace.A01[ 476 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 483 ]), &(acadoWorkspace.evGx[ 3332 ]), &(acadoWorkspace.A01[ 483 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 490 ]), &(acadoWorkspace.evGx[ 3381 ]), &(acadoWorkspace.A01[ 490 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 497 ]), &(acadoWorkspace.evGx[ 3430 ]), &(acadoWorkspace.A01[ 497 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 504 ]), &(acadoWorkspace.evGx[ 3479 ]), &(acadoWorkspace.A01[ 504 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 511 ]), &(acadoWorkspace.evGx[ 3528 ]), &(acadoWorkspace.A01[ 511 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 518 ]), &(acadoWorkspace.evGx[ 3577 ]), &(acadoWorkspace.A01[ 518 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 525 ]), &(acadoWorkspace.evGx[ 3626 ]), &(acadoWorkspace.A01[ 525 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 532 ]), &(acadoWorkspace.evGx[ 3675 ]), &(acadoWorkspace.A01[ 532 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 539 ]), &(acadoWorkspace.evGx[ 3724 ]), &(acadoWorkspace.A01[ 539 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 546 ]), &(acadoWorkspace.evGx[ 3773 ]), &(acadoWorkspace.A01[ 546 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 553 ]), &(acadoWorkspace.evGx[ 3822 ]), &(acadoWorkspace.A01[ 553 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 560 ]), &(acadoWorkspace.evGx[ 3871 ]), &(acadoWorkspace.A01[ 560 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 567 ]), &(acadoWorkspace.evGx[ 3920 ]), &(acadoWorkspace.A01[ 567 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 574 ]), &(acadoWorkspace.evGx[ 3969 ]), &(acadoWorkspace.A01[ 574 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 581 ]), &(acadoWorkspace.evGx[ 4018 ]), &(acadoWorkspace.A01[ 581 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 588 ]), &(acadoWorkspace.evGx[ 4067 ]), &(acadoWorkspace.A01[ 588 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 595 ]), &(acadoWorkspace.evGx[ 4116 ]), &(acadoWorkspace.A01[ 595 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 602 ]), &(acadoWorkspace.evGx[ 4165 ]), &(acadoWorkspace.A01[ 602 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 609 ]), &(acadoWorkspace.evGx[ 4214 ]), &(acadoWorkspace.A01[ 609 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 616 ]), &(acadoWorkspace.evGx[ 4263 ]), &(acadoWorkspace.A01[ 616 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 623 ]), &(acadoWorkspace.evGx[ 4312 ]), &(acadoWorkspace.A01[ 623 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 630 ]), &(acadoWorkspace.evGx[ 4361 ]), &(acadoWorkspace.A01[ 630 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 637 ]), &(acadoWorkspace.evGx[ 4410 ]), &(acadoWorkspace.A01[ 637 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 644 ]), &(acadoWorkspace.evGx[ 4459 ]), &(acadoWorkspace.A01[ 644 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 651 ]), &(acadoWorkspace.evGx[ 4508 ]), &(acadoWorkspace.A01[ 651 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 658 ]), &(acadoWorkspace.evGx[ 4557 ]), &(acadoWorkspace.A01[ 658 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 665 ]), &(acadoWorkspace.evGx[ 4606 ]), &(acadoWorkspace.A01[ 665 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 672 ]), &(acadoWorkspace.evGx[ 4655 ]), &(acadoWorkspace.A01[ 672 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 679 ]), &(acadoWorkspace.evGx[ 4704 ]), &(acadoWorkspace.A01[ 679 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 686 ]), &(acadoWorkspace.evGx[ 4753 ]), &(acadoWorkspace.A01[ 686 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 693 ]), &(acadoWorkspace.evGx[ 4802 ]), &(acadoWorkspace.A01[ 693 ]) );

for (lRun2 = 0; lRun2 < 99; ++lRun2)
{
for (lRun3 = 0; lRun3 < lRun2 + 1; ++lRun3)
{
lRun4 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun3);
lRun5 = lRun2 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun2 * 7 + 7 ]), &(acadoWorkspace.E[ lRun4 * 21 ]), lRun5, lRun3 );
}
}

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[2] = acadoWorkspace.evHu[2];
acadoWorkspace.A[303] = acadoWorkspace.evHu[3];
acadoWorkspace.A[304] = acadoWorkspace.evHu[4];
acadoWorkspace.A[305] = acadoWorkspace.evHu[5];
acadoWorkspace.A[606] = acadoWorkspace.evHu[6];
acadoWorkspace.A[607] = acadoWorkspace.evHu[7];
acadoWorkspace.A[608] = acadoWorkspace.evHu[8];
acadoWorkspace.A[909] = acadoWorkspace.evHu[9];
acadoWorkspace.A[910] = acadoWorkspace.evHu[10];
acadoWorkspace.A[911] = acadoWorkspace.evHu[11];
acadoWorkspace.A[1212] = acadoWorkspace.evHu[12];
acadoWorkspace.A[1213] = acadoWorkspace.evHu[13];
acadoWorkspace.A[1214] = acadoWorkspace.evHu[14];
acadoWorkspace.A[1515] = acadoWorkspace.evHu[15];
acadoWorkspace.A[1516] = acadoWorkspace.evHu[16];
acadoWorkspace.A[1517] = acadoWorkspace.evHu[17];
acadoWorkspace.A[1818] = acadoWorkspace.evHu[18];
acadoWorkspace.A[1819] = acadoWorkspace.evHu[19];
acadoWorkspace.A[1820] = acadoWorkspace.evHu[20];
acadoWorkspace.A[2121] = acadoWorkspace.evHu[21];
acadoWorkspace.A[2122] = acadoWorkspace.evHu[22];
acadoWorkspace.A[2123] = acadoWorkspace.evHu[23];
acadoWorkspace.A[2424] = acadoWorkspace.evHu[24];
acadoWorkspace.A[2425] = acadoWorkspace.evHu[25];
acadoWorkspace.A[2426] = acadoWorkspace.evHu[26];
acadoWorkspace.A[2727] = acadoWorkspace.evHu[27];
acadoWorkspace.A[2728] = acadoWorkspace.evHu[28];
acadoWorkspace.A[2729] = acadoWorkspace.evHu[29];
acadoWorkspace.A[3030] = acadoWorkspace.evHu[30];
acadoWorkspace.A[3031] = acadoWorkspace.evHu[31];
acadoWorkspace.A[3032] = acadoWorkspace.evHu[32];
acadoWorkspace.A[3333] = acadoWorkspace.evHu[33];
acadoWorkspace.A[3334] = acadoWorkspace.evHu[34];
acadoWorkspace.A[3335] = acadoWorkspace.evHu[35];
acadoWorkspace.A[3636] = acadoWorkspace.evHu[36];
acadoWorkspace.A[3637] = acadoWorkspace.evHu[37];
acadoWorkspace.A[3638] = acadoWorkspace.evHu[38];
acadoWorkspace.A[3939] = acadoWorkspace.evHu[39];
acadoWorkspace.A[3940] = acadoWorkspace.evHu[40];
acadoWorkspace.A[3941] = acadoWorkspace.evHu[41];
acadoWorkspace.A[4242] = acadoWorkspace.evHu[42];
acadoWorkspace.A[4243] = acadoWorkspace.evHu[43];
acadoWorkspace.A[4244] = acadoWorkspace.evHu[44];
acadoWorkspace.A[4545] = acadoWorkspace.evHu[45];
acadoWorkspace.A[4546] = acadoWorkspace.evHu[46];
acadoWorkspace.A[4547] = acadoWorkspace.evHu[47];
acadoWorkspace.A[4848] = acadoWorkspace.evHu[48];
acadoWorkspace.A[4849] = acadoWorkspace.evHu[49];
acadoWorkspace.A[4850] = acadoWorkspace.evHu[50];
acadoWorkspace.A[5151] = acadoWorkspace.evHu[51];
acadoWorkspace.A[5152] = acadoWorkspace.evHu[52];
acadoWorkspace.A[5153] = acadoWorkspace.evHu[53];
acadoWorkspace.A[5454] = acadoWorkspace.evHu[54];
acadoWorkspace.A[5455] = acadoWorkspace.evHu[55];
acadoWorkspace.A[5456] = acadoWorkspace.evHu[56];
acadoWorkspace.A[5757] = acadoWorkspace.evHu[57];
acadoWorkspace.A[5758] = acadoWorkspace.evHu[58];
acadoWorkspace.A[5759] = acadoWorkspace.evHu[59];
acadoWorkspace.A[6060] = acadoWorkspace.evHu[60];
acadoWorkspace.A[6061] = acadoWorkspace.evHu[61];
acadoWorkspace.A[6062] = acadoWorkspace.evHu[62];
acadoWorkspace.A[6363] = acadoWorkspace.evHu[63];
acadoWorkspace.A[6364] = acadoWorkspace.evHu[64];
acadoWorkspace.A[6365] = acadoWorkspace.evHu[65];
acadoWorkspace.A[6666] = acadoWorkspace.evHu[66];
acadoWorkspace.A[6667] = acadoWorkspace.evHu[67];
acadoWorkspace.A[6668] = acadoWorkspace.evHu[68];
acadoWorkspace.A[6969] = acadoWorkspace.evHu[69];
acadoWorkspace.A[6970] = acadoWorkspace.evHu[70];
acadoWorkspace.A[6971] = acadoWorkspace.evHu[71];
acadoWorkspace.A[7272] = acadoWorkspace.evHu[72];
acadoWorkspace.A[7273] = acadoWorkspace.evHu[73];
acadoWorkspace.A[7274] = acadoWorkspace.evHu[74];
acadoWorkspace.A[7575] = acadoWorkspace.evHu[75];
acadoWorkspace.A[7576] = acadoWorkspace.evHu[76];
acadoWorkspace.A[7577] = acadoWorkspace.evHu[77];
acadoWorkspace.A[7878] = acadoWorkspace.evHu[78];
acadoWorkspace.A[7879] = acadoWorkspace.evHu[79];
acadoWorkspace.A[7880] = acadoWorkspace.evHu[80];
acadoWorkspace.A[8181] = acadoWorkspace.evHu[81];
acadoWorkspace.A[8182] = acadoWorkspace.evHu[82];
acadoWorkspace.A[8183] = acadoWorkspace.evHu[83];
acadoWorkspace.A[8484] = acadoWorkspace.evHu[84];
acadoWorkspace.A[8485] = acadoWorkspace.evHu[85];
acadoWorkspace.A[8486] = acadoWorkspace.evHu[86];
acadoWorkspace.A[8787] = acadoWorkspace.evHu[87];
acadoWorkspace.A[8788] = acadoWorkspace.evHu[88];
acadoWorkspace.A[8789] = acadoWorkspace.evHu[89];
acadoWorkspace.A[9090] = acadoWorkspace.evHu[90];
acadoWorkspace.A[9091] = acadoWorkspace.evHu[91];
acadoWorkspace.A[9092] = acadoWorkspace.evHu[92];
acadoWorkspace.A[9393] = acadoWorkspace.evHu[93];
acadoWorkspace.A[9394] = acadoWorkspace.evHu[94];
acadoWorkspace.A[9395] = acadoWorkspace.evHu[95];
acadoWorkspace.A[9696] = acadoWorkspace.evHu[96];
acadoWorkspace.A[9697] = acadoWorkspace.evHu[97];
acadoWorkspace.A[9698] = acadoWorkspace.evHu[98];
acadoWorkspace.A[9999] = acadoWorkspace.evHu[99];
acadoWorkspace.A[10000] = acadoWorkspace.evHu[100];
acadoWorkspace.A[10001] = acadoWorkspace.evHu[101];
acadoWorkspace.A[10302] = acadoWorkspace.evHu[102];
acadoWorkspace.A[10303] = acadoWorkspace.evHu[103];
acadoWorkspace.A[10304] = acadoWorkspace.evHu[104];
acadoWorkspace.A[10605] = acadoWorkspace.evHu[105];
acadoWorkspace.A[10606] = acadoWorkspace.evHu[106];
acadoWorkspace.A[10607] = acadoWorkspace.evHu[107];
acadoWorkspace.A[10908] = acadoWorkspace.evHu[108];
acadoWorkspace.A[10909] = acadoWorkspace.evHu[109];
acadoWorkspace.A[10910] = acadoWorkspace.evHu[110];
acadoWorkspace.A[11211] = acadoWorkspace.evHu[111];
acadoWorkspace.A[11212] = acadoWorkspace.evHu[112];
acadoWorkspace.A[11213] = acadoWorkspace.evHu[113];
acadoWorkspace.A[11514] = acadoWorkspace.evHu[114];
acadoWorkspace.A[11515] = acadoWorkspace.evHu[115];
acadoWorkspace.A[11516] = acadoWorkspace.evHu[116];
acadoWorkspace.A[11817] = acadoWorkspace.evHu[117];
acadoWorkspace.A[11818] = acadoWorkspace.evHu[118];
acadoWorkspace.A[11819] = acadoWorkspace.evHu[119];
acadoWorkspace.A[12120] = acadoWorkspace.evHu[120];
acadoWorkspace.A[12121] = acadoWorkspace.evHu[121];
acadoWorkspace.A[12122] = acadoWorkspace.evHu[122];
acadoWorkspace.A[12423] = acadoWorkspace.evHu[123];
acadoWorkspace.A[12424] = acadoWorkspace.evHu[124];
acadoWorkspace.A[12425] = acadoWorkspace.evHu[125];
acadoWorkspace.A[12726] = acadoWorkspace.evHu[126];
acadoWorkspace.A[12727] = acadoWorkspace.evHu[127];
acadoWorkspace.A[12728] = acadoWorkspace.evHu[128];
acadoWorkspace.A[13029] = acadoWorkspace.evHu[129];
acadoWorkspace.A[13030] = acadoWorkspace.evHu[130];
acadoWorkspace.A[13031] = acadoWorkspace.evHu[131];
acadoWorkspace.A[13332] = acadoWorkspace.evHu[132];
acadoWorkspace.A[13333] = acadoWorkspace.evHu[133];
acadoWorkspace.A[13334] = acadoWorkspace.evHu[134];
acadoWorkspace.A[13635] = acadoWorkspace.evHu[135];
acadoWorkspace.A[13636] = acadoWorkspace.evHu[136];
acadoWorkspace.A[13637] = acadoWorkspace.evHu[137];
acadoWorkspace.A[13938] = acadoWorkspace.evHu[138];
acadoWorkspace.A[13939] = acadoWorkspace.evHu[139];
acadoWorkspace.A[13940] = acadoWorkspace.evHu[140];
acadoWorkspace.A[14241] = acadoWorkspace.evHu[141];
acadoWorkspace.A[14242] = acadoWorkspace.evHu[142];
acadoWorkspace.A[14243] = acadoWorkspace.evHu[143];
acadoWorkspace.A[14544] = acadoWorkspace.evHu[144];
acadoWorkspace.A[14545] = acadoWorkspace.evHu[145];
acadoWorkspace.A[14546] = acadoWorkspace.evHu[146];
acadoWorkspace.A[14847] = acadoWorkspace.evHu[147];
acadoWorkspace.A[14848] = acadoWorkspace.evHu[148];
acadoWorkspace.A[14849] = acadoWorkspace.evHu[149];
acadoWorkspace.A[15150] = acadoWorkspace.evHu[150];
acadoWorkspace.A[15151] = acadoWorkspace.evHu[151];
acadoWorkspace.A[15152] = acadoWorkspace.evHu[152];
acadoWorkspace.A[15453] = acadoWorkspace.evHu[153];
acadoWorkspace.A[15454] = acadoWorkspace.evHu[154];
acadoWorkspace.A[15455] = acadoWorkspace.evHu[155];
acadoWorkspace.A[15756] = acadoWorkspace.evHu[156];
acadoWorkspace.A[15757] = acadoWorkspace.evHu[157];
acadoWorkspace.A[15758] = acadoWorkspace.evHu[158];
acadoWorkspace.A[16059] = acadoWorkspace.evHu[159];
acadoWorkspace.A[16060] = acadoWorkspace.evHu[160];
acadoWorkspace.A[16061] = acadoWorkspace.evHu[161];
acadoWorkspace.A[16362] = acadoWorkspace.evHu[162];
acadoWorkspace.A[16363] = acadoWorkspace.evHu[163];
acadoWorkspace.A[16364] = acadoWorkspace.evHu[164];
acadoWorkspace.A[16665] = acadoWorkspace.evHu[165];
acadoWorkspace.A[16666] = acadoWorkspace.evHu[166];
acadoWorkspace.A[16667] = acadoWorkspace.evHu[167];
acadoWorkspace.A[16968] = acadoWorkspace.evHu[168];
acadoWorkspace.A[16969] = acadoWorkspace.evHu[169];
acadoWorkspace.A[16970] = acadoWorkspace.evHu[170];
acadoWorkspace.A[17271] = acadoWorkspace.evHu[171];
acadoWorkspace.A[17272] = acadoWorkspace.evHu[172];
acadoWorkspace.A[17273] = acadoWorkspace.evHu[173];
acadoWorkspace.A[17574] = acadoWorkspace.evHu[174];
acadoWorkspace.A[17575] = acadoWorkspace.evHu[175];
acadoWorkspace.A[17576] = acadoWorkspace.evHu[176];
acadoWorkspace.A[17877] = acadoWorkspace.evHu[177];
acadoWorkspace.A[17878] = acadoWorkspace.evHu[178];
acadoWorkspace.A[17879] = acadoWorkspace.evHu[179];
acadoWorkspace.A[18180] = acadoWorkspace.evHu[180];
acadoWorkspace.A[18181] = acadoWorkspace.evHu[181];
acadoWorkspace.A[18182] = acadoWorkspace.evHu[182];
acadoWorkspace.A[18483] = acadoWorkspace.evHu[183];
acadoWorkspace.A[18484] = acadoWorkspace.evHu[184];
acadoWorkspace.A[18485] = acadoWorkspace.evHu[185];
acadoWorkspace.A[18786] = acadoWorkspace.evHu[186];
acadoWorkspace.A[18787] = acadoWorkspace.evHu[187];
acadoWorkspace.A[18788] = acadoWorkspace.evHu[188];
acadoWorkspace.A[19089] = acadoWorkspace.evHu[189];
acadoWorkspace.A[19090] = acadoWorkspace.evHu[190];
acadoWorkspace.A[19091] = acadoWorkspace.evHu[191];
acadoWorkspace.A[19392] = acadoWorkspace.evHu[192];
acadoWorkspace.A[19393] = acadoWorkspace.evHu[193];
acadoWorkspace.A[19394] = acadoWorkspace.evHu[194];
acadoWorkspace.A[19695] = acadoWorkspace.evHu[195];
acadoWorkspace.A[19696] = acadoWorkspace.evHu[196];
acadoWorkspace.A[19697] = acadoWorkspace.evHu[197];
acadoWorkspace.A[19998] = acadoWorkspace.evHu[198];
acadoWorkspace.A[19999] = acadoWorkspace.evHu[199];
acadoWorkspace.A[20000] = acadoWorkspace.evHu[200];
acadoWorkspace.A[20301] = acadoWorkspace.evHu[201];
acadoWorkspace.A[20302] = acadoWorkspace.evHu[202];
acadoWorkspace.A[20303] = acadoWorkspace.evHu[203];
acadoWorkspace.A[20604] = acadoWorkspace.evHu[204];
acadoWorkspace.A[20605] = acadoWorkspace.evHu[205];
acadoWorkspace.A[20606] = acadoWorkspace.evHu[206];
acadoWorkspace.A[20907] = acadoWorkspace.evHu[207];
acadoWorkspace.A[20908] = acadoWorkspace.evHu[208];
acadoWorkspace.A[20909] = acadoWorkspace.evHu[209];
acadoWorkspace.A[21210] = acadoWorkspace.evHu[210];
acadoWorkspace.A[21211] = acadoWorkspace.evHu[211];
acadoWorkspace.A[21212] = acadoWorkspace.evHu[212];
acadoWorkspace.A[21513] = acadoWorkspace.evHu[213];
acadoWorkspace.A[21514] = acadoWorkspace.evHu[214];
acadoWorkspace.A[21515] = acadoWorkspace.evHu[215];
acadoWorkspace.A[21816] = acadoWorkspace.evHu[216];
acadoWorkspace.A[21817] = acadoWorkspace.evHu[217];
acadoWorkspace.A[21818] = acadoWorkspace.evHu[218];
acadoWorkspace.A[22119] = acadoWorkspace.evHu[219];
acadoWorkspace.A[22120] = acadoWorkspace.evHu[220];
acadoWorkspace.A[22121] = acadoWorkspace.evHu[221];
acadoWorkspace.A[22422] = acadoWorkspace.evHu[222];
acadoWorkspace.A[22423] = acadoWorkspace.evHu[223];
acadoWorkspace.A[22424] = acadoWorkspace.evHu[224];
acadoWorkspace.A[22725] = acadoWorkspace.evHu[225];
acadoWorkspace.A[22726] = acadoWorkspace.evHu[226];
acadoWorkspace.A[22727] = acadoWorkspace.evHu[227];
acadoWorkspace.A[23028] = acadoWorkspace.evHu[228];
acadoWorkspace.A[23029] = acadoWorkspace.evHu[229];
acadoWorkspace.A[23030] = acadoWorkspace.evHu[230];
acadoWorkspace.A[23331] = acadoWorkspace.evHu[231];
acadoWorkspace.A[23332] = acadoWorkspace.evHu[232];
acadoWorkspace.A[23333] = acadoWorkspace.evHu[233];
acadoWorkspace.A[23634] = acadoWorkspace.evHu[234];
acadoWorkspace.A[23635] = acadoWorkspace.evHu[235];
acadoWorkspace.A[23636] = acadoWorkspace.evHu[236];
acadoWorkspace.A[23937] = acadoWorkspace.evHu[237];
acadoWorkspace.A[23938] = acadoWorkspace.evHu[238];
acadoWorkspace.A[23939] = acadoWorkspace.evHu[239];
acadoWorkspace.A[24240] = acadoWorkspace.evHu[240];
acadoWorkspace.A[24241] = acadoWorkspace.evHu[241];
acadoWorkspace.A[24242] = acadoWorkspace.evHu[242];
acadoWorkspace.A[24543] = acadoWorkspace.evHu[243];
acadoWorkspace.A[24544] = acadoWorkspace.evHu[244];
acadoWorkspace.A[24545] = acadoWorkspace.evHu[245];
acadoWorkspace.A[24846] = acadoWorkspace.evHu[246];
acadoWorkspace.A[24847] = acadoWorkspace.evHu[247];
acadoWorkspace.A[24848] = acadoWorkspace.evHu[248];
acadoWorkspace.A[25149] = acadoWorkspace.evHu[249];
acadoWorkspace.A[25150] = acadoWorkspace.evHu[250];
acadoWorkspace.A[25151] = acadoWorkspace.evHu[251];
acadoWorkspace.A[25452] = acadoWorkspace.evHu[252];
acadoWorkspace.A[25453] = acadoWorkspace.evHu[253];
acadoWorkspace.A[25454] = acadoWorkspace.evHu[254];
acadoWorkspace.A[25755] = acadoWorkspace.evHu[255];
acadoWorkspace.A[25756] = acadoWorkspace.evHu[256];
acadoWorkspace.A[25757] = acadoWorkspace.evHu[257];
acadoWorkspace.A[26058] = acadoWorkspace.evHu[258];
acadoWorkspace.A[26059] = acadoWorkspace.evHu[259];
acadoWorkspace.A[26060] = acadoWorkspace.evHu[260];
acadoWorkspace.A[26361] = acadoWorkspace.evHu[261];
acadoWorkspace.A[26362] = acadoWorkspace.evHu[262];
acadoWorkspace.A[26363] = acadoWorkspace.evHu[263];
acadoWorkspace.A[26664] = acadoWorkspace.evHu[264];
acadoWorkspace.A[26665] = acadoWorkspace.evHu[265];
acadoWorkspace.A[26666] = acadoWorkspace.evHu[266];
acadoWorkspace.A[26967] = acadoWorkspace.evHu[267];
acadoWorkspace.A[26968] = acadoWorkspace.evHu[268];
acadoWorkspace.A[26969] = acadoWorkspace.evHu[269];
acadoWorkspace.A[27270] = acadoWorkspace.evHu[270];
acadoWorkspace.A[27271] = acadoWorkspace.evHu[271];
acadoWorkspace.A[27272] = acadoWorkspace.evHu[272];
acadoWorkspace.A[27573] = acadoWorkspace.evHu[273];
acadoWorkspace.A[27574] = acadoWorkspace.evHu[274];
acadoWorkspace.A[27575] = acadoWorkspace.evHu[275];
acadoWorkspace.A[27876] = acadoWorkspace.evHu[276];
acadoWorkspace.A[27877] = acadoWorkspace.evHu[277];
acadoWorkspace.A[27878] = acadoWorkspace.evHu[278];
acadoWorkspace.A[28179] = acadoWorkspace.evHu[279];
acadoWorkspace.A[28180] = acadoWorkspace.evHu[280];
acadoWorkspace.A[28181] = acadoWorkspace.evHu[281];
acadoWorkspace.A[28482] = acadoWorkspace.evHu[282];
acadoWorkspace.A[28483] = acadoWorkspace.evHu[283];
acadoWorkspace.A[28484] = acadoWorkspace.evHu[284];
acadoWorkspace.A[28785] = acadoWorkspace.evHu[285];
acadoWorkspace.A[28786] = acadoWorkspace.evHu[286];
acadoWorkspace.A[28787] = acadoWorkspace.evHu[287];
acadoWorkspace.A[29088] = acadoWorkspace.evHu[288];
acadoWorkspace.A[29089] = acadoWorkspace.evHu[289];
acadoWorkspace.A[29090] = acadoWorkspace.evHu[290];
acadoWorkspace.A[29391] = acadoWorkspace.evHu[291];
acadoWorkspace.A[29392] = acadoWorkspace.evHu[292];
acadoWorkspace.A[29393] = acadoWorkspace.evHu[293];
acadoWorkspace.A[29694] = acadoWorkspace.evHu[294];
acadoWorkspace.A[29695] = acadoWorkspace.evHu[295];
acadoWorkspace.A[29696] = acadoWorkspace.evHu[296];
acadoWorkspace.A[29997] = acadoWorkspace.evHu[297];
acadoWorkspace.A[29998] = acadoWorkspace.evHu[298];
acadoWorkspace.A[29999] = acadoWorkspace.evHu[299];
acadoWorkspace.lbA[0] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[4];
acadoWorkspace.lbA[5] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[5];
acadoWorkspace.lbA[6] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[6];
acadoWorkspace.lbA[7] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[7];
acadoWorkspace.lbA[8] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[8];
acadoWorkspace.lbA[9] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[9];
acadoWorkspace.lbA[10] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[10];
acadoWorkspace.lbA[11] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[11];
acadoWorkspace.lbA[12] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[12];
acadoWorkspace.lbA[13] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[13];
acadoWorkspace.lbA[14] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[14];
acadoWorkspace.lbA[15] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[15];
acadoWorkspace.lbA[16] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[16];
acadoWorkspace.lbA[17] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[17];
acadoWorkspace.lbA[18] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[18];
acadoWorkspace.lbA[19] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[19];
acadoWorkspace.lbA[20] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[20];
acadoWorkspace.lbA[21] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[21];
acadoWorkspace.lbA[22] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[22];
acadoWorkspace.lbA[23] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[23];
acadoWorkspace.lbA[24] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[24];
acadoWorkspace.lbA[25] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[25];
acadoWorkspace.lbA[26] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[26];
acadoWorkspace.lbA[27] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[27];
acadoWorkspace.lbA[28] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[28];
acadoWorkspace.lbA[29] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[29];
acadoWorkspace.lbA[30] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[30];
acadoWorkspace.lbA[31] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[31];
acadoWorkspace.lbA[32] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[32];
acadoWorkspace.lbA[33] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[33];
acadoWorkspace.lbA[34] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[34];
acadoWorkspace.lbA[35] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[35];
acadoWorkspace.lbA[36] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[36];
acadoWorkspace.lbA[37] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[37];
acadoWorkspace.lbA[38] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[38];
acadoWorkspace.lbA[39] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[39];
acadoWorkspace.lbA[40] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[40];
acadoWorkspace.lbA[41] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[41];
acadoWorkspace.lbA[42] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[42];
acadoWorkspace.lbA[43] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[43];
acadoWorkspace.lbA[44] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[44];
acadoWorkspace.lbA[45] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[45];
acadoWorkspace.lbA[46] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[46];
acadoWorkspace.lbA[47] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[47];
acadoWorkspace.lbA[48] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[48];
acadoWorkspace.lbA[49] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[49];
acadoWorkspace.lbA[50] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[50];
acadoWorkspace.lbA[51] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[51];
acadoWorkspace.lbA[52] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[52];
acadoWorkspace.lbA[53] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[53];
acadoWorkspace.lbA[54] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[54];
acadoWorkspace.lbA[55] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[55];
acadoWorkspace.lbA[56] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[56];
acadoWorkspace.lbA[57] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[57];
acadoWorkspace.lbA[58] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[58];
acadoWorkspace.lbA[59] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[59];
acadoWorkspace.lbA[60] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[60];
acadoWorkspace.lbA[61] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[61];
acadoWorkspace.lbA[62] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[62];
acadoWorkspace.lbA[63] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[63];
acadoWorkspace.lbA[64] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[64];
acadoWorkspace.lbA[65] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[65];
acadoWorkspace.lbA[66] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[66];
acadoWorkspace.lbA[67] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[67];
acadoWorkspace.lbA[68] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[68];
acadoWorkspace.lbA[69] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[69];
acadoWorkspace.lbA[70] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[70];
acadoWorkspace.lbA[71] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[71];
acadoWorkspace.lbA[72] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[72];
acadoWorkspace.lbA[73] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[73];
acadoWorkspace.lbA[74] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[74];
acadoWorkspace.lbA[75] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[75];
acadoWorkspace.lbA[76] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[76];
acadoWorkspace.lbA[77] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[77];
acadoWorkspace.lbA[78] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[78];
acadoWorkspace.lbA[79] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[79];
acadoWorkspace.lbA[80] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[80];
acadoWorkspace.lbA[81] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[81];
acadoWorkspace.lbA[82] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[82];
acadoWorkspace.lbA[83] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[83];
acadoWorkspace.lbA[84] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[84];
acadoWorkspace.lbA[85] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[85];
acadoWorkspace.lbA[86] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[86];
acadoWorkspace.lbA[87] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[87];
acadoWorkspace.lbA[88] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[88];
acadoWorkspace.lbA[89] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[89];
acadoWorkspace.lbA[90] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[90];
acadoWorkspace.lbA[91] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[91];
acadoWorkspace.lbA[92] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[92];
acadoWorkspace.lbA[93] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[93];
acadoWorkspace.lbA[94] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[94];
acadoWorkspace.lbA[95] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[95];
acadoWorkspace.lbA[96] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[96];
acadoWorkspace.lbA[97] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[97];
acadoWorkspace.lbA[98] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[98];
acadoWorkspace.lbA[99] = (real_t)-4.0000000000000000e+00 - acadoWorkspace.evH[99];

acadoWorkspace.ubA[0] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[4];
acadoWorkspace.ubA[5] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[5];
acadoWorkspace.ubA[6] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[6];
acadoWorkspace.ubA[7] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[7];
acadoWorkspace.ubA[8] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[8];
acadoWorkspace.ubA[9] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[9];
acadoWorkspace.ubA[10] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[10];
acadoWorkspace.ubA[11] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[11];
acadoWorkspace.ubA[12] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[12];
acadoWorkspace.ubA[13] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[13];
acadoWorkspace.ubA[14] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[14];
acadoWorkspace.ubA[15] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[15];
acadoWorkspace.ubA[16] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[16];
acadoWorkspace.ubA[17] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[17];
acadoWorkspace.ubA[18] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[18];
acadoWorkspace.ubA[19] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[19];
acadoWorkspace.ubA[20] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[20];
acadoWorkspace.ubA[21] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[21];
acadoWorkspace.ubA[22] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[22];
acadoWorkspace.ubA[23] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[23];
acadoWorkspace.ubA[24] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[24];
acadoWorkspace.ubA[25] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[25];
acadoWorkspace.ubA[26] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[26];
acadoWorkspace.ubA[27] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[27];
acadoWorkspace.ubA[28] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[28];
acadoWorkspace.ubA[29] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[29];
acadoWorkspace.ubA[30] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[30];
acadoWorkspace.ubA[31] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[31];
acadoWorkspace.ubA[32] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[32];
acadoWorkspace.ubA[33] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[33];
acadoWorkspace.ubA[34] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[34];
acadoWorkspace.ubA[35] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[35];
acadoWorkspace.ubA[36] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[36];
acadoWorkspace.ubA[37] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[37];
acadoWorkspace.ubA[38] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[38];
acadoWorkspace.ubA[39] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[39];
acadoWorkspace.ubA[40] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[40];
acadoWorkspace.ubA[41] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[41];
acadoWorkspace.ubA[42] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[42];
acadoWorkspace.ubA[43] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[43];
acadoWorkspace.ubA[44] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[44];
acadoWorkspace.ubA[45] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[45];
acadoWorkspace.ubA[46] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[46];
acadoWorkspace.ubA[47] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[47];
acadoWorkspace.ubA[48] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[48];
acadoWorkspace.ubA[49] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[49];
acadoWorkspace.ubA[50] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[50];
acadoWorkspace.ubA[51] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[51];
acadoWorkspace.ubA[52] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[52];
acadoWorkspace.ubA[53] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[53];
acadoWorkspace.ubA[54] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[54];
acadoWorkspace.ubA[55] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[55];
acadoWorkspace.ubA[56] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[56];
acadoWorkspace.ubA[57] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[57];
acadoWorkspace.ubA[58] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[58];
acadoWorkspace.ubA[59] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[59];
acadoWorkspace.ubA[60] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[60];
acadoWorkspace.ubA[61] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[61];
acadoWorkspace.ubA[62] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[62];
acadoWorkspace.ubA[63] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[63];
acadoWorkspace.ubA[64] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[64];
acadoWorkspace.ubA[65] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[65];
acadoWorkspace.ubA[66] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[66];
acadoWorkspace.ubA[67] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[67];
acadoWorkspace.ubA[68] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[68];
acadoWorkspace.ubA[69] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[69];
acadoWorkspace.ubA[70] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[70];
acadoWorkspace.ubA[71] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[71];
acadoWorkspace.ubA[72] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[72];
acadoWorkspace.ubA[73] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[73];
acadoWorkspace.ubA[74] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[74];
acadoWorkspace.ubA[75] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[75];
acadoWorkspace.ubA[76] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[76];
acadoWorkspace.ubA[77] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[77];
acadoWorkspace.ubA[78] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[78];
acadoWorkspace.ubA[79] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[79];
acadoWorkspace.ubA[80] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[80];
acadoWorkspace.ubA[81] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[81];
acadoWorkspace.ubA[82] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[82];
acadoWorkspace.ubA[83] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[83];
acadoWorkspace.ubA[84] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[84];
acadoWorkspace.ubA[85] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[85];
acadoWorkspace.ubA[86] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[86];
acadoWorkspace.ubA[87] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[87];
acadoWorkspace.ubA[88] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[88];
acadoWorkspace.ubA[89] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[89];
acadoWorkspace.ubA[90] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[90];
acadoWorkspace.ubA[91] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[91];
acadoWorkspace.ubA[92] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[92];
acadoWorkspace.ubA[93] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[93];
acadoWorkspace.ubA[94] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[94];
acadoWorkspace.ubA[95] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[95];
acadoWorkspace.ubA[96] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[96];
acadoWorkspace.ubA[97] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[97];
acadoWorkspace.ubA[98] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[98];
acadoWorkspace.ubA[99] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[99];

acado_macHxd( &(acadoWorkspace.evHx[ 7 ]), acadoWorkspace.d, &(acadoWorkspace.lbA[ 1 ]), &(acadoWorkspace.ubA[ 1 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 14 ]), &(acadoWorkspace.d[ 7 ]), &(acadoWorkspace.lbA[ 2 ]), &(acadoWorkspace.ubA[ 2 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 21 ]), &(acadoWorkspace.d[ 14 ]), &(acadoWorkspace.lbA[ 3 ]), &(acadoWorkspace.ubA[ 3 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 28 ]), &(acadoWorkspace.d[ 21 ]), &(acadoWorkspace.lbA[ 4 ]), &(acadoWorkspace.ubA[ 4 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 35 ]), &(acadoWorkspace.d[ 28 ]), &(acadoWorkspace.lbA[ 5 ]), &(acadoWorkspace.ubA[ 5 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.d[ 35 ]), &(acadoWorkspace.lbA[ 6 ]), &(acadoWorkspace.ubA[ 6 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 49 ]), &(acadoWorkspace.d[ 42 ]), &(acadoWorkspace.lbA[ 7 ]), &(acadoWorkspace.ubA[ 7 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.d[ 49 ]), &(acadoWorkspace.lbA[ 8 ]), &(acadoWorkspace.ubA[ 8 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 63 ]), &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.lbA[ 9 ]), &(acadoWorkspace.ubA[ 9 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 70 ]), &(acadoWorkspace.d[ 63 ]), &(acadoWorkspace.lbA[ 10 ]), &(acadoWorkspace.ubA[ 10 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 77 ]), &(acadoWorkspace.d[ 70 ]), &(acadoWorkspace.lbA[ 11 ]), &(acadoWorkspace.ubA[ 11 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.d[ 77 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 91 ]), &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.lbA[ 13 ]), &(acadoWorkspace.ubA[ 13 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 98 ]), &(acadoWorkspace.d[ 91 ]), &(acadoWorkspace.lbA[ 14 ]), &(acadoWorkspace.ubA[ 14 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 105 ]), &(acadoWorkspace.d[ 98 ]), &(acadoWorkspace.lbA[ 15 ]), &(acadoWorkspace.ubA[ 15 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 112 ]), &(acadoWorkspace.d[ 105 ]), &(acadoWorkspace.lbA[ 16 ]), &(acadoWorkspace.ubA[ 16 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 119 ]), &(acadoWorkspace.d[ 112 ]), &(acadoWorkspace.lbA[ 17 ]), &(acadoWorkspace.ubA[ 17 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 126 ]), &(acadoWorkspace.d[ 119 ]), &(acadoWorkspace.lbA[ 18 ]), &(acadoWorkspace.ubA[ 18 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 133 ]), &(acadoWorkspace.d[ 126 ]), &(acadoWorkspace.lbA[ 19 ]), &(acadoWorkspace.ubA[ 19 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.d[ 133 ]), &(acadoWorkspace.lbA[ 20 ]), &(acadoWorkspace.ubA[ 20 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 147 ]), &(acadoWorkspace.d[ 140 ]), &(acadoWorkspace.lbA[ 21 ]), &(acadoWorkspace.ubA[ 21 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 154 ]), &(acadoWorkspace.d[ 147 ]), &(acadoWorkspace.lbA[ 22 ]), &(acadoWorkspace.ubA[ 22 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 161 ]), &(acadoWorkspace.d[ 154 ]), &(acadoWorkspace.lbA[ 23 ]), &(acadoWorkspace.ubA[ 23 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.d[ 161 ]), &(acadoWorkspace.lbA[ 24 ]), &(acadoWorkspace.ubA[ 24 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 175 ]), &(acadoWorkspace.d[ 168 ]), &(acadoWorkspace.lbA[ 25 ]), &(acadoWorkspace.ubA[ 25 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 182 ]), &(acadoWorkspace.d[ 175 ]), &(acadoWorkspace.lbA[ 26 ]), &(acadoWorkspace.ubA[ 26 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 189 ]), &(acadoWorkspace.d[ 182 ]), &(acadoWorkspace.lbA[ 27 ]), &(acadoWorkspace.ubA[ 27 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 196 ]), &(acadoWorkspace.d[ 189 ]), &(acadoWorkspace.lbA[ 28 ]), &(acadoWorkspace.ubA[ 28 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 203 ]), &(acadoWorkspace.d[ 196 ]), &(acadoWorkspace.lbA[ 29 ]), &(acadoWorkspace.ubA[ 29 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.d[ 203 ]), &(acadoWorkspace.lbA[ 30 ]), &(acadoWorkspace.ubA[ 30 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 217 ]), &(acadoWorkspace.d[ 210 ]), &(acadoWorkspace.lbA[ 31 ]), &(acadoWorkspace.ubA[ 31 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 224 ]), &(acadoWorkspace.d[ 217 ]), &(acadoWorkspace.lbA[ 32 ]), &(acadoWorkspace.ubA[ 32 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 231 ]), &(acadoWorkspace.d[ 224 ]), &(acadoWorkspace.lbA[ 33 ]), &(acadoWorkspace.ubA[ 33 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 238 ]), &(acadoWorkspace.d[ 231 ]), &(acadoWorkspace.lbA[ 34 ]), &(acadoWorkspace.ubA[ 34 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 245 ]), &(acadoWorkspace.d[ 238 ]), &(acadoWorkspace.lbA[ 35 ]), &(acadoWorkspace.ubA[ 35 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 252 ]), &(acadoWorkspace.d[ 245 ]), &(acadoWorkspace.lbA[ 36 ]), &(acadoWorkspace.ubA[ 36 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 259 ]), &(acadoWorkspace.d[ 252 ]), &(acadoWorkspace.lbA[ 37 ]), &(acadoWorkspace.ubA[ 37 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 266 ]), &(acadoWorkspace.d[ 259 ]), &(acadoWorkspace.lbA[ 38 ]), &(acadoWorkspace.ubA[ 38 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 273 ]), &(acadoWorkspace.d[ 266 ]), &(acadoWorkspace.lbA[ 39 ]), &(acadoWorkspace.ubA[ 39 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 280 ]), &(acadoWorkspace.d[ 273 ]), &(acadoWorkspace.lbA[ 40 ]), &(acadoWorkspace.ubA[ 40 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 287 ]), &(acadoWorkspace.d[ 280 ]), &(acadoWorkspace.lbA[ 41 ]), &(acadoWorkspace.ubA[ 41 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 294 ]), &(acadoWorkspace.d[ 287 ]), &(acadoWorkspace.lbA[ 42 ]), &(acadoWorkspace.ubA[ 42 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 301 ]), &(acadoWorkspace.d[ 294 ]), &(acadoWorkspace.lbA[ 43 ]), &(acadoWorkspace.ubA[ 43 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 308 ]), &(acadoWorkspace.d[ 301 ]), &(acadoWorkspace.lbA[ 44 ]), &(acadoWorkspace.ubA[ 44 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 315 ]), &(acadoWorkspace.d[ 308 ]), &(acadoWorkspace.lbA[ 45 ]), &(acadoWorkspace.ubA[ 45 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 322 ]), &(acadoWorkspace.d[ 315 ]), &(acadoWorkspace.lbA[ 46 ]), &(acadoWorkspace.ubA[ 46 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 329 ]), &(acadoWorkspace.d[ 322 ]), &(acadoWorkspace.lbA[ 47 ]), &(acadoWorkspace.ubA[ 47 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 336 ]), &(acadoWorkspace.d[ 329 ]), &(acadoWorkspace.lbA[ 48 ]), &(acadoWorkspace.ubA[ 48 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 343 ]), &(acadoWorkspace.d[ 336 ]), &(acadoWorkspace.lbA[ 49 ]), &(acadoWorkspace.ubA[ 49 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 350 ]), &(acadoWorkspace.d[ 343 ]), &(acadoWorkspace.lbA[ 50 ]), &(acadoWorkspace.ubA[ 50 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 357 ]), &(acadoWorkspace.d[ 350 ]), &(acadoWorkspace.lbA[ 51 ]), &(acadoWorkspace.ubA[ 51 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 364 ]), &(acadoWorkspace.d[ 357 ]), &(acadoWorkspace.lbA[ 52 ]), &(acadoWorkspace.ubA[ 52 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 371 ]), &(acadoWorkspace.d[ 364 ]), &(acadoWorkspace.lbA[ 53 ]), &(acadoWorkspace.ubA[ 53 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 378 ]), &(acadoWorkspace.d[ 371 ]), &(acadoWorkspace.lbA[ 54 ]), &(acadoWorkspace.ubA[ 54 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 385 ]), &(acadoWorkspace.d[ 378 ]), &(acadoWorkspace.lbA[ 55 ]), &(acadoWorkspace.ubA[ 55 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 392 ]), &(acadoWorkspace.d[ 385 ]), &(acadoWorkspace.lbA[ 56 ]), &(acadoWorkspace.ubA[ 56 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 399 ]), &(acadoWorkspace.d[ 392 ]), &(acadoWorkspace.lbA[ 57 ]), &(acadoWorkspace.ubA[ 57 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 406 ]), &(acadoWorkspace.d[ 399 ]), &(acadoWorkspace.lbA[ 58 ]), &(acadoWorkspace.ubA[ 58 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 413 ]), &(acadoWorkspace.d[ 406 ]), &(acadoWorkspace.lbA[ 59 ]), &(acadoWorkspace.ubA[ 59 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 420 ]), &(acadoWorkspace.d[ 413 ]), &(acadoWorkspace.lbA[ 60 ]), &(acadoWorkspace.ubA[ 60 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 427 ]), &(acadoWorkspace.d[ 420 ]), &(acadoWorkspace.lbA[ 61 ]), &(acadoWorkspace.ubA[ 61 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 434 ]), &(acadoWorkspace.d[ 427 ]), &(acadoWorkspace.lbA[ 62 ]), &(acadoWorkspace.ubA[ 62 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 441 ]), &(acadoWorkspace.d[ 434 ]), &(acadoWorkspace.lbA[ 63 ]), &(acadoWorkspace.ubA[ 63 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 448 ]), &(acadoWorkspace.d[ 441 ]), &(acadoWorkspace.lbA[ 64 ]), &(acadoWorkspace.ubA[ 64 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 455 ]), &(acadoWorkspace.d[ 448 ]), &(acadoWorkspace.lbA[ 65 ]), &(acadoWorkspace.ubA[ 65 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 462 ]), &(acadoWorkspace.d[ 455 ]), &(acadoWorkspace.lbA[ 66 ]), &(acadoWorkspace.ubA[ 66 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 469 ]), &(acadoWorkspace.d[ 462 ]), &(acadoWorkspace.lbA[ 67 ]), &(acadoWorkspace.ubA[ 67 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 476 ]), &(acadoWorkspace.d[ 469 ]), &(acadoWorkspace.lbA[ 68 ]), &(acadoWorkspace.ubA[ 68 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 483 ]), &(acadoWorkspace.d[ 476 ]), &(acadoWorkspace.lbA[ 69 ]), &(acadoWorkspace.ubA[ 69 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 490 ]), &(acadoWorkspace.d[ 483 ]), &(acadoWorkspace.lbA[ 70 ]), &(acadoWorkspace.ubA[ 70 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 497 ]), &(acadoWorkspace.d[ 490 ]), &(acadoWorkspace.lbA[ 71 ]), &(acadoWorkspace.ubA[ 71 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 504 ]), &(acadoWorkspace.d[ 497 ]), &(acadoWorkspace.lbA[ 72 ]), &(acadoWorkspace.ubA[ 72 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 511 ]), &(acadoWorkspace.d[ 504 ]), &(acadoWorkspace.lbA[ 73 ]), &(acadoWorkspace.ubA[ 73 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 518 ]), &(acadoWorkspace.d[ 511 ]), &(acadoWorkspace.lbA[ 74 ]), &(acadoWorkspace.ubA[ 74 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 525 ]), &(acadoWorkspace.d[ 518 ]), &(acadoWorkspace.lbA[ 75 ]), &(acadoWorkspace.ubA[ 75 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 532 ]), &(acadoWorkspace.d[ 525 ]), &(acadoWorkspace.lbA[ 76 ]), &(acadoWorkspace.ubA[ 76 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 539 ]), &(acadoWorkspace.d[ 532 ]), &(acadoWorkspace.lbA[ 77 ]), &(acadoWorkspace.ubA[ 77 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 546 ]), &(acadoWorkspace.d[ 539 ]), &(acadoWorkspace.lbA[ 78 ]), &(acadoWorkspace.ubA[ 78 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 553 ]), &(acadoWorkspace.d[ 546 ]), &(acadoWorkspace.lbA[ 79 ]), &(acadoWorkspace.ubA[ 79 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 560 ]), &(acadoWorkspace.d[ 553 ]), &(acadoWorkspace.lbA[ 80 ]), &(acadoWorkspace.ubA[ 80 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 567 ]), &(acadoWorkspace.d[ 560 ]), &(acadoWorkspace.lbA[ 81 ]), &(acadoWorkspace.ubA[ 81 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 574 ]), &(acadoWorkspace.d[ 567 ]), &(acadoWorkspace.lbA[ 82 ]), &(acadoWorkspace.ubA[ 82 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 581 ]), &(acadoWorkspace.d[ 574 ]), &(acadoWorkspace.lbA[ 83 ]), &(acadoWorkspace.ubA[ 83 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 588 ]), &(acadoWorkspace.d[ 581 ]), &(acadoWorkspace.lbA[ 84 ]), &(acadoWorkspace.ubA[ 84 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 595 ]), &(acadoWorkspace.d[ 588 ]), &(acadoWorkspace.lbA[ 85 ]), &(acadoWorkspace.ubA[ 85 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 602 ]), &(acadoWorkspace.d[ 595 ]), &(acadoWorkspace.lbA[ 86 ]), &(acadoWorkspace.ubA[ 86 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 609 ]), &(acadoWorkspace.d[ 602 ]), &(acadoWorkspace.lbA[ 87 ]), &(acadoWorkspace.ubA[ 87 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 616 ]), &(acadoWorkspace.d[ 609 ]), &(acadoWorkspace.lbA[ 88 ]), &(acadoWorkspace.ubA[ 88 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 623 ]), &(acadoWorkspace.d[ 616 ]), &(acadoWorkspace.lbA[ 89 ]), &(acadoWorkspace.ubA[ 89 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 630 ]), &(acadoWorkspace.d[ 623 ]), &(acadoWorkspace.lbA[ 90 ]), &(acadoWorkspace.ubA[ 90 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 637 ]), &(acadoWorkspace.d[ 630 ]), &(acadoWorkspace.lbA[ 91 ]), &(acadoWorkspace.ubA[ 91 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 644 ]), &(acadoWorkspace.d[ 637 ]), &(acadoWorkspace.lbA[ 92 ]), &(acadoWorkspace.ubA[ 92 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 651 ]), &(acadoWorkspace.d[ 644 ]), &(acadoWorkspace.lbA[ 93 ]), &(acadoWorkspace.ubA[ 93 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 658 ]), &(acadoWorkspace.d[ 651 ]), &(acadoWorkspace.lbA[ 94 ]), &(acadoWorkspace.ubA[ 94 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 665 ]), &(acadoWorkspace.d[ 658 ]), &(acadoWorkspace.lbA[ 95 ]), &(acadoWorkspace.ubA[ 95 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 672 ]), &(acadoWorkspace.d[ 665 ]), &(acadoWorkspace.lbA[ 96 ]), &(acadoWorkspace.ubA[ 96 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 679 ]), &(acadoWorkspace.d[ 672 ]), &(acadoWorkspace.lbA[ 97 ]), &(acadoWorkspace.ubA[ 97 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 686 ]), &(acadoWorkspace.d[ 679 ]), &(acadoWorkspace.lbA[ 98 ]), &(acadoWorkspace.ubA[ 98 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 693 ]), &(acadoWorkspace.d[ 686 ]), &(acadoWorkspace.lbA[ 99 ]), &(acadoWorkspace.ubA[ 99 ]) );

}

void acado_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];

for (lRun2 = 0; lRun2 < 500; ++lRun2)
acadoWorkspace.Dy[lRun2] -= acadoVariables.y[lRun2];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 15 ]), &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 30 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 45 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 60 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 75 ]), &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 90 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 105 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 120 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 135 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 150 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 165 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 180 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 195 ]), &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 210 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 225 ]), &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 240 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 255 ]), &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 270 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 285 ]), &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.g[ 57 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 300 ]), &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 315 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.g[ 63 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 330 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.g[ 66 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 345 ]), &(acadoWorkspace.Dy[ 115 ]), &(acadoWorkspace.g[ 69 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 360 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 375 ]), &(acadoWorkspace.Dy[ 125 ]), &(acadoWorkspace.g[ 75 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 390 ]), &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.g[ 78 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 405 ]), &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.g[ 81 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 420 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.g[ 84 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 435 ]), &(acadoWorkspace.Dy[ 145 ]), &(acadoWorkspace.g[ 87 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 450 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.g[ 90 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 465 ]), &(acadoWorkspace.Dy[ 155 ]), &(acadoWorkspace.g[ 93 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 480 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.g[ 96 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 495 ]), &(acadoWorkspace.Dy[ 165 ]), &(acadoWorkspace.g[ 99 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 510 ]), &(acadoWorkspace.Dy[ 170 ]), &(acadoWorkspace.g[ 102 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 525 ]), &(acadoWorkspace.Dy[ 175 ]), &(acadoWorkspace.g[ 105 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 540 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 108 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 555 ]), &(acadoWorkspace.Dy[ 185 ]), &(acadoWorkspace.g[ 111 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 570 ]), &(acadoWorkspace.Dy[ 190 ]), &(acadoWorkspace.g[ 114 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 585 ]), &(acadoWorkspace.Dy[ 195 ]), &(acadoWorkspace.g[ 117 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 600 ]), &(acadoWorkspace.Dy[ 200 ]), &(acadoWorkspace.g[ 120 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 615 ]), &(acadoWorkspace.Dy[ 205 ]), &(acadoWorkspace.g[ 123 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 630 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.g[ 126 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 645 ]), &(acadoWorkspace.Dy[ 215 ]), &(acadoWorkspace.g[ 129 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 660 ]), &(acadoWorkspace.Dy[ 220 ]), &(acadoWorkspace.g[ 132 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 675 ]), &(acadoWorkspace.Dy[ 225 ]), &(acadoWorkspace.g[ 135 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 690 ]), &(acadoWorkspace.Dy[ 230 ]), &(acadoWorkspace.g[ 138 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 705 ]), &(acadoWorkspace.Dy[ 235 ]), &(acadoWorkspace.g[ 141 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 720 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.g[ 144 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 735 ]), &(acadoWorkspace.Dy[ 245 ]), &(acadoWorkspace.g[ 147 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 750 ]), &(acadoWorkspace.Dy[ 250 ]), &(acadoWorkspace.g[ 150 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 765 ]), &(acadoWorkspace.Dy[ 255 ]), &(acadoWorkspace.g[ 153 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 780 ]), &(acadoWorkspace.Dy[ 260 ]), &(acadoWorkspace.g[ 156 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 795 ]), &(acadoWorkspace.Dy[ 265 ]), &(acadoWorkspace.g[ 159 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 810 ]), &(acadoWorkspace.Dy[ 270 ]), &(acadoWorkspace.g[ 162 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 825 ]), &(acadoWorkspace.Dy[ 275 ]), &(acadoWorkspace.g[ 165 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 840 ]), &(acadoWorkspace.Dy[ 280 ]), &(acadoWorkspace.g[ 168 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 855 ]), &(acadoWorkspace.Dy[ 285 ]), &(acadoWorkspace.g[ 171 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 870 ]), &(acadoWorkspace.Dy[ 290 ]), &(acadoWorkspace.g[ 174 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 885 ]), &(acadoWorkspace.Dy[ 295 ]), &(acadoWorkspace.g[ 177 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 900 ]), &(acadoWorkspace.Dy[ 300 ]), &(acadoWorkspace.g[ 180 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 915 ]), &(acadoWorkspace.Dy[ 305 ]), &(acadoWorkspace.g[ 183 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 930 ]), &(acadoWorkspace.Dy[ 310 ]), &(acadoWorkspace.g[ 186 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 945 ]), &(acadoWorkspace.Dy[ 315 ]), &(acadoWorkspace.g[ 189 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 960 ]), &(acadoWorkspace.Dy[ 320 ]), &(acadoWorkspace.g[ 192 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 975 ]), &(acadoWorkspace.Dy[ 325 ]), &(acadoWorkspace.g[ 195 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 990 ]), &(acadoWorkspace.Dy[ 330 ]), &(acadoWorkspace.g[ 198 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1005 ]), &(acadoWorkspace.Dy[ 335 ]), &(acadoWorkspace.g[ 201 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1020 ]), &(acadoWorkspace.Dy[ 340 ]), &(acadoWorkspace.g[ 204 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1035 ]), &(acadoWorkspace.Dy[ 345 ]), &(acadoWorkspace.g[ 207 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1050 ]), &(acadoWorkspace.Dy[ 350 ]), &(acadoWorkspace.g[ 210 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1065 ]), &(acadoWorkspace.Dy[ 355 ]), &(acadoWorkspace.g[ 213 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1080 ]), &(acadoWorkspace.Dy[ 360 ]), &(acadoWorkspace.g[ 216 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1095 ]), &(acadoWorkspace.Dy[ 365 ]), &(acadoWorkspace.g[ 219 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1110 ]), &(acadoWorkspace.Dy[ 370 ]), &(acadoWorkspace.g[ 222 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1125 ]), &(acadoWorkspace.Dy[ 375 ]), &(acadoWorkspace.g[ 225 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1140 ]), &(acadoWorkspace.Dy[ 380 ]), &(acadoWorkspace.g[ 228 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1155 ]), &(acadoWorkspace.Dy[ 385 ]), &(acadoWorkspace.g[ 231 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1170 ]), &(acadoWorkspace.Dy[ 390 ]), &(acadoWorkspace.g[ 234 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1185 ]), &(acadoWorkspace.Dy[ 395 ]), &(acadoWorkspace.g[ 237 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1200 ]), &(acadoWorkspace.Dy[ 400 ]), &(acadoWorkspace.g[ 240 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1215 ]), &(acadoWorkspace.Dy[ 405 ]), &(acadoWorkspace.g[ 243 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1230 ]), &(acadoWorkspace.Dy[ 410 ]), &(acadoWorkspace.g[ 246 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1245 ]), &(acadoWorkspace.Dy[ 415 ]), &(acadoWorkspace.g[ 249 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1260 ]), &(acadoWorkspace.Dy[ 420 ]), &(acadoWorkspace.g[ 252 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1275 ]), &(acadoWorkspace.Dy[ 425 ]), &(acadoWorkspace.g[ 255 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1290 ]), &(acadoWorkspace.Dy[ 430 ]), &(acadoWorkspace.g[ 258 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1305 ]), &(acadoWorkspace.Dy[ 435 ]), &(acadoWorkspace.g[ 261 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1320 ]), &(acadoWorkspace.Dy[ 440 ]), &(acadoWorkspace.g[ 264 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1335 ]), &(acadoWorkspace.Dy[ 445 ]), &(acadoWorkspace.g[ 267 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1350 ]), &(acadoWorkspace.Dy[ 450 ]), &(acadoWorkspace.g[ 270 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1365 ]), &(acadoWorkspace.Dy[ 455 ]), &(acadoWorkspace.g[ 273 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1380 ]), &(acadoWorkspace.Dy[ 460 ]), &(acadoWorkspace.g[ 276 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1395 ]), &(acadoWorkspace.Dy[ 465 ]), &(acadoWorkspace.g[ 279 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1410 ]), &(acadoWorkspace.Dy[ 470 ]), &(acadoWorkspace.g[ 282 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1425 ]), &(acadoWorkspace.Dy[ 475 ]), &(acadoWorkspace.g[ 285 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1440 ]), &(acadoWorkspace.Dy[ 480 ]), &(acadoWorkspace.g[ 288 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1455 ]), &(acadoWorkspace.Dy[ 485 ]), &(acadoWorkspace.g[ 291 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1470 ]), &(acadoWorkspace.Dy[ 490 ]), &(acadoWorkspace.g[ 294 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1485 ]), &(acadoWorkspace.Dy[ 495 ]), &(acadoWorkspace.g[ 297 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 35 ]), &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.QDy[ 7 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 70 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.QDy[ 14 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 105 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 21 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 140 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 28 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 175 ]), &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.QDy[ 35 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 210 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 42 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 245 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 49 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 280 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 56 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 315 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 63 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 350 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.QDy[ 70 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 385 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.QDy[ 77 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 420 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 455 ]), &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.QDy[ 91 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 490 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 98 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 525 ]), &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.QDy[ 105 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 560 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 112 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 595 ]), &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.QDy[ 119 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 630 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 126 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 665 ]), &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.QDy[ 133 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 700 ]), &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.QDy[ 140 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 735 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.QDy[ 147 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 770 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.QDy[ 154 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 805 ]), &(acadoWorkspace.Dy[ 115 ]), &(acadoWorkspace.QDy[ 161 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 840 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 168 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 875 ]), &(acadoWorkspace.Dy[ 125 ]), &(acadoWorkspace.QDy[ 175 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 910 ]), &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.QDy[ 182 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 945 ]), &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.QDy[ 189 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 980 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.QDy[ 196 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1015 ]), &(acadoWorkspace.Dy[ 145 ]), &(acadoWorkspace.QDy[ 203 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1050 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.QDy[ 210 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1085 ]), &(acadoWorkspace.Dy[ 155 ]), &(acadoWorkspace.QDy[ 217 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1120 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.QDy[ 224 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1155 ]), &(acadoWorkspace.Dy[ 165 ]), &(acadoWorkspace.QDy[ 231 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1190 ]), &(acadoWorkspace.Dy[ 170 ]), &(acadoWorkspace.QDy[ 238 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1225 ]), &(acadoWorkspace.Dy[ 175 ]), &(acadoWorkspace.QDy[ 245 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1260 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 252 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1295 ]), &(acadoWorkspace.Dy[ 185 ]), &(acadoWorkspace.QDy[ 259 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1330 ]), &(acadoWorkspace.Dy[ 190 ]), &(acadoWorkspace.QDy[ 266 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1365 ]), &(acadoWorkspace.Dy[ 195 ]), &(acadoWorkspace.QDy[ 273 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1400 ]), &(acadoWorkspace.Dy[ 200 ]), &(acadoWorkspace.QDy[ 280 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1435 ]), &(acadoWorkspace.Dy[ 205 ]), &(acadoWorkspace.QDy[ 287 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1470 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.QDy[ 294 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1505 ]), &(acadoWorkspace.Dy[ 215 ]), &(acadoWorkspace.QDy[ 301 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1540 ]), &(acadoWorkspace.Dy[ 220 ]), &(acadoWorkspace.QDy[ 308 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1575 ]), &(acadoWorkspace.Dy[ 225 ]), &(acadoWorkspace.QDy[ 315 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1610 ]), &(acadoWorkspace.Dy[ 230 ]), &(acadoWorkspace.QDy[ 322 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1645 ]), &(acadoWorkspace.Dy[ 235 ]), &(acadoWorkspace.QDy[ 329 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1680 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.QDy[ 336 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1715 ]), &(acadoWorkspace.Dy[ 245 ]), &(acadoWorkspace.QDy[ 343 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1750 ]), &(acadoWorkspace.Dy[ 250 ]), &(acadoWorkspace.QDy[ 350 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1785 ]), &(acadoWorkspace.Dy[ 255 ]), &(acadoWorkspace.QDy[ 357 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1820 ]), &(acadoWorkspace.Dy[ 260 ]), &(acadoWorkspace.QDy[ 364 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1855 ]), &(acadoWorkspace.Dy[ 265 ]), &(acadoWorkspace.QDy[ 371 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1890 ]), &(acadoWorkspace.Dy[ 270 ]), &(acadoWorkspace.QDy[ 378 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1925 ]), &(acadoWorkspace.Dy[ 275 ]), &(acadoWorkspace.QDy[ 385 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1960 ]), &(acadoWorkspace.Dy[ 280 ]), &(acadoWorkspace.QDy[ 392 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1995 ]), &(acadoWorkspace.Dy[ 285 ]), &(acadoWorkspace.QDy[ 399 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2030 ]), &(acadoWorkspace.Dy[ 290 ]), &(acadoWorkspace.QDy[ 406 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2065 ]), &(acadoWorkspace.Dy[ 295 ]), &(acadoWorkspace.QDy[ 413 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2100 ]), &(acadoWorkspace.Dy[ 300 ]), &(acadoWorkspace.QDy[ 420 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2135 ]), &(acadoWorkspace.Dy[ 305 ]), &(acadoWorkspace.QDy[ 427 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2170 ]), &(acadoWorkspace.Dy[ 310 ]), &(acadoWorkspace.QDy[ 434 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2205 ]), &(acadoWorkspace.Dy[ 315 ]), &(acadoWorkspace.QDy[ 441 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2240 ]), &(acadoWorkspace.Dy[ 320 ]), &(acadoWorkspace.QDy[ 448 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2275 ]), &(acadoWorkspace.Dy[ 325 ]), &(acadoWorkspace.QDy[ 455 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2310 ]), &(acadoWorkspace.Dy[ 330 ]), &(acadoWorkspace.QDy[ 462 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2345 ]), &(acadoWorkspace.Dy[ 335 ]), &(acadoWorkspace.QDy[ 469 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2380 ]), &(acadoWorkspace.Dy[ 340 ]), &(acadoWorkspace.QDy[ 476 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2415 ]), &(acadoWorkspace.Dy[ 345 ]), &(acadoWorkspace.QDy[ 483 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2450 ]), &(acadoWorkspace.Dy[ 350 ]), &(acadoWorkspace.QDy[ 490 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2485 ]), &(acadoWorkspace.Dy[ 355 ]), &(acadoWorkspace.QDy[ 497 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2520 ]), &(acadoWorkspace.Dy[ 360 ]), &(acadoWorkspace.QDy[ 504 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2555 ]), &(acadoWorkspace.Dy[ 365 ]), &(acadoWorkspace.QDy[ 511 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2590 ]), &(acadoWorkspace.Dy[ 370 ]), &(acadoWorkspace.QDy[ 518 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2625 ]), &(acadoWorkspace.Dy[ 375 ]), &(acadoWorkspace.QDy[ 525 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2660 ]), &(acadoWorkspace.Dy[ 380 ]), &(acadoWorkspace.QDy[ 532 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2695 ]), &(acadoWorkspace.Dy[ 385 ]), &(acadoWorkspace.QDy[ 539 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2730 ]), &(acadoWorkspace.Dy[ 390 ]), &(acadoWorkspace.QDy[ 546 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2765 ]), &(acadoWorkspace.Dy[ 395 ]), &(acadoWorkspace.QDy[ 553 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2800 ]), &(acadoWorkspace.Dy[ 400 ]), &(acadoWorkspace.QDy[ 560 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2835 ]), &(acadoWorkspace.Dy[ 405 ]), &(acadoWorkspace.QDy[ 567 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2870 ]), &(acadoWorkspace.Dy[ 410 ]), &(acadoWorkspace.QDy[ 574 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2905 ]), &(acadoWorkspace.Dy[ 415 ]), &(acadoWorkspace.QDy[ 581 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2940 ]), &(acadoWorkspace.Dy[ 420 ]), &(acadoWorkspace.QDy[ 588 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2975 ]), &(acadoWorkspace.Dy[ 425 ]), &(acadoWorkspace.QDy[ 595 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3010 ]), &(acadoWorkspace.Dy[ 430 ]), &(acadoWorkspace.QDy[ 602 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3045 ]), &(acadoWorkspace.Dy[ 435 ]), &(acadoWorkspace.QDy[ 609 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3080 ]), &(acadoWorkspace.Dy[ 440 ]), &(acadoWorkspace.QDy[ 616 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3115 ]), &(acadoWorkspace.Dy[ 445 ]), &(acadoWorkspace.QDy[ 623 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3150 ]), &(acadoWorkspace.Dy[ 450 ]), &(acadoWorkspace.QDy[ 630 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3185 ]), &(acadoWorkspace.Dy[ 455 ]), &(acadoWorkspace.QDy[ 637 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3220 ]), &(acadoWorkspace.Dy[ 460 ]), &(acadoWorkspace.QDy[ 644 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3255 ]), &(acadoWorkspace.Dy[ 465 ]), &(acadoWorkspace.QDy[ 651 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3290 ]), &(acadoWorkspace.Dy[ 470 ]), &(acadoWorkspace.QDy[ 658 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3325 ]), &(acadoWorkspace.Dy[ 475 ]), &(acadoWorkspace.QDy[ 665 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3360 ]), &(acadoWorkspace.Dy[ 480 ]), &(acadoWorkspace.QDy[ 672 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3395 ]), &(acadoWorkspace.Dy[ 485 ]), &(acadoWorkspace.QDy[ 679 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3430 ]), &(acadoWorkspace.Dy[ 490 ]), &(acadoWorkspace.QDy[ 686 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3465 ]), &(acadoWorkspace.Dy[ 495 ]), &(acadoWorkspace.QDy[ 693 ]) );

acadoWorkspace.QDy[700] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[701] = + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[702] = + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[703] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[704] = + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[705] = + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[706] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1];

for (lRun2 = 0; lRun2 < 700; ++lRun2)
acadoWorkspace.QDy[lRun2 + 7] += acadoWorkspace.Qd[lRun2];


for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 100; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multEQDy( &(acadoWorkspace.E[ lRun3 * 21 ]), &(acadoWorkspace.QDy[ lRun2 * 7 + 7 ]), &(acadoWorkspace.g[ lRun1 * 3 ]) );
}
}

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[1] += + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[2] += + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[3] += + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[4] += + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[5] += + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[6] += + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[7] += + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[8] += + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[9] += + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[10] += + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[11] += + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[12] += + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[13] += + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[14] += + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[15] += + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[16] += + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[17] += + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[18] += + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[19] += + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[20] += + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[21] += + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[22] += + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[23] += + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[24] += + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[25] += + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[26] += + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[27] += + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[28] += + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[29] += + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[30] += + acadoWorkspace.H10[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[214]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[215]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[216]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[31] += + acadoWorkspace.H10[217]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[218]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[219]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[220]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[221]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[222]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[223]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[32] += + acadoWorkspace.H10[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[227]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[228]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[229]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[230]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[33] += + acadoWorkspace.H10[231]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[232]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[233]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[234]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[235]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[236]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[237]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[34] += + acadoWorkspace.H10[238]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[239]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[240]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[241]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[242]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[243]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[244]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[35] += + acadoWorkspace.H10[245]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[246]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[247]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[248]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[249]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[250]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[251]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[36] += + acadoWorkspace.H10[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[256]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[257]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[258]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[37] += + acadoWorkspace.H10[259]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[260]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[261]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[262]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[263]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[264]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[265]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[38] += + acadoWorkspace.H10[266]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[267]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[268]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[269]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[270]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[271]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[272]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[39] += + acadoWorkspace.H10[273]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[274]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[275]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[276]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[277]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[278]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[279]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[40] += + acadoWorkspace.H10[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[284]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[285]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[286]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[41] += + acadoWorkspace.H10[287]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[288]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[289]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[290]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[291]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[292]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[293]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[42] += + acadoWorkspace.H10[294]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[295]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[296]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[297]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[298]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[299]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[300]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[43] += + acadoWorkspace.H10[301]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[302]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[303]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[304]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[305]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[306]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[307]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[44] += + acadoWorkspace.H10[308]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[309]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[310]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[311]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[312]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[313]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[314]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[45] += + acadoWorkspace.H10[315]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[316]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[317]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[318]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[319]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[320]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[321]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[46] += + acadoWorkspace.H10[322]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[323]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[324]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[325]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[326]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[327]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[328]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[47] += + acadoWorkspace.H10[329]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[330]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[331]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[332]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[333]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[334]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[335]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[48] += + acadoWorkspace.H10[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[339]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[340]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[341]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[342]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[49] += + acadoWorkspace.H10[343]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[344]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[345]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[346]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[347]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[348]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[349]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[50] += + acadoWorkspace.H10[350]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[351]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[352]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[353]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[354]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[355]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[356]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[51] += + acadoWorkspace.H10[357]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[358]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[359]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[360]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[361]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[362]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[363]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[52] += + acadoWorkspace.H10[364]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[365]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[366]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[367]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[368]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[369]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[370]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[53] += + acadoWorkspace.H10[371]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[372]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[373]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[374]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[375]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[376]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[377]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[54] += + acadoWorkspace.H10[378]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[379]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[380]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[381]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[382]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[383]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[384]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[55] += + acadoWorkspace.H10[385]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[386]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[387]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[388]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[389]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[390]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[391]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[56] += + acadoWorkspace.H10[392]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[393]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[394]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[395]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[396]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[397]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[398]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[57] += + acadoWorkspace.H10[399]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[400]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[401]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[402]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[403]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[404]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[405]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[58] += + acadoWorkspace.H10[406]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[407]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[408]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[409]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[410]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[411]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[412]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[59] += + acadoWorkspace.H10[413]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[414]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[415]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[416]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[417]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[418]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[419]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[60] += + acadoWorkspace.H10[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[424]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[425]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[426]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[61] += + acadoWorkspace.H10[427]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[428]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[429]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[430]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[431]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[432]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[433]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[62] += + acadoWorkspace.H10[434]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[435]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[436]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[437]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[438]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[439]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[440]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[63] += + acadoWorkspace.H10[441]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[442]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[443]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[444]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[445]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[446]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[447]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[64] += + acadoWorkspace.H10[448]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[449]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[450]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[451]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[452]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[453]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[454]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[65] += + acadoWorkspace.H10[455]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[456]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[457]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[458]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[459]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[460]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[461]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[66] += + acadoWorkspace.H10[462]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[463]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[464]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[465]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[466]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[467]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[468]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[67] += + acadoWorkspace.H10[469]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[470]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[471]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[472]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[473]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[474]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[475]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[68] += + acadoWorkspace.H10[476]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[477]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[478]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[479]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[480]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[481]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[482]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[69] += + acadoWorkspace.H10[483]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[484]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[485]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[486]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[487]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[488]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[489]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[70] += + acadoWorkspace.H10[490]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[491]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[492]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[493]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[494]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[495]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[496]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[71] += + acadoWorkspace.H10[497]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[498]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[499]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[500]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[501]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[502]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[503]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[72] += + acadoWorkspace.H10[504]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[505]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[506]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[507]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[508]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[509]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[510]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[73] += + acadoWorkspace.H10[511]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[512]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[513]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[514]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[515]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[516]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[517]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[74] += + acadoWorkspace.H10[518]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[519]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[520]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[521]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[522]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[523]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[524]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[75] += + acadoWorkspace.H10[525]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[526]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[527]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[528]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[529]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[530]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[531]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[76] += + acadoWorkspace.H10[532]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[533]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[534]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[535]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[536]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[537]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[538]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[77] += + acadoWorkspace.H10[539]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[540]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[541]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[542]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[543]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[544]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[545]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[78] += + acadoWorkspace.H10[546]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[547]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[548]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[549]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[550]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[551]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[552]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[79] += + acadoWorkspace.H10[553]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[554]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[555]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[556]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[557]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[558]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[559]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[80] += + acadoWorkspace.H10[560]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[561]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[562]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[563]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[564]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[565]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[566]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[81] += + acadoWorkspace.H10[567]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[568]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[569]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[570]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[571]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[572]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[573]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[82] += + acadoWorkspace.H10[574]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[575]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[576]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[577]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[578]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[579]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[580]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[83] += + acadoWorkspace.H10[581]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[582]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[583]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[584]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[585]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[586]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[587]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[84] += + acadoWorkspace.H10[588]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[589]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[590]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[591]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[592]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[593]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[594]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[85] += + acadoWorkspace.H10[595]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[596]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[597]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[598]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[599]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[600]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[601]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[86] += + acadoWorkspace.H10[602]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[603]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[604]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[605]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[606]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[607]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[608]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[87] += + acadoWorkspace.H10[609]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[610]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[611]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[612]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[613]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[614]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[615]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[88] += + acadoWorkspace.H10[616]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[617]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[618]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[619]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[620]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[621]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[622]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[89] += + acadoWorkspace.H10[623]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[624]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[625]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[626]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[627]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[628]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[629]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[90] += + acadoWorkspace.H10[630]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[631]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[632]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[633]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[634]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[635]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[636]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[91] += + acadoWorkspace.H10[637]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[638]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[639]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[640]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[641]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[642]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[643]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[92] += + acadoWorkspace.H10[644]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[645]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[646]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[647]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[648]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[649]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[650]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[93] += + acadoWorkspace.H10[651]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[652]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[653]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[654]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[655]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[656]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[657]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[94] += + acadoWorkspace.H10[658]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[659]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[660]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[661]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[662]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[663]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[664]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[95] += + acadoWorkspace.H10[665]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[666]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[667]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[668]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[669]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[670]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[671]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[96] += + acadoWorkspace.H10[672]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[673]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[674]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[675]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[676]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[677]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[678]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[97] += + acadoWorkspace.H10[679]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[680]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[681]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[682]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[683]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[684]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[685]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[98] += + acadoWorkspace.H10[686]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[687]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[688]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[689]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[690]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[691]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[692]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[99] += + acadoWorkspace.H10[693]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[694]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[695]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[696]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[697]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[698]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[699]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[100] += + acadoWorkspace.H10[700]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[701]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[702]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[703]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[704]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[705]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[706]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[101] += + acadoWorkspace.H10[707]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[708]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[709]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[710]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[711]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[712]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[713]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[102] += + acadoWorkspace.H10[714]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[715]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[716]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[717]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[718]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[719]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[720]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[103] += + acadoWorkspace.H10[721]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[722]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[723]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[724]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[725]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[726]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[727]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[104] += + acadoWorkspace.H10[728]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[729]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[730]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[731]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[732]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[733]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[734]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[105] += + acadoWorkspace.H10[735]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[736]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[737]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[738]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[739]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[740]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[741]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[106] += + acadoWorkspace.H10[742]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[743]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[744]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[745]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[746]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[747]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[748]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[107] += + acadoWorkspace.H10[749]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[750]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[751]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[752]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[753]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[754]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[755]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[108] += + acadoWorkspace.H10[756]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[757]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[758]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[759]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[760]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[761]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[762]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[109] += + acadoWorkspace.H10[763]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[764]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[765]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[766]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[767]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[768]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[769]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[110] += + acadoWorkspace.H10[770]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[771]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[772]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[773]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[774]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[775]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[776]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[111] += + acadoWorkspace.H10[777]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[778]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[779]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[780]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[781]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[782]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[783]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[112] += + acadoWorkspace.H10[784]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[785]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[786]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[787]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[788]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[789]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[790]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[113] += + acadoWorkspace.H10[791]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[792]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[793]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[794]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[795]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[796]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[797]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[114] += + acadoWorkspace.H10[798]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[799]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[800]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[801]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[802]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[803]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[804]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[115] += + acadoWorkspace.H10[805]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[806]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[807]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[808]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[809]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[810]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[811]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[116] += + acadoWorkspace.H10[812]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[813]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[814]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[815]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[816]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[817]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[818]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[117] += + acadoWorkspace.H10[819]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[820]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[821]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[822]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[823]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[824]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[825]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[118] += + acadoWorkspace.H10[826]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[827]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[828]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[829]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[830]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[831]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[832]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[119] += + acadoWorkspace.H10[833]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[834]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[835]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[836]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[837]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[838]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[839]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[120] += + acadoWorkspace.H10[840]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[841]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[842]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[843]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[844]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[845]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[846]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[121] += + acadoWorkspace.H10[847]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[848]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[849]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[850]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[851]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[852]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[853]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[122] += + acadoWorkspace.H10[854]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[855]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[856]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[857]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[858]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[859]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[860]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[123] += + acadoWorkspace.H10[861]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[862]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[863]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[864]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[865]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[866]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[867]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[124] += + acadoWorkspace.H10[868]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[869]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[870]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[871]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[872]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[873]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[874]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[125] += + acadoWorkspace.H10[875]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[876]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[877]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[878]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[879]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[880]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[881]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[126] += + acadoWorkspace.H10[882]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[883]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[884]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[885]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[886]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[887]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[888]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[127] += + acadoWorkspace.H10[889]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[890]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[891]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[892]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[893]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[894]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[895]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[128] += + acadoWorkspace.H10[896]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[897]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[898]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[899]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[900]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[901]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[902]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[129] += + acadoWorkspace.H10[903]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[904]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[905]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[906]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[907]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[908]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[909]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[130] += + acadoWorkspace.H10[910]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[911]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[912]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[913]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[914]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[915]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[916]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[131] += + acadoWorkspace.H10[917]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[918]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[919]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[920]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[921]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[922]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[923]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[132] += + acadoWorkspace.H10[924]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[925]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[926]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[927]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[928]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[929]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[930]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[133] += + acadoWorkspace.H10[931]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[932]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[933]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[934]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[935]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[936]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[937]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[134] += + acadoWorkspace.H10[938]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[939]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[940]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[941]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[942]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[943]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[944]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[135] += + acadoWorkspace.H10[945]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[946]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[947]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[948]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[949]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[950]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[951]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[136] += + acadoWorkspace.H10[952]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[953]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[954]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[955]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[956]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[957]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[958]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[137] += + acadoWorkspace.H10[959]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[960]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[961]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[962]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[963]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[964]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[965]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[138] += + acadoWorkspace.H10[966]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[967]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[968]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[969]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[970]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[971]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[972]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[139] += + acadoWorkspace.H10[973]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[974]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[975]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[976]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[977]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[978]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[979]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[140] += + acadoWorkspace.H10[980]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[981]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[982]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[983]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[984]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[985]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[986]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[141] += + acadoWorkspace.H10[987]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[988]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[989]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[990]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[991]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[992]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[993]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[142] += + acadoWorkspace.H10[994]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[995]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[996]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[997]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[998]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[999]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1000]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[143] += + acadoWorkspace.H10[1001]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1002]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1003]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1004]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1005]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1006]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1007]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[144] += + acadoWorkspace.H10[1008]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1009]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1010]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1011]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1012]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1013]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1014]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[145] += + acadoWorkspace.H10[1015]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1016]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1017]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1018]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1019]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1020]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1021]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[146] += + acadoWorkspace.H10[1022]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1023]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1024]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1025]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1026]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1027]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1028]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[147] += + acadoWorkspace.H10[1029]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1030]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1031]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1032]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1033]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1034]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1035]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[148] += + acadoWorkspace.H10[1036]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1037]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1038]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1039]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1040]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1041]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1042]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[149] += + acadoWorkspace.H10[1043]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1044]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1045]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1046]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1047]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1048]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1049]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[150] += + acadoWorkspace.H10[1050]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1051]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1052]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1053]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1054]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1055]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1056]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[151] += + acadoWorkspace.H10[1057]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1058]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1059]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1060]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1061]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1062]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1063]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[152] += + acadoWorkspace.H10[1064]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1065]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1066]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1067]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1068]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1069]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1070]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[153] += + acadoWorkspace.H10[1071]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1072]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1073]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1074]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1075]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1076]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1077]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[154] += + acadoWorkspace.H10[1078]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1079]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1080]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1081]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1082]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1083]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1084]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[155] += + acadoWorkspace.H10[1085]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1086]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1087]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1088]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1089]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1090]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1091]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[156] += + acadoWorkspace.H10[1092]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1093]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1094]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1095]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1096]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1097]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1098]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[157] += + acadoWorkspace.H10[1099]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1100]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1101]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1102]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1103]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1104]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1105]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[158] += + acadoWorkspace.H10[1106]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1107]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1108]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1109]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1110]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1111]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1112]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[159] += + acadoWorkspace.H10[1113]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1114]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1115]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1116]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1117]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1118]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1119]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[160] += + acadoWorkspace.H10[1120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1123]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1124]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1125]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1126]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[161] += + acadoWorkspace.H10[1127]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1128]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1129]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1130]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1131]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1132]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1133]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[162] += + acadoWorkspace.H10[1134]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1135]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1136]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1137]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1138]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1139]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1140]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[163] += + acadoWorkspace.H10[1141]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1142]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1143]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1144]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1145]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1146]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1147]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[164] += + acadoWorkspace.H10[1148]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1149]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1150]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1151]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1152]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1153]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1154]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[165] += + acadoWorkspace.H10[1155]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1156]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1157]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1158]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1159]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1160]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1161]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[166] += + acadoWorkspace.H10[1162]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1163]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1164]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1165]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1166]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1167]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1168]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[167] += + acadoWorkspace.H10[1169]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1170]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1171]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1172]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1173]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1174]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1175]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[168] += + acadoWorkspace.H10[1176]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1177]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1178]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1179]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1180]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1181]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1182]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[169] += + acadoWorkspace.H10[1183]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1184]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1185]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1186]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1187]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1188]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1189]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[170] += + acadoWorkspace.H10[1190]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1191]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1192]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1193]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1194]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1195]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1196]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[171] += + acadoWorkspace.H10[1197]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1198]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1199]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1200]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1201]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1202]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1203]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[172] += + acadoWorkspace.H10[1204]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1205]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1206]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1207]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1208]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1209]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1210]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[173] += + acadoWorkspace.H10[1211]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1212]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1213]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1214]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1215]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1216]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1217]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[174] += + acadoWorkspace.H10[1218]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1219]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1220]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1221]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1222]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1223]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1224]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[175] += + acadoWorkspace.H10[1225]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1226]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1227]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1228]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1229]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1230]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1231]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[176] += + acadoWorkspace.H10[1232]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1233]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1234]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1235]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1236]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1237]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1238]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[177] += + acadoWorkspace.H10[1239]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1240]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1241]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1242]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1243]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1244]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1245]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[178] += + acadoWorkspace.H10[1246]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1247]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1248]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1249]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1250]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1251]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1252]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[179] += + acadoWorkspace.H10[1253]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1254]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1255]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1256]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1257]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1258]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1259]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[180] += + acadoWorkspace.H10[1260]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1261]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1262]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1263]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1264]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1265]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1266]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[181] += + acadoWorkspace.H10[1267]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1268]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1269]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1270]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1271]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1272]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1273]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[182] += + acadoWorkspace.H10[1274]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1275]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1276]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1277]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1278]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1279]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1280]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[183] += + acadoWorkspace.H10[1281]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1282]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1283]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1284]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1285]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1286]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1287]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[184] += + acadoWorkspace.H10[1288]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1289]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1290]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1291]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1292]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1293]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1294]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[185] += + acadoWorkspace.H10[1295]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1296]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1297]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1298]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1299]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1300]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1301]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[186] += + acadoWorkspace.H10[1302]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1303]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1304]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1305]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1306]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1307]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1308]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[187] += + acadoWorkspace.H10[1309]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1310]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1311]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1312]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1313]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1314]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1315]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[188] += + acadoWorkspace.H10[1316]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1317]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1318]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1319]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1320]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1321]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1322]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[189] += + acadoWorkspace.H10[1323]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1324]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1325]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1326]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1327]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1328]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1329]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[190] += + acadoWorkspace.H10[1330]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1331]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1332]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1333]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1334]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1335]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1336]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[191] += + acadoWorkspace.H10[1337]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1338]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1339]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1340]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1341]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1342]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1343]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[192] += + acadoWorkspace.H10[1344]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1345]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1346]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1347]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1348]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1349]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1350]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[193] += + acadoWorkspace.H10[1351]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1352]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1353]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1354]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1355]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1356]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1357]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[194] += + acadoWorkspace.H10[1358]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1359]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1360]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1361]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1362]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1363]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1364]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[195] += + acadoWorkspace.H10[1365]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1366]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1367]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1368]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1369]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1370]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1371]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[196] += + acadoWorkspace.H10[1372]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1373]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1374]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1375]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1376]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1377]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1378]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[197] += + acadoWorkspace.H10[1379]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1380]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1381]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1382]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1383]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1384]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1385]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[198] += + acadoWorkspace.H10[1386]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1387]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1388]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1389]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1390]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1391]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1392]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[199] += + acadoWorkspace.H10[1393]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1394]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1395]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1396]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1397]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1398]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1399]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[200] += + acadoWorkspace.H10[1400]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1401]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1402]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1403]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1404]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1405]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1406]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[201] += + acadoWorkspace.H10[1407]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1408]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1409]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1410]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1411]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1412]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1413]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[202] += + acadoWorkspace.H10[1414]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1415]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1416]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1417]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1418]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1419]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1420]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[203] += + acadoWorkspace.H10[1421]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1422]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1423]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1424]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1425]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1426]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1427]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[204] += + acadoWorkspace.H10[1428]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1429]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1430]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1431]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1432]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1433]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1434]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[205] += + acadoWorkspace.H10[1435]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1436]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1437]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1438]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1439]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1440]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1441]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[206] += + acadoWorkspace.H10[1442]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1443]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1444]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1445]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1446]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1447]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1448]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[207] += + acadoWorkspace.H10[1449]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1450]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1451]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1452]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1453]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1454]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1455]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[208] += + acadoWorkspace.H10[1456]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1457]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1458]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1459]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1460]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1461]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1462]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[209] += + acadoWorkspace.H10[1463]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1464]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1465]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1466]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1467]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1468]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1469]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[210] += + acadoWorkspace.H10[1470]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1471]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1472]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1473]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1474]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1475]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1476]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[211] += + acadoWorkspace.H10[1477]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1478]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1479]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1480]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1481]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1482]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1483]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[212] += + acadoWorkspace.H10[1484]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1485]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1486]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1487]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1488]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1489]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1490]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[213] += + acadoWorkspace.H10[1491]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1492]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1493]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1494]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1495]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1496]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1497]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[214] += + acadoWorkspace.H10[1498]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1499]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1500]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1501]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1502]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1503]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1504]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[215] += + acadoWorkspace.H10[1505]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1506]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1507]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1508]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1509]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1510]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1511]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[216] += + acadoWorkspace.H10[1512]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1513]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1514]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1515]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1516]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1517]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1518]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[217] += + acadoWorkspace.H10[1519]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1520]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1521]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1522]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1523]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1524]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1525]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[218] += + acadoWorkspace.H10[1526]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1527]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1528]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1529]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1530]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1531]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1532]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[219] += + acadoWorkspace.H10[1533]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1534]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1535]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1536]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1537]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1538]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1539]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[220] += + acadoWorkspace.H10[1540]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1541]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1542]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1543]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1544]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1545]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1546]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[221] += + acadoWorkspace.H10[1547]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1548]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1549]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1550]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1551]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1552]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1553]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[222] += + acadoWorkspace.H10[1554]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1555]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1556]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1557]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1558]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1559]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1560]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[223] += + acadoWorkspace.H10[1561]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1562]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1563]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1564]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1565]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1566]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1567]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[224] += + acadoWorkspace.H10[1568]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1569]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1570]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1571]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1572]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1573]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1574]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[225] += + acadoWorkspace.H10[1575]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1576]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1577]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1578]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1579]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1580]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1581]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[226] += + acadoWorkspace.H10[1582]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1583]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1584]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1585]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1586]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1587]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1588]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[227] += + acadoWorkspace.H10[1589]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1590]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1591]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1592]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1593]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1594]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1595]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[228] += + acadoWorkspace.H10[1596]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1597]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1598]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1599]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1600]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1601]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1602]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[229] += + acadoWorkspace.H10[1603]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1604]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1605]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1606]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1607]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1608]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1609]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[230] += + acadoWorkspace.H10[1610]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1611]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1612]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1613]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1614]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1615]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1616]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[231] += + acadoWorkspace.H10[1617]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1618]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1619]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1620]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1621]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1622]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1623]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[232] += + acadoWorkspace.H10[1624]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1625]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1626]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1627]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1628]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1629]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1630]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[233] += + acadoWorkspace.H10[1631]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1632]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1633]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1634]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1635]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1636]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1637]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[234] += + acadoWorkspace.H10[1638]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1639]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1640]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1641]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1642]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1643]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1644]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[235] += + acadoWorkspace.H10[1645]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1646]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1647]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1648]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1649]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1650]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1651]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[236] += + acadoWorkspace.H10[1652]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1653]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1654]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1655]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1656]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1657]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1658]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[237] += + acadoWorkspace.H10[1659]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1660]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1661]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1662]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1663]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1664]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1665]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[238] += + acadoWorkspace.H10[1666]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1667]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1668]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1669]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1670]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1671]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1672]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[239] += + acadoWorkspace.H10[1673]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1674]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1675]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1676]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1677]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1678]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1679]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[240] += + acadoWorkspace.H10[1680]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1681]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1682]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1683]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1684]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1685]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1686]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[241] += + acadoWorkspace.H10[1687]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1688]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1689]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1690]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1691]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1692]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1693]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[242] += + acadoWorkspace.H10[1694]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1695]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1696]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1697]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1698]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1699]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1700]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[243] += + acadoWorkspace.H10[1701]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1702]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1703]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1704]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1705]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1706]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1707]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[244] += + acadoWorkspace.H10[1708]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1709]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1710]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1711]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1712]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1713]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1714]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[245] += + acadoWorkspace.H10[1715]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1716]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1717]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1718]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1719]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1720]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1721]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[246] += + acadoWorkspace.H10[1722]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1723]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1724]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1725]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1726]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1727]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1728]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[247] += + acadoWorkspace.H10[1729]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1730]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1731]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1732]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1733]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1734]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1735]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[248] += + acadoWorkspace.H10[1736]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1737]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1738]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1739]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1740]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1741]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1742]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[249] += + acadoWorkspace.H10[1743]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1744]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1745]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1746]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1747]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1748]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1749]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[250] += + acadoWorkspace.H10[1750]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1751]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1752]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1753]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1754]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1755]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1756]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[251] += + acadoWorkspace.H10[1757]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1758]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1759]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1760]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1761]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1762]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1763]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[252] += + acadoWorkspace.H10[1764]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1765]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1766]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1767]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1768]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1769]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1770]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[253] += + acadoWorkspace.H10[1771]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1772]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1773]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1774]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1775]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1776]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1777]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[254] += + acadoWorkspace.H10[1778]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1779]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1780]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1781]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1782]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1783]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1784]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[255] += + acadoWorkspace.H10[1785]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1786]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1787]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1788]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1789]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1790]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1791]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[256] += + acadoWorkspace.H10[1792]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1793]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1794]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1795]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1796]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1797]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1798]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[257] += + acadoWorkspace.H10[1799]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1800]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1801]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1802]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1803]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1804]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1805]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[258] += + acadoWorkspace.H10[1806]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1807]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1808]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1809]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1810]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1811]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1812]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[259] += + acadoWorkspace.H10[1813]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1814]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1815]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1816]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1817]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1818]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1819]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[260] += + acadoWorkspace.H10[1820]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1821]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1822]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1823]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1824]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1825]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1826]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[261] += + acadoWorkspace.H10[1827]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1828]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1829]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1830]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1831]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1832]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1833]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[262] += + acadoWorkspace.H10[1834]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1835]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1836]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1837]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1838]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1839]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1840]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[263] += + acadoWorkspace.H10[1841]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1842]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1843]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1844]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1845]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1846]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1847]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[264] += + acadoWorkspace.H10[1848]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1849]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1850]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1851]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1852]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1853]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1854]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[265] += + acadoWorkspace.H10[1855]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1856]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1857]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1858]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1859]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1860]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1861]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[266] += + acadoWorkspace.H10[1862]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1863]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1864]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1865]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1866]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1867]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1868]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[267] += + acadoWorkspace.H10[1869]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1870]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1871]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1872]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1873]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1874]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1875]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[268] += + acadoWorkspace.H10[1876]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1877]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1878]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1879]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1880]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1881]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1882]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[269] += + acadoWorkspace.H10[1883]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1884]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1885]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1886]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1887]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1888]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1889]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[270] += + acadoWorkspace.H10[1890]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1891]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1892]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1893]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1894]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1895]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1896]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[271] += + acadoWorkspace.H10[1897]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1898]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1899]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1900]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1901]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1902]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1903]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[272] += + acadoWorkspace.H10[1904]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1905]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1906]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1907]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1908]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1909]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1910]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[273] += + acadoWorkspace.H10[1911]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1912]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1913]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1914]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1915]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1916]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1917]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[274] += + acadoWorkspace.H10[1918]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1919]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1920]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1921]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1922]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1923]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1924]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[275] += + acadoWorkspace.H10[1925]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1926]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1927]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1928]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1929]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1930]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1931]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[276] += + acadoWorkspace.H10[1932]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1933]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1934]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1935]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1936]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1937]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1938]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[277] += + acadoWorkspace.H10[1939]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1940]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1941]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1942]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1943]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1944]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1945]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[278] += + acadoWorkspace.H10[1946]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1947]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1948]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1949]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1950]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1951]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1952]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[279] += + acadoWorkspace.H10[1953]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1954]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1955]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1956]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1957]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1958]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1959]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[280] += + acadoWorkspace.H10[1960]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1961]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1962]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1963]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1964]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1965]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1966]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[281] += + acadoWorkspace.H10[1967]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1968]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1969]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1970]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1971]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1972]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1973]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[282] += + acadoWorkspace.H10[1974]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1975]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1976]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1977]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1978]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1979]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1980]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[283] += + acadoWorkspace.H10[1981]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1982]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1983]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1984]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1985]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1986]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1987]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[284] += + acadoWorkspace.H10[1988]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1989]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1990]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1991]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1992]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1993]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1994]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[285] += + acadoWorkspace.H10[1995]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1996]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1997]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1998]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1999]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[2000]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[2001]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[286] += + acadoWorkspace.H10[2002]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[2003]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2004]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[2005]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[2006]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[2007]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[2008]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[287] += + acadoWorkspace.H10[2009]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[2010]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2011]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[2012]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[2013]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[2014]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[2015]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[288] += + acadoWorkspace.H10[2016]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[2017]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2018]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[2019]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[2020]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[2021]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[2022]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[289] += + acadoWorkspace.H10[2023]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[2024]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2025]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[2026]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[2027]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[2028]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[2029]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[290] += + acadoWorkspace.H10[2030]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[2031]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2032]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[2033]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[2034]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[2035]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[2036]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[291] += + acadoWorkspace.H10[2037]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[2038]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2039]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[2040]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[2041]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[2042]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[2043]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[292] += + acadoWorkspace.H10[2044]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[2045]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2046]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[2047]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[2048]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[2049]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[2050]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[293] += + acadoWorkspace.H10[2051]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[2052]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2053]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[2054]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[2055]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[2056]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[2057]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[294] += + acadoWorkspace.H10[2058]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[2059]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2060]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[2061]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[2062]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[2063]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[2064]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[295] += + acadoWorkspace.H10[2065]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[2066]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2067]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[2068]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[2069]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[2070]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[2071]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[296] += + acadoWorkspace.H10[2072]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[2073]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2074]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[2075]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[2076]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[2077]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[2078]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[297] += + acadoWorkspace.H10[2079]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[2080]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2081]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[2082]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[2083]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[2084]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[2085]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[298] += + acadoWorkspace.H10[2086]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[2087]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2088]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[2089]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[2090]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[2091]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[2092]*acadoWorkspace.Dx0[6];
acadoWorkspace.g[299] += + acadoWorkspace.H10[2093]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[2094]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2095]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[2096]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[2097]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[2098]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[2099]*acadoWorkspace.Dx0[6];

acadoWorkspace.pacA01Dx0[0] = + acadoWorkspace.A01[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[5]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[6]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[1] = + acadoWorkspace.A01[7]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[8]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[9]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[10]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[11]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[12]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[13]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[2] = + acadoWorkspace.A01[14]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[15]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[16]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[17]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[18]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[19]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[20]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[3] = + acadoWorkspace.A01[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[23]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[24]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[25]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[26]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[27]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[4] = + acadoWorkspace.A01[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[30]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[31]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[32]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[33]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[34]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[5] = + acadoWorkspace.A01[35]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[36]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[37]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[38]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[39]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[40]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[41]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[6] = + acadoWorkspace.A01[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[44]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[45]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[46]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[47]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[48]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[7] = + acadoWorkspace.A01[49]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[50]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[51]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[52]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[53]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[54]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[55]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[8] = + acadoWorkspace.A01[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[59]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[60]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[61]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[62]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[9] = + acadoWorkspace.A01[63]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[64]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[65]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[66]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[67]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[68]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[69]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[10] = + acadoWorkspace.A01[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[74]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[75]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[76]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[11] = + acadoWorkspace.A01[77]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[78]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[79]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[80]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[81]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[82]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[83]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[12] = + acadoWorkspace.A01[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[88]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[89]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[90]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[13] = + acadoWorkspace.A01[91]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[92]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[93]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[94]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[95]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[96]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[97]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[14] = + acadoWorkspace.A01[98]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[99]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[100]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[101]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[102]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[103]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[104]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[15] = + acadoWorkspace.A01[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[108]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[109]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[110]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[111]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[16] = + acadoWorkspace.A01[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[115]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[116]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[117]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[118]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[17] = + acadoWorkspace.A01[119]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[120]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[121]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[122]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[123]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[124]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[125]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[18] = + acadoWorkspace.A01[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[128]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[129]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[130]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[131]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[132]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[19] = + acadoWorkspace.A01[133]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[134]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[135]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[136]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[137]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[138]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[139]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[20] = + acadoWorkspace.A01[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[144]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[145]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[146]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[21] = + acadoWorkspace.A01[147]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[148]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[149]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[150]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[151]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[152]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[153]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[22] = + acadoWorkspace.A01[154]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[155]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[156]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[157]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[158]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[159]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[160]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[23] = + acadoWorkspace.A01[161]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[162]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[163]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[164]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[165]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[166]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[167]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[24] = + acadoWorkspace.A01[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[172]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[173]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[174]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[25] = + acadoWorkspace.A01[175]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[176]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[177]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[178]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[179]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[180]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[181]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[26] = + acadoWorkspace.A01[182]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[183]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[184]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[185]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[186]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[187]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[188]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[27] = + acadoWorkspace.A01[189]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[190]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[191]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[192]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[193]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[194]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[195]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[28] = + acadoWorkspace.A01[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[197]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[198]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[199]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[200]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[201]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[202]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[29] = + acadoWorkspace.A01[203]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[204]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[205]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[206]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[207]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[208]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[209]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[30] = + acadoWorkspace.A01[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[214]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[215]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[216]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[31] = + acadoWorkspace.A01[217]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[218]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[219]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[220]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[221]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[222]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[223]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[32] = + acadoWorkspace.A01[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[227]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[228]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[229]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[230]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[33] = + acadoWorkspace.A01[231]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[232]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[233]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[234]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[235]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[236]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[237]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[34] = + acadoWorkspace.A01[238]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[239]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[240]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[241]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[242]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[243]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[244]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[35] = + acadoWorkspace.A01[245]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[246]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[247]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[248]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[249]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[250]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[251]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[36] = + acadoWorkspace.A01[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[256]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[257]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[258]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[37] = + acadoWorkspace.A01[259]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[260]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[261]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[262]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[263]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[264]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[265]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[38] = + acadoWorkspace.A01[266]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[267]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[268]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[269]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[270]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[271]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[272]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[39] = + acadoWorkspace.A01[273]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[274]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[275]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[276]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[277]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[278]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[279]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[40] = + acadoWorkspace.A01[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[284]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[285]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[286]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[41] = + acadoWorkspace.A01[287]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[288]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[289]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[290]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[291]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[292]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[293]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[42] = + acadoWorkspace.A01[294]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[295]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[296]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[297]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[298]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[299]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[300]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[43] = + acadoWorkspace.A01[301]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[302]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[303]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[304]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[305]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[306]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[307]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[44] = + acadoWorkspace.A01[308]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[309]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[310]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[311]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[312]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[313]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[314]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[45] = + acadoWorkspace.A01[315]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[316]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[317]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[318]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[319]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[320]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[321]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[46] = + acadoWorkspace.A01[322]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[323]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[324]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[325]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[326]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[327]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[328]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[47] = + acadoWorkspace.A01[329]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[330]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[331]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[332]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[333]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[334]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[335]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[48] = + acadoWorkspace.A01[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[339]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[340]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[341]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[342]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[49] = + acadoWorkspace.A01[343]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[344]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[345]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[346]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[347]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[348]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[349]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[50] = + acadoWorkspace.A01[350]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[351]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[352]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[353]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[354]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[355]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[356]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[51] = + acadoWorkspace.A01[357]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[358]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[359]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[360]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[361]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[362]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[363]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[52] = + acadoWorkspace.A01[364]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[365]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[366]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[367]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[368]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[369]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[370]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[53] = + acadoWorkspace.A01[371]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[372]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[373]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[374]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[375]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[376]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[377]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[54] = + acadoWorkspace.A01[378]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[379]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[380]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[381]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[382]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[383]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[384]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[55] = + acadoWorkspace.A01[385]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[386]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[387]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[388]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[389]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[390]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[391]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[56] = + acadoWorkspace.A01[392]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[393]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[394]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[395]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[396]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[397]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[398]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[57] = + acadoWorkspace.A01[399]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[400]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[401]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[402]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[403]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[404]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[405]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[58] = + acadoWorkspace.A01[406]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[407]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[408]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[409]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[410]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[411]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[412]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[59] = + acadoWorkspace.A01[413]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[414]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[415]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[416]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[417]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[418]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[419]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[60] = + acadoWorkspace.A01[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[424]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[425]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[426]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[61] = + acadoWorkspace.A01[427]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[428]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[429]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[430]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[431]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[432]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[433]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[62] = + acadoWorkspace.A01[434]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[435]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[436]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[437]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[438]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[439]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[440]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[63] = + acadoWorkspace.A01[441]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[442]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[443]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[444]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[445]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[446]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[447]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[64] = + acadoWorkspace.A01[448]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[449]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[450]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[451]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[452]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[453]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[454]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[65] = + acadoWorkspace.A01[455]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[456]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[457]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[458]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[459]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[460]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[461]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[66] = + acadoWorkspace.A01[462]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[463]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[464]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[465]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[466]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[467]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[468]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[67] = + acadoWorkspace.A01[469]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[470]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[471]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[472]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[473]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[474]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[475]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[68] = + acadoWorkspace.A01[476]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[477]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[478]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[479]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[480]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[481]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[482]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[69] = + acadoWorkspace.A01[483]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[484]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[485]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[486]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[487]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[488]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[489]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[70] = + acadoWorkspace.A01[490]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[491]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[492]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[493]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[494]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[495]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[496]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[71] = + acadoWorkspace.A01[497]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[498]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[499]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[500]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[501]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[502]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[503]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[72] = + acadoWorkspace.A01[504]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[505]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[506]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[507]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[508]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[509]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[510]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[73] = + acadoWorkspace.A01[511]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[512]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[513]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[514]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[515]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[516]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[517]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[74] = + acadoWorkspace.A01[518]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[519]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[520]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[521]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[522]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[523]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[524]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[75] = + acadoWorkspace.A01[525]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[526]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[527]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[528]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[529]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[530]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[531]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[76] = + acadoWorkspace.A01[532]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[533]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[534]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[535]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[536]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[537]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[538]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[77] = + acadoWorkspace.A01[539]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[540]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[541]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[542]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[543]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[544]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[545]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[78] = + acadoWorkspace.A01[546]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[547]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[548]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[549]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[550]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[551]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[552]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[79] = + acadoWorkspace.A01[553]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[554]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[555]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[556]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[557]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[558]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[559]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[80] = + acadoWorkspace.A01[560]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[561]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[562]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[563]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[564]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[565]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[566]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[81] = + acadoWorkspace.A01[567]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[568]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[569]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[570]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[571]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[572]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[573]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[82] = + acadoWorkspace.A01[574]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[575]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[576]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[577]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[578]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[579]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[580]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[83] = + acadoWorkspace.A01[581]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[582]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[583]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[584]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[585]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[586]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[587]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[84] = + acadoWorkspace.A01[588]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[589]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[590]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[591]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[592]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[593]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[594]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[85] = + acadoWorkspace.A01[595]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[596]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[597]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[598]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[599]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[600]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[601]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[86] = + acadoWorkspace.A01[602]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[603]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[604]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[605]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[606]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[607]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[608]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[87] = + acadoWorkspace.A01[609]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[610]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[611]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[612]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[613]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[614]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[615]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[88] = + acadoWorkspace.A01[616]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[617]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[618]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[619]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[620]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[621]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[622]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[89] = + acadoWorkspace.A01[623]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[624]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[625]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[626]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[627]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[628]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[629]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[90] = + acadoWorkspace.A01[630]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[631]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[632]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[633]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[634]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[635]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[636]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[91] = + acadoWorkspace.A01[637]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[638]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[639]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[640]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[641]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[642]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[643]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[92] = + acadoWorkspace.A01[644]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[645]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[646]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[647]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[648]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[649]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[650]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[93] = + acadoWorkspace.A01[651]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[652]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[653]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[654]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[655]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[656]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[657]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[94] = + acadoWorkspace.A01[658]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[659]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[660]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[661]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[662]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[663]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[664]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[95] = + acadoWorkspace.A01[665]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[666]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[667]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[668]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[669]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[670]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[671]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[96] = + acadoWorkspace.A01[672]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[673]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[674]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[675]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[676]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[677]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[678]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[97] = + acadoWorkspace.A01[679]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[680]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[681]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[682]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[683]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[684]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[685]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[98] = + acadoWorkspace.A01[686]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[687]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[688]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[689]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[690]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[691]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[692]*acadoWorkspace.Dx0[6];
acadoWorkspace.pacA01Dx0[99] = + acadoWorkspace.A01[693]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[694]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[695]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[696]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[697]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[698]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[699]*acadoWorkspace.Dx0[6];
acadoWorkspace.lbA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.lbA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.lbA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.lbA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.lbA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.lbA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.lbA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.lbA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.lbA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.lbA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.lbA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.lbA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.lbA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.lbA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.lbA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.lbA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.lbA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.lbA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.lbA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.lbA[19] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.lbA[20] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.lbA[21] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.lbA[22] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.lbA[23] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.lbA[24] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.lbA[25] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.lbA[26] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.lbA[27] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.lbA[28] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.lbA[29] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.lbA[30] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.lbA[31] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.lbA[32] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.lbA[33] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.lbA[34] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.lbA[35] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.lbA[36] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.lbA[37] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.lbA[38] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.lbA[39] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.lbA[40] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.lbA[41] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.lbA[42] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.lbA[43] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.lbA[44] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.lbA[45] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.lbA[46] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.lbA[47] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.lbA[48] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.lbA[49] -= acadoWorkspace.pacA01Dx0[49];
acadoWorkspace.lbA[50] -= acadoWorkspace.pacA01Dx0[50];
acadoWorkspace.lbA[51] -= acadoWorkspace.pacA01Dx0[51];
acadoWorkspace.lbA[52] -= acadoWorkspace.pacA01Dx0[52];
acadoWorkspace.lbA[53] -= acadoWorkspace.pacA01Dx0[53];
acadoWorkspace.lbA[54] -= acadoWorkspace.pacA01Dx0[54];
acadoWorkspace.lbA[55] -= acadoWorkspace.pacA01Dx0[55];
acadoWorkspace.lbA[56] -= acadoWorkspace.pacA01Dx0[56];
acadoWorkspace.lbA[57] -= acadoWorkspace.pacA01Dx0[57];
acadoWorkspace.lbA[58] -= acadoWorkspace.pacA01Dx0[58];
acadoWorkspace.lbA[59] -= acadoWorkspace.pacA01Dx0[59];
acadoWorkspace.lbA[60] -= acadoWorkspace.pacA01Dx0[60];
acadoWorkspace.lbA[61] -= acadoWorkspace.pacA01Dx0[61];
acadoWorkspace.lbA[62] -= acadoWorkspace.pacA01Dx0[62];
acadoWorkspace.lbA[63] -= acadoWorkspace.pacA01Dx0[63];
acadoWorkspace.lbA[64] -= acadoWorkspace.pacA01Dx0[64];
acadoWorkspace.lbA[65] -= acadoWorkspace.pacA01Dx0[65];
acadoWorkspace.lbA[66] -= acadoWorkspace.pacA01Dx0[66];
acadoWorkspace.lbA[67] -= acadoWorkspace.pacA01Dx0[67];
acadoWorkspace.lbA[68] -= acadoWorkspace.pacA01Dx0[68];
acadoWorkspace.lbA[69] -= acadoWorkspace.pacA01Dx0[69];
acadoWorkspace.lbA[70] -= acadoWorkspace.pacA01Dx0[70];
acadoWorkspace.lbA[71] -= acadoWorkspace.pacA01Dx0[71];
acadoWorkspace.lbA[72] -= acadoWorkspace.pacA01Dx0[72];
acadoWorkspace.lbA[73] -= acadoWorkspace.pacA01Dx0[73];
acadoWorkspace.lbA[74] -= acadoWorkspace.pacA01Dx0[74];
acadoWorkspace.lbA[75] -= acadoWorkspace.pacA01Dx0[75];
acadoWorkspace.lbA[76] -= acadoWorkspace.pacA01Dx0[76];
acadoWorkspace.lbA[77] -= acadoWorkspace.pacA01Dx0[77];
acadoWorkspace.lbA[78] -= acadoWorkspace.pacA01Dx0[78];
acadoWorkspace.lbA[79] -= acadoWorkspace.pacA01Dx0[79];
acadoWorkspace.lbA[80] -= acadoWorkspace.pacA01Dx0[80];
acadoWorkspace.lbA[81] -= acadoWorkspace.pacA01Dx0[81];
acadoWorkspace.lbA[82] -= acadoWorkspace.pacA01Dx0[82];
acadoWorkspace.lbA[83] -= acadoWorkspace.pacA01Dx0[83];
acadoWorkspace.lbA[84] -= acadoWorkspace.pacA01Dx0[84];
acadoWorkspace.lbA[85] -= acadoWorkspace.pacA01Dx0[85];
acadoWorkspace.lbA[86] -= acadoWorkspace.pacA01Dx0[86];
acadoWorkspace.lbA[87] -= acadoWorkspace.pacA01Dx0[87];
acadoWorkspace.lbA[88] -= acadoWorkspace.pacA01Dx0[88];
acadoWorkspace.lbA[89] -= acadoWorkspace.pacA01Dx0[89];
acadoWorkspace.lbA[90] -= acadoWorkspace.pacA01Dx0[90];
acadoWorkspace.lbA[91] -= acadoWorkspace.pacA01Dx0[91];
acadoWorkspace.lbA[92] -= acadoWorkspace.pacA01Dx0[92];
acadoWorkspace.lbA[93] -= acadoWorkspace.pacA01Dx0[93];
acadoWorkspace.lbA[94] -= acadoWorkspace.pacA01Dx0[94];
acadoWorkspace.lbA[95] -= acadoWorkspace.pacA01Dx0[95];
acadoWorkspace.lbA[96] -= acadoWorkspace.pacA01Dx0[96];
acadoWorkspace.lbA[97] -= acadoWorkspace.pacA01Dx0[97];
acadoWorkspace.lbA[98] -= acadoWorkspace.pacA01Dx0[98];
acadoWorkspace.lbA[99] -= acadoWorkspace.pacA01Dx0[99];

acadoWorkspace.ubA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.ubA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.ubA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.ubA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.ubA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.ubA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.ubA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.ubA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.ubA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.ubA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.ubA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.ubA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.ubA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.ubA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.ubA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.ubA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.ubA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.ubA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.ubA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.ubA[19] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.ubA[20] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.ubA[21] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.ubA[22] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.ubA[23] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.ubA[24] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.ubA[25] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.ubA[26] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.ubA[27] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.ubA[28] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.ubA[29] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.ubA[30] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.ubA[31] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.ubA[32] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.ubA[33] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.ubA[34] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.ubA[35] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.ubA[36] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.ubA[37] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.ubA[38] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.ubA[39] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.ubA[40] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.ubA[41] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.ubA[42] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.ubA[43] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.ubA[44] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.ubA[45] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.ubA[46] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.ubA[47] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.ubA[48] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.ubA[49] -= acadoWorkspace.pacA01Dx0[49];
acadoWorkspace.ubA[50] -= acadoWorkspace.pacA01Dx0[50];
acadoWorkspace.ubA[51] -= acadoWorkspace.pacA01Dx0[51];
acadoWorkspace.ubA[52] -= acadoWorkspace.pacA01Dx0[52];
acadoWorkspace.ubA[53] -= acadoWorkspace.pacA01Dx0[53];
acadoWorkspace.ubA[54] -= acadoWorkspace.pacA01Dx0[54];
acadoWorkspace.ubA[55] -= acadoWorkspace.pacA01Dx0[55];
acadoWorkspace.ubA[56] -= acadoWorkspace.pacA01Dx0[56];
acadoWorkspace.ubA[57] -= acadoWorkspace.pacA01Dx0[57];
acadoWorkspace.ubA[58] -= acadoWorkspace.pacA01Dx0[58];
acadoWorkspace.ubA[59] -= acadoWorkspace.pacA01Dx0[59];
acadoWorkspace.ubA[60] -= acadoWorkspace.pacA01Dx0[60];
acadoWorkspace.ubA[61] -= acadoWorkspace.pacA01Dx0[61];
acadoWorkspace.ubA[62] -= acadoWorkspace.pacA01Dx0[62];
acadoWorkspace.ubA[63] -= acadoWorkspace.pacA01Dx0[63];
acadoWorkspace.ubA[64] -= acadoWorkspace.pacA01Dx0[64];
acadoWorkspace.ubA[65] -= acadoWorkspace.pacA01Dx0[65];
acadoWorkspace.ubA[66] -= acadoWorkspace.pacA01Dx0[66];
acadoWorkspace.ubA[67] -= acadoWorkspace.pacA01Dx0[67];
acadoWorkspace.ubA[68] -= acadoWorkspace.pacA01Dx0[68];
acadoWorkspace.ubA[69] -= acadoWorkspace.pacA01Dx0[69];
acadoWorkspace.ubA[70] -= acadoWorkspace.pacA01Dx0[70];
acadoWorkspace.ubA[71] -= acadoWorkspace.pacA01Dx0[71];
acadoWorkspace.ubA[72] -= acadoWorkspace.pacA01Dx0[72];
acadoWorkspace.ubA[73] -= acadoWorkspace.pacA01Dx0[73];
acadoWorkspace.ubA[74] -= acadoWorkspace.pacA01Dx0[74];
acadoWorkspace.ubA[75] -= acadoWorkspace.pacA01Dx0[75];
acadoWorkspace.ubA[76] -= acadoWorkspace.pacA01Dx0[76];
acadoWorkspace.ubA[77] -= acadoWorkspace.pacA01Dx0[77];
acadoWorkspace.ubA[78] -= acadoWorkspace.pacA01Dx0[78];
acadoWorkspace.ubA[79] -= acadoWorkspace.pacA01Dx0[79];
acadoWorkspace.ubA[80] -= acadoWorkspace.pacA01Dx0[80];
acadoWorkspace.ubA[81] -= acadoWorkspace.pacA01Dx0[81];
acadoWorkspace.ubA[82] -= acadoWorkspace.pacA01Dx0[82];
acadoWorkspace.ubA[83] -= acadoWorkspace.pacA01Dx0[83];
acadoWorkspace.ubA[84] -= acadoWorkspace.pacA01Dx0[84];
acadoWorkspace.ubA[85] -= acadoWorkspace.pacA01Dx0[85];
acadoWorkspace.ubA[86] -= acadoWorkspace.pacA01Dx0[86];
acadoWorkspace.ubA[87] -= acadoWorkspace.pacA01Dx0[87];
acadoWorkspace.ubA[88] -= acadoWorkspace.pacA01Dx0[88];
acadoWorkspace.ubA[89] -= acadoWorkspace.pacA01Dx0[89];
acadoWorkspace.ubA[90] -= acadoWorkspace.pacA01Dx0[90];
acadoWorkspace.ubA[91] -= acadoWorkspace.pacA01Dx0[91];
acadoWorkspace.ubA[92] -= acadoWorkspace.pacA01Dx0[92];
acadoWorkspace.ubA[93] -= acadoWorkspace.pacA01Dx0[93];
acadoWorkspace.ubA[94] -= acadoWorkspace.pacA01Dx0[94];
acadoWorkspace.ubA[95] -= acadoWorkspace.pacA01Dx0[95];
acadoWorkspace.ubA[96] -= acadoWorkspace.pacA01Dx0[96];
acadoWorkspace.ubA[97] -= acadoWorkspace.pacA01Dx0[97];
acadoWorkspace.ubA[98] -= acadoWorkspace.pacA01Dx0[98];
acadoWorkspace.ubA[99] -= acadoWorkspace.pacA01Dx0[99];

}

void acado_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun1 = 0; lRun1 < 300; ++lRun1)
acadoVariables.u[lRun1] += acadoWorkspace.x[lRun1];


acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];
acadoVariables.x[4] += acadoWorkspace.Dx0[4];
acadoVariables.x[5] += acadoWorkspace.Dx0[5];
acadoVariables.x[6] += acadoWorkspace.Dx0[6];

for (lRun1 = 0; lRun1 < 700; ++lRun1)
{
for (lRun2 = 0; lRun2 < 1; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 7; ++lRun3)
{
t += + acadoWorkspace.evGx[(lRun1 * 7) + (lRun3)]*acadoWorkspace.Dx0[(lRun3) + (lRun2)];
}
acadoVariables.x[(lRun1 + 7) + (lRun2)] += t + acadoWorkspace.d[(lRun1) + (lRun2)];
}
}

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multEDu( &(acadoWorkspace.E[ lRun3 * 21 ]), &(acadoWorkspace.x[ lRun2 * 3 ]), &(acadoVariables.x[ lRun1 * 7 + 7 ]) );
}
}
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
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 100; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 7];
acadoWorkspace.state[1] = acadoVariables.x[index * 7 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 7 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 7 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 7 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 7 + 5];
acadoWorkspace.state[6] = acadoVariables.x[index * 7 + 6];
acadoWorkspace.state[77] = acadoVariables.u[index * 3];
acadoWorkspace.state[78] = acadoVariables.u[index * 3 + 1];
acadoWorkspace.state[79] = acadoVariables.u[index * 3 + 2];
acadoWorkspace.state[80] = acadoVariables.od[index * 9];
acadoWorkspace.state[81] = acadoVariables.od[index * 9 + 1];
acadoWorkspace.state[82] = acadoVariables.od[index * 9 + 2];
acadoWorkspace.state[83] = acadoVariables.od[index * 9 + 3];
acadoWorkspace.state[84] = acadoVariables.od[index * 9 + 4];
acadoWorkspace.state[85] = acadoVariables.od[index * 9 + 5];
acadoWorkspace.state[86] = acadoVariables.od[index * 9 + 6];
acadoWorkspace.state[87] = acadoVariables.od[index * 9 + 7];
acadoWorkspace.state[88] = acadoVariables.od[index * 9 + 8];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 7 + 7] = acadoWorkspace.state[0];
acadoVariables.x[index * 7 + 8] = acadoWorkspace.state[1];
acadoVariables.x[index * 7 + 9] = acadoWorkspace.state[2];
acadoVariables.x[index * 7 + 10] = acadoWorkspace.state[3];
acadoVariables.x[index * 7 + 11] = acadoWorkspace.state[4];
acadoVariables.x[index * 7 + 12] = acadoWorkspace.state[5];
acadoVariables.x[index * 7 + 13] = acadoWorkspace.state[6];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 100; ++index)
{
acadoVariables.x[index * 7] = acadoVariables.x[index * 7 + 7];
acadoVariables.x[index * 7 + 1] = acadoVariables.x[index * 7 + 8];
acadoVariables.x[index * 7 + 2] = acadoVariables.x[index * 7 + 9];
acadoVariables.x[index * 7 + 3] = acadoVariables.x[index * 7 + 10];
acadoVariables.x[index * 7 + 4] = acadoVariables.x[index * 7 + 11];
acadoVariables.x[index * 7 + 5] = acadoVariables.x[index * 7 + 12];
acadoVariables.x[index * 7 + 6] = acadoVariables.x[index * 7 + 13];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[700] = xEnd[0];
acadoVariables.x[701] = xEnd[1];
acadoVariables.x[702] = xEnd[2];
acadoVariables.x[703] = xEnd[3];
acadoVariables.x[704] = xEnd[4];
acadoVariables.x[705] = xEnd[5];
acadoVariables.x[706] = xEnd[6];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[700];
acadoWorkspace.state[1] = acadoVariables.x[701];
acadoWorkspace.state[2] = acadoVariables.x[702];
acadoWorkspace.state[3] = acadoVariables.x[703];
acadoWorkspace.state[4] = acadoVariables.x[704];
acadoWorkspace.state[5] = acadoVariables.x[705];
acadoWorkspace.state[6] = acadoVariables.x[706];
if (uEnd != 0)
{
acadoWorkspace.state[77] = uEnd[0];
acadoWorkspace.state[78] = uEnd[1];
acadoWorkspace.state[79] = uEnd[2];
}
else
{
acadoWorkspace.state[77] = acadoVariables.u[297];
acadoWorkspace.state[78] = acadoVariables.u[298];
acadoWorkspace.state[79] = acadoVariables.u[299];
}
acadoWorkspace.state[80] = acadoVariables.od[900];
acadoWorkspace.state[81] = acadoVariables.od[901];
acadoWorkspace.state[82] = acadoVariables.od[902];
acadoWorkspace.state[83] = acadoVariables.od[903];
acadoWorkspace.state[84] = acadoVariables.od[904];
acadoWorkspace.state[85] = acadoVariables.od[905];
acadoWorkspace.state[86] = acadoVariables.od[906];
acadoWorkspace.state[87] = acadoVariables.od[907];
acadoWorkspace.state[88] = acadoVariables.od[908];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[700] = acadoWorkspace.state[0];
acadoVariables.x[701] = acadoWorkspace.state[1];
acadoVariables.x[702] = acadoWorkspace.state[2];
acadoVariables.x[703] = acadoWorkspace.state[3];
acadoVariables.x[704] = acadoWorkspace.state[4];
acadoVariables.x[705] = acadoWorkspace.state[5];
acadoVariables.x[706] = acadoWorkspace.state[6];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 99; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[297] = uEnd[0];
acadoVariables.u[298] = uEnd[1];
acadoVariables.u[299] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89] + acadoWorkspace.g[90]*acadoWorkspace.x[90] + acadoWorkspace.g[91]*acadoWorkspace.x[91] + acadoWorkspace.g[92]*acadoWorkspace.x[92] + acadoWorkspace.g[93]*acadoWorkspace.x[93] + acadoWorkspace.g[94]*acadoWorkspace.x[94] + acadoWorkspace.g[95]*acadoWorkspace.x[95] + acadoWorkspace.g[96]*acadoWorkspace.x[96] + acadoWorkspace.g[97]*acadoWorkspace.x[97] + acadoWorkspace.g[98]*acadoWorkspace.x[98] + acadoWorkspace.g[99]*acadoWorkspace.x[99] + acadoWorkspace.g[100]*acadoWorkspace.x[100] + acadoWorkspace.g[101]*acadoWorkspace.x[101] + acadoWorkspace.g[102]*acadoWorkspace.x[102] + acadoWorkspace.g[103]*acadoWorkspace.x[103] + acadoWorkspace.g[104]*acadoWorkspace.x[104] + acadoWorkspace.g[105]*acadoWorkspace.x[105] + acadoWorkspace.g[106]*acadoWorkspace.x[106] + acadoWorkspace.g[107]*acadoWorkspace.x[107] + acadoWorkspace.g[108]*acadoWorkspace.x[108] + acadoWorkspace.g[109]*acadoWorkspace.x[109] + acadoWorkspace.g[110]*acadoWorkspace.x[110] + acadoWorkspace.g[111]*acadoWorkspace.x[111] + acadoWorkspace.g[112]*acadoWorkspace.x[112] + acadoWorkspace.g[113]*acadoWorkspace.x[113] + acadoWorkspace.g[114]*acadoWorkspace.x[114] + acadoWorkspace.g[115]*acadoWorkspace.x[115] + acadoWorkspace.g[116]*acadoWorkspace.x[116] + acadoWorkspace.g[117]*acadoWorkspace.x[117] + acadoWorkspace.g[118]*acadoWorkspace.x[118] + acadoWorkspace.g[119]*acadoWorkspace.x[119] + acadoWorkspace.g[120]*acadoWorkspace.x[120] + acadoWorkspace.g[121]*acadoWorkspace.x[121] + acadoWorkspace.g[122]*acadoWorkspace.x[122] + acadoWorkspace.g[123]*acadoWorkspace.x[123] + acadoWorkspace.g[124]*acadoWorkspace.x[124] + acadoWorkspace.g[125]*acadoWorkspace.x[125] + acadoWorkspace.g[126]*acadoWorkspace.x[126] + acadoWorkspace.g[127]*acadoWorkspace.x[127] + acadoWorkspace.g[128]*acadoWorkspace.x[128] + acadoWorkspace.g[129]*acadoWorkspace.x[129] + acadoWorkspace.g[130]*acadoWorkspace.x[130] + acadoWorkspace.g[131]*acadoWorkspace.x[131] + acadoWorkspace.g[132]*acadoWorkspace.x[132] + acadoWorkspace.g[133]*acadoWorkspace.x[133] + acadoWorkspace.g[134]*acadoWorkspace.x[134] + acadoWorkspace.g[135]*acadoWorkspace.x[135] + acadoWorkspace.g[136]*acadoWorkspace.x[136] + acadoWorkspace.g[137]*acadoWorkspace.x[137] + acadoWorkspace.g[138]*acadoWorkspace.x[138] + acadoWorkspace.g[139]*acadoWorkspace.x[139] + acadoWorkspace.g[140]*acadoWorkspace.x[140] + acadoWorkspace.g[141]*acadoWorkspace.x[141] + acadoWorkspace.g[142]*acadoWorkspace.x[142] + acadoWorkspace.g[143]*acadoWorkspace.x[143] + acadoWorkspace.g[144]*acadoWorkspace.x[144] + acadoWorkspace.g[145]*acadoWorkspace.x[145] + acadoWorkspace.g[146]*acadoWorkspace.x[146] + acadoWorkspace.g[147]*acadoWorkspace.x[147] + acadoWorkspace.g[148]*acadoWorkspace.x[148] + acadoWorkspace.g[149]*acadoWorkspace.x[149] + acadoWorkspace.g[150]*acadoWorkspace.x[150] + acadoWorkspace.g[151]*acadoWorkspace.x[151] + acadoWorkspace.g[152]*acadoWorkspace.x[152] + acadoWorkspace.g[153]*acadoWorkspace.x[153] + acadoWorkspace.g[154]*acadoWorkspace.x[154] + acadoWorkspace.g[155]*acadoWorkspace.x[155] + acadoWorkspace.g[156]*acadoWorkspace.x[156] + acadoWorkspace.g[157]*acadoWorkspace.x[157] + acadoWorkspace.g[158]*acadoWorkspace.x[158] + acadoWorkspace.g[159]*acadoWorkspace.x[159] + acadoWorkspace.g[160]*acadoWorkspace.x[160] + acadoWorkspace.g[161]*acadoWorkspace.x[161] + acadoWorkspace.g[162]*acadoWorkspace.x[162] + acadoWorkspace.g[163]*acadoWorkspace.x[163] + acadoWorkspace.g[164]*acadoWorkspace.x[164] + acadoWorkspace.g[165]*acadoWorkspace.x[165] + acadoWorkspace.g[166]*acadoWorkspace.x[166] + acadoWorkspace.g[167]*acadoWorkspace.x[167] + acadoWorkspace.g[168]*acadoWorkspace.x[168] + acadoWorkspace.g[169]*acadoWorkspace.x[169] + acadoWorkspace.g[170]*acadoWorkspace.x[170] + acadoWorkspace.g[171]*acadoWorkspace.x[171] + acadoWorkspace.g[172]*acadoWorkspace.x[172] + acadoWorkspace.g[173]*acadoWorkspace.x[173] + acadoWorkspace.g[174]*acadoWorkspace.x[174] + acadoWorkspace.g[175]*acadoWorkspace.x[175] + acadoWorkspace.g[176]*acadoWorkspace.x[176] + acadoWorkspace.g[177]*acadoWorkspace.x[177] + acadoWorkspace.g[178]*acadoWorkspace.x[178] + acadoWorkspace.g[179]*acadoWorkspace.x[179] + acadoWorkspace.g[180]*acadoWorkspace.x[180] + acadoWorkspace.g[181]*acadoWorkspace.x[181] + acadoWorkspace.g[182]*acadoWorkspace.x[182] + acadoWorkspace.g[183]*acadoWorkspace.x[183] + acadoWorkspace.g[184]*acadoWorkspace.x[184] + acadoWorkspace.g[185]*acadoWorkspace.x[185] + acadoWorkspace.g[186]*acadoWorkspace.x[186] + acadoWorkspace.g[187]*acadoWorkspace.x[187] + acadoWorkspace.g[188]*acadoWorkspace.x[188] + acadoWorkspace.g[189]*acadoWorkspace.x[189] + acadoWorkspace.g[190]*acadoWorkspace.x[190] + acadoWorkspace.g[191]*acadoWorkspace.x[191] + acadoWorkspace.g[192]*acadoWorkspace.x[192] + acadoWorkspace.g[193]*acadoWorkspace.x[193] + acadoWorkspace.g[194]*acadoWorkspace.x[194] + acadoWorkspace.g[195]*acadoWorkspace.x[195] + acadoWorkspace.g[196]*acadoWorkspace.x[196] + acadoWorkspace.g[197]*acadoWorkspace.x[197] + acadoWorkspace.g[198]*acadoWorkspace.x[198] + acadoWorkspace.g[199]*acadoWorkspace.x[199] + acadoWorkspace.g[200]*acadoWorkspace.x[200] + acadoWorkspace.g[201]*acadoWorkspace.x[201] + acadoWorkspace.g[202]*acadoWorkspace.x[202] + acadoWorkspace.g[203]*acadoWorkspace.x[203] + acadoWorkspace.g[204]*acadoWorkspace.x[204] + acadoWorkspace.g[205]*acadoWorkspace.x[205] + acadoWorkspace.g[206]*acadoWorkspace.x[206] + acadoWorkspace.g[207]*acadoWorkspace.x[207] + acadoWorkspace.g[208]*acadoWorkspace.x[208] + acadoWorkspace.g[209]*acadoWorkspace.x[209] + acadoWorkspace.g[210]*acadoWorkspace.x[210] + acadoWorkspace.g[211]*acadoWorkspace.x[211] + acadoWorkspace.g[212]*acadoWorkspace.x[212] + acadoWorkspace.g[213]*acadoWorkspace.x[213] + acadoWorkspace.g[214]*acadoWorkspace.x[214] + acadoWorkspace.g[215]*acadoWorkspace.x[215] + acadoWorkspace.g[216]*acadoWorkspace.x[216] + acadoWorkspace.g[217]*acadoWorkspace.x[217] + acadoWorkspace.g[218]*acadoWorkspace.x[218] + acadoWorkspace.g[219]*acadoWorkspace.x[219] + acadoWorkspace.g[220]*acadoWorkspace.x[220] + acadoWorkspace.g[221]*acadoWorkspace.x[221] + acadoWorkspace.g[222]*acadoWorkspace.x[222] + acadoWorkspace.g[223]*acadoWorkspace.x[223] + acadoWorkspace.g[224]*acadoWorkspace.x[224] + acadoWorkspace.g[225]*acadoWorkspace.x[225] + acadoWorkspace.g[226]*acadoWorkspace.x[226] + acadoWorkspace.g[227]*acadoWorkspace.x[227] + acadoWorkspace.g[228]*acadoWorkspace.x[228] + acadoWorkspace.g[229]*acadoWorkspace.x[229] + acadoWorkspace.g[230]*acadoWorkspace.x[230] + acadoWorkspace.g[231]*acadoWorkspace.x[231] + acadoWorkspace.g[232]*acadoWorkspace.x[232] + acadoWorkspace.g[233]*acadoWorkspace.x[233] + acadoWorkspace.g[234]*acadoWorkspace.x[234] + acadoWorkspace.g[235]*acadoWorkspace.x[235] + acadoWorkspace.g[236]*acadoWorkspace.x[236] + acadoWorkspace.g[237]*acadoWorkspace.x[237] + acadoWorkspace.g[238]*acadoWorkspace.x[238] + acadoWorkspace.g[239]*acadoWorkspace.x[239] + acadoWorkspace.g[240]*acadoWorkspace.x[240] + acadoWorkspace.g[241]*acadoWorkspace.x[241] + acadoWorkspace.g[242]*acadoWorkspace.x[242] + acadoWorkspace.g[243]*acadoWorkspace.x[243] + acadoWorkspace.g[244]*acadoWorkspace.x[244] + acadoWorkspace.g[245]*acadoWorkspace.x[245] + acadoWorkspace.g[246]*acadoWorkspace.x[246] + acadoWorkspace.g[247]*acadoWorkspace.x[247] + acadoWorkspace.g[248]*acadoWorkspace.x[248] + acadoWorkspace.g[249]*acadoWorkspace.x[249] + acadoWorkspace.g[250]*acadoWorkspace.x[250] + acadoWorkspace.g[251]*acadoWorkspace.x[251] + acadoWorkspace.g[252]*acadoWorkspace.x[252] + acadoWorkspace.g[253]*acadoWorkspace.x[253] + acadoWorkspace.g[254]*acadoWorkspace.x[254] + acadoWorkspace.g[255]*acadoWorkspace.x[255] + acadoWorkspace.g[256]*acadoWorkspace.x[256] + acadoWorkspace.g[257]*acadoWorkspace.x[257] + acadoWorkspace.g[258]*acadoWorkspace.x[258] + acadoWorkspace.g[259]*acadoWorkspace.x[259] + acadoWorkspace.g[260]*acadoWorkspace.x[260] + acadoWorkspace.g[261]*acadoWorkspace.x[261] + acadoWorkspace.g[262]*acadoWorkspace.x[262] + acadoWorkspace.g[263]*acadoWorkspace.x[263] + acadoWorkspace.g[264]*acadoWorkspace.x[264] + acadoWorkspace.g[265]*acadoWorkspace.x[265] + acadoWorkspace.g[266]*acadoWorkspace.x[266] + acadoWorkspace.g[267]*acadoWorkspace.x[267] + acadoWorkspace.g[268]*acadoWorkspace.x[268] + acadoWorkspace.g[269]*acadoWorkspace.x[269] + acadoWorkspace.g[270]*acadoWorkspace.x[270] + acadoWorkspace.g[271]*acadoWorkspace.x[271] + acadoWorkspace.g[272]*acadoWorkspace.x[272] + acadoWorkspace.g[273]*acadoWorkspace.x[273] + acadoWorkspace.g[274]*acadoWorkspace.x[274] + acadoWorkspace.g[275]*acadoWorkspace.x[275] + acadoWorkspace.g[276]*acadoWorkspace.x[276] + acadoWorkspace.g[277]*acadoWorkspace.x[277] + acadoWorkspace.g[278]*acadoWorkspace.x[278] + acadoWorkspace.g[279]*acadoWorkspace.x[279] + acadoWorkspace.g[280]*acadoWorkspace.x[280] + acadoWorkspace.g[281]*acadoWorkspace.x[281] + acadoWorkspace.g[282]*acadoWorkspace.x[282] + acadoWorkspace.g[283]*acadoWorkspace.x[283] + acadoWorkspace.g[284]*acadoWorkspace.x[284] + acadoWorkspace.g[285]*acadoWorkspace.x[285] + acadoWorkspace.g[286]*acadoWorkspace.x[286] + acadoWorkspace.g[287]*acadoWorkspace.x[287] + acadoWorkspace.g[288]*acadoWorkspace.x[288] + acadoWorkspace.g[289]*acadoWorkspace.x[289] + acadoWorkspace.g[290]*acadoWorkspace.x[290] + acadoWorkspace.g[291]*acadoWorkspace.x[291] + acadoWorkspace.g[292]*acadoWorkspace.x[292] + acadoWorkspace.g[293]*acadoWorkspace.x[293] + acadoWorkspace.g[294]*acadoWorkspace.x[294] + acadoWorkspace.g[295]*acadoWorkspace.x[295] + acadoWorkspace.g[296]*acadoWorkspace.x[296] + acadoWorkspace.g[297]*acadoWorkspace.x[297] + acadoWorkspace.g[298]*acadoWorkspace.x[298] + acadoWorkspace.g[299]*acadoWorkspace.x[299];
kkt = fabs( kkt );
for (index = 0; index < 300; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 100; ++index)
{
prd = acadoWorkspace.y[index + 300];
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
/** Row vector of size: 5 */
real_t tmpDy[ 5 ];

/** Row vector of size: 2 */
real_t tmpDyN[ 2 ];

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 7];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 7 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 7 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 7 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 7 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 7 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 7 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.objValueIn[8] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[9] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 9];
acadoWorkspace.objValueIn[11] = acadoVariables.od[lRun1 * 9 + 1];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 9 + 2];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 9 + 3];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 9 + 4];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 9 + 5];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 9 + 6];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 9 + 7];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 9 + 8];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 5] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 5];
acadoWorkspace.Dy[lRun1 * 5 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 5 + 1];
acadoWorkspace.Dy[lRun1 * 5 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 5 + 2];
acadoWorkspace.Dy[lRun1 * 5 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 5 + 3];
acadoWorkspace.Dy[lRun1 * 5 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 5 + 4];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[700];
acadoWorkspace.objValueIn[1] = acadoVariables.x[701];
acadoWorkspace.objValueIn[2] = acadoVariables.x[702];
acadoWorkspace.objValueIn[3] = acadoVariables.x[703];
acadoWorkspace.objValueIn[4] = acadoVariables.x[704];
acadoWorkspace.objValueIn[5] = acadoVariables.x[705];
acadoWorkspace.objValueIn[6] = acadoVariables.x[706];
acadoWorkspace.objValueIn[7] = acadoVariables.od[900];
acadoWorkspace.objValueIn[8] = acadoVariables.od[901];
acadoWorkspace.objValueIn[9] = acadoVariables.od[902];
acadoWorkspace.objValueIn[10] = acadoVariables.od[903];
acadoWorkspace.objValueIn[11] = acadoVariables.od[904];
acadoWorkspace.objValueIn[12] = acadoVariables.od[905];
acadoWorkspace.objValueIn[13] = acadoVariables.od[906];
acadoWorkspace.objValueIn[14] = acadoVariables.od[907];
acadoWorkspace.objValueIn[15] = acadoVariables.od[908];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[lRun1 * 25] + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[lRun1 * 25 + 5] + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[lRun1 * 25 + 10] + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[lRun1 * 25 + 15] + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[lRun1 * 25 + 20];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[lRun1 * 25 + 1] + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[lRun1 * 25 + 6] + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[lRun1 * 25 + 11] + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[lRun1 * 25 + 16] + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[lRun1 * 25 + 21];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[lRun1 * 25 + 2] + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[lRun1 * 25 + 7] + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[lRun1 * 25 + 12] + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[lRun1 * 25 + 17] + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[lRun1 * 25 + 22];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[lRun1 * 25 + 3] + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[lRun1 * 25 + 8] + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[lRun1 * 25 + 13] + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[lRun1 * 25 + 18] + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[lRun1 * 25 + 23];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[lRun1 * 25 + 4] + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[lRun1 * 25 + 9] + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[lRun1 * 25 + 14] + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[lRun1 * 25 + 19] + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[lRun1 * 25 + 24];
objVal += + acadoWorkspace.Dy[lRun1 * 5]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 5 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 5 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 5 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 5 + 4]*tmpDy[4];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[3];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1];

objVal *= 0.5;
return objVal;
}

