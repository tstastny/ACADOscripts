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


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 5;
const real_t* od = in + 6;
/* Vector of auxiliary variables; number of elements: 3. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (cos(xd[3]));
a[1] = (sin(xd[3]));
a[2] = (tan(xd[2]));

/* Compute outputs: */
out[0] = ((od[0]*a[0])+od[5]);
out[1] = ((od[0]*a[1])+od[6]);
out[2] = xd[4];
out[3] = (((real_t)(9.8100000000000005e+00)/od[0])*a[2]);
out[4] = ((((real_t)(1.3480000000000000e+01)*u[0])-((real_t)(1.3970000000000001e+01)*xd[2]))-((real_t)(6.5770000000000000e+00)*xd[4]));
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 6;
/* Vector of auxiliary variables; number of elements: 3. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[1] = (cos(xd[3]));
a[2] = ((real_t)(1.0000000000000000e+00)/(pow((cos(xd[2])),2)));

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (od[0]*a[0]);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (od[0]*a[1]);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(1.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (((real_t)(9.8100000000000005e+00)/od[0])*a[2]);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.3970000000000001e+01));
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = ((real_t)(0.0000000000000000e+00)-(real_t)(6.5770000000000000e+00));
out[29] = (real_t)(1.3480000000000000e+01);
}



void acado_solve_dim10_triangular( real_t* const A, real_t* const b )
{

b[9] = b[9]/A[99];
b[8] -= + A[89]*b[9];
b[8] = b[8]/A[88];
b[7] -= + A[79]*b[9];
b[7] -= + A[78]*b[8];
b[7] = b[7]/A[77];
b[6] -= + A[69]*b[9];
b[6] -= + A[68]*b[8];
b[6] -= + A[67]*b[7];
b[6] = b[6]/A[66];
b[5] -= + A[59]*b[9];
b[5] -= + A[58]*b[8];
b[5] -= + A[57]*b[7];
b[5] -= + A[56]*b[6];
b[5] = b[5]/A[55];
b[4] -= + A[49]*b[9];
b[4] -= + A[48]*b[8];
b[4] -= + A[47]*b[7];
b[4] -= + A[46]*b[6];
b[4] -= + A[45]*b[5];
b[4] = b[4]/A[44];
b[3] -= + A[39]*b[9];
b[3] -= + A[38]*b[8];
b[3] -= + A[37]*b[7];
b[3] -= + A[36]*b[6];
b[3] -= + A[35]*b[5];
b[3] -= + A[34]*b[4];
b[3] = b[3]/A[33];
b[2] -= + A[29]*b[9];
b[2] -= + A[28]*b[8];
b[2] -= + A[27]*b[7];
b[2] -= + A[26]*b[6];
b[2] -= + A[25]*b[5];
b[2] -= + A[24]*b[4];
b[2] -= + A[23]*b[3];
b[2] = b[2]/A[22];
b[1] -= + A[19]*b[9];
b[1] -= + A[18]*b[8];
b[1] -= + A[17]*b[7];
b[1] -= + A[16]*b[6];
b[1] -= + A[15]*b[5];
b[1] -= + A[14]*b[4];
b[1] -= + A[13]*b[3];
b[1] -= + A[12]*b[2];
b[1] = b[1]/A[11];
b[0] -= + A[9]*b[9];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim10_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 10; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (9); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*10+i]);
	for( j=(i+1); j < 10; j++ ) {
		temp = fabs(A[j*10+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 10; ++k)
{
	acadoWorkspace.rk_dim10_swap = A[i*10+k];
	A[i*10+k] = A[indexMax*10+k];
	A[indexMax*10+k] = acadoWorkspace.rk_dim10_swap;
}
	acadoWorkspace.rk_dim10_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = acadoWorkspace.rk_dim10_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*10+i];
	for( j=i+1; j < 10; j++ ) {
		A[j*10+i] = -A[j*10+i]/A[i*10+i];
		for( k=i+1; k < 10; k++ ) {
			A[j*10+k] += A[j*10+i] * A[i*10+k];
		}
		b[j] += A[j*10+i] * b[i];
	}
}
det *= A[99];
det = fabs(det);
acado_solve_dim10_triangular( A, b );
return det;
}

void acado_solve_dim10_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim10_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim10_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim10_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim10_bPerm[3] = b[rk_perm[3]];
acadoWorkspace.rk_dim10_bPerm[4] = b[rk_perm[4]];
acadoWorkspace.rk_dim10_bPerm[5] = b[rk_perm[5]];
acadoWorkspace.rk_dim10_bPerm[6] = b[rk_perm[6]];
acadoWorkspace.rk_dim10_bPerm[7] = b[rk_perm[7]];
acadoWorkspace.rk_dim10_bPerm[8] = b[rk_perm[8]];
acadoWorkspace.rk_dim10_bPerm[9] = b[rk_perm[9]];
acadoWorkspace.rk_dim10_bPerm[1] += A[10]*acadoWorkspace.rk_dim10_bPerm[0];

acadoWorkspace.rk_dim10_bPerm[2] += A[20]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[2] += A[21]*acadoWorkspace.rk_dim10_bPerm[1];

acadoWorkspace.rk_dim10_bPerm[3] += A[30]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[3] += A[31]*acadoWorkspace.rk_dim10_bPerm[1];
acadoWorkspace.rk_dim10_bPerm[3] += A[32]*acadoWorkspace.rk_dim10_bPerm[2];

acadoWorkspace.rk_dim10_bPerm[4] += A[40]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[4] += A[41]*acadoWorkspace.rk_dim10_bPerm[1];
acadoWorkspace.rk_dim10_bPerm[4] += A[42]*acadoWorkspace.rk_dim10_bPerm[2];
acadoWorkspace.rk_dim10_bPerm[4] += A[43]*acadoWorkspace.rk_dim10_bPerm[3];

acadoWorkspace.rk_dim10_bPerm[5] += A[50]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[5] += A[51]*acadoWorkspace.rk_dim10_bPerm[1];
acadoWorkspace.rk_dim10_bPerm[5] += A[52]*acadoWorkspace.rk_dim10_bPerm[2];
acadoWorkspace.rk_dim10_bPerm[5] += A[53]*acadoWorkspace.rk_dim10_bPerm[3];
acadoWorkspace.rk_dim10_bPerm[5] += A[54]*acadoWorkspace.rk_dim10_bPerm[4];

acadoWorkspace.rk_dim10_bPerm[6] += A[60]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[6] += A[61]*acadoWorkspace.rk_dim10_bPerm[1];
acadoWorkspace.rk_dim10_bPerm[6] += A[62]*acadoWorkspace.rk_dim10_bPerm[2];
acadoWorkspace.rk_dim10_bPerm[6] += A[63]*acadoWorkspace.rk_dim10_bPerm[3];
acadoWorkspace.rk_dim10_bPerm[6] += A[64]*acadoWorkspace.rk_dim10_bPerm[4];
acadoWorkspace.rk_dim10_bPerm[6] += A[65]*acadoWorkspace.rk_dim10_bPerm[5];

acadoWorkspace.rk_dim10_bPerm[7] += A[70]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[7] += A[71]*acadoWorkspace.rk_dim10_bPerm[1];
acadoWorkspace.rk_dim10_bPerm[7] += A[72]*acadoWorkspace.rk_dim10_bPerm[2];
acadoWorkspace.rk_dim10_bPerm[7] += A[73]*acadoWorkspace.rk_dim10_bPerm[3];
acadoWorkspace.rk_dim10_bPerm[7] += A[74]*acadoWorkspace.rk_dim10_bPerm[4];
acadoWorkspace.rk_dim10_bPerm[7] += A[75]*acadoWorkspace.rk_dim10_bPerm[5];
acadoWorkspace.rk_dim10_bPerm[7] += A[76]*acadoWorkspace.rk_dim10_bPerm[6];

acadoWorkspace.rk_dim10_bPerm[8] += A[80]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[8] += A[81]*acadoWorkspace.rk_dim10_bPerm[1];
acadoWorkspace.rk_dim10_bPerm[8] += A[82]*acadoWorkspace.rk_dim10_bPerm[2];
acadoWorkspace.rk_dim10_bPerm[8] += A[83]*acadoWorkspace.rk_dim10_bPerm[3];
acadoWorkspace.rk_dim10_bPerm[8] += A[84]*acadoWorkspace.rk_dim10_bPerm[4];
acadoWorkspace.rk_dim10_bPerm[8] += A[85]*acadoWorkspace.rk_dim10_bPerm[5];
acadoWorkspace.rk_dim10_bPerm[8] += A[86]*acadoWorkspace.rk_dim10_bPerm[6];
acadoWorkspace.rk_dim10_bPerm[8] += A[87]*acadoWorkspace.rk_dim10_bPerm[7];

acadoWorkspace.rk_dim10_bPerm[9] += A[90]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[9] += A[91]*acadoWorkspace.rk_dim10_bPerm[1];
acadoWorkspace.rk_dim10_bPerm[9] += A[92]*acadoWorkspace.rk_dim10_bPerm[2];
acadoWorkspace.rk_dim10_bPerm[9] += A[93]*acadoWorkspace.rk_dim10_bPerm[3];
acadoWorkspace.rk_dim10_bPerm[9] += A[94]*acadoWorkspace.rk_dim10_bPerm[4];
acadoWorkspace.rk_dim10_bPerm[9] += A[95]*acadoWorkspace.rk_dim10_bPerm[5];
acadoWorkspace.rk_dim10_bPerm[9] += A[96]*acadoWorkspace.rk_dim10_bPerm[6];
acadoWorkspace.rk_dim10_bPerm[9] += A[97]*acadoWorkspace.rk_dim10_bPerm[7];
acadoWorkspace.rk_dim10_bPerm[9] += A[98]*acadoWorkspace.rk_dim10_bPerm[8];


acado_solve_dim10_triangular( A, acadoWorkspace.rk_dim10_bPerm );
b[0] = acadoWorkspace.rk_dim10_bPerm[0];
b[1] = acadoWorkspace.rk_dim10_bPerm[1];
b[2] = acadoWorkspace.rk_dim10_bPerm[2];
b[3] = acadoWorkspace.rk_dim10_bPerm[3];
b[4] = acadoWorkspace.rk_dim10_bPerm[4];
b[5] = acadoWorkspace.rk_dim10_bPerm[5];
b[6] = acadoWorkspace.rk_dim10_bPerm[6];
b[7] = acadoWorkspace.rk_dim10_bPerm[7];
b[8] = acadoWorkspace.rk_dim10_bPerm[8];
b[9] = acadoWorkspace.rk_dim10_bPerm[9];
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t acado_Ah_mat[ 4 ] = 
{ 2.5000000000000001e-02, 5.3867513459481292e-02, 
-3.8675134594812867e-03, 2.5000000000000001e-02 };


/* Fixed step size:0.1 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[5] = rk_eta[35];
acadoWorkspace.rk_xxx[6] = rk_eta[36];
acadoWorkspace.rk_xxx[7] = rk_eta[37];
acadoWorkspace.rk_xxx[8] = rk_eta[38];
acadoWorkspace.rk_xxx[9] = rk_eta[39];
acadoWorkspace.rk_xxx[10] = rk_eta[40];
acadoWorkspace.rk_xxx[11] = rk_eta[41];
acadoWorkspace.rk_xxx[12] = rk_eta[42];

for (run = 0; run < 1; ++run)
{
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 5; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 30 ]) );
for (j = 0; j < 5; ++j)
{
tmp_index1 = (run1 * 5) + (j);
acadoWorkspace.rk_A[tmp_index1 * 10] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 1] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 2] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 3] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 4] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 4)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 10) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 10 + 5] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 6] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 7] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 8] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 9] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 4)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 10) + (j + 5)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 5] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 5 + 1] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 5 + 2] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 5 + 3] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 5 + 4] = acadoWorkspace.rk_kkk[run1 + 8] - acadoWorkspace.rk_rhsTemp[4];
}
det = acado_solve_dim10_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim10_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 5];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 5 + 1];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 5 + 2];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 5 + 3];
acadoWorkspace.rk_kkk[j + 8] += acadoWorkspace.rk_b[j * 5 + 4];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 5; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 5] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 5 + 1] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 5 + 2] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 5 + 3] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 5 + 4] = acadoWorkspace.rk_kkk[run1 + 8] - acadoWorkspace.rk_rhsTemp[4];
}
acado_solve_dim10_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim10_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 5];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 5 + 1];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 5 + 2];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 5 + 3];
acadoWorkspace.rk_kkk[j + 8] += acadoWorkspace.rk_b[j * 5 + 4];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 5; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 30 ]) );
for (j = 0; j < 5; ++j)
{
tmp_index1 = (run1 * 5) + (j);
acadoWorkspace.rk_A[tmp_index1 * 10] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 1] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 2] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 3] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 4] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 4)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 10) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 10 + 5] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 6] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 7] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 8] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 9] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 30) + (j * 6 + 4)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 10) + (j + 5)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 5; ++run1)
{
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_b[i * 5] = - acadoWorkspace.rk_diffsTemp2[(i * 30) + (run1)];
acadoWorkspace.rk_b[i * 5 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 30) + (run1 + 6)];
acadoWorkspace.rk_b[i * 5 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 30) + (run1 + 12)];
acadoWorkspace.rk_b[i * 5 + 3] = - acadoWorkspace.rk_diffsTemp2[(i * 30) + (run1 + 18)];
acadoWorkspace.rk_b[i * 5 + 4] = - acadoWorkspace.rk_diffsTemp2[(i * 30) + (run1 + 24)];
}
if( 0 == run1 ) {
det = acado_solve_dim10_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim10_perm );
}
 else {
acado_solve_dim10_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim10_perm );
}
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 5];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 5 + 1];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 5 + 2];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 5 + 3];
acadoWorkspace.rk_diffK[i + 8] = acadoWorkspace.rk_b[i * 5 + 4];
}
for (i = 0; i < 5; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 6) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 6) + (run1)] += + acadoWorkspace.rk_diffK[i * 2]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_diffK[i * 2 + 1]*(real_t)5.0000000000000003e-02;
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 5; ++j)
{
tmp_index1 = (i * 5) + (j);
tmp_index2 = (run1) + (j * 6);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 30) + (tmp_index2 + 5)];
}
}
acado_solve_dim10_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim10_perm );
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 5];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 5 + 1];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 5 + 2];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 5 + 3];
acadoWorkspace.rk_diffK[i + 8] = acadoWorkspace.rk_b[i * 5 + 4];
}
for (i = 0; i < 5; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 6) + (run1 + 5)] = + acadoWorkspace.rk_diffK[i * 2]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_diffK[i * 2 + 1]*(real_t)5.0000000000000003e-02;
}
}
rk_eta[0] += + acadoWorkspace.rk_kkk[0]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_kkk[1]*(real_t)5.0000000000000003e-02;
rk_eta[1] += + acadoWorkspace.rk_kkk[2]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_kkk[3]*(real_t)5.0000000000000003e-02;
rk_eta[2] += + acadoWorkspace.rk_kkk[4]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_kkk[5]*(real_t)5.0000000000000003e-02;
rk_eta[3] += + acadoWorkspace.rk_kkk[6]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_kkk[7]*(real_t)5.0000000000000003e-02;
rk_eta[4] += + acadoWorkspace.rk_kkk[8]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_kkk[9]*(real_t)5.0000000000000003e-02;
for (i = 0; i < 5; ++i)
{
for (j = 0; j < 5; ++j)
{
tmp_index2 = (j) + (i * 5);
rk_eta[tmp_index2 + 5] = acadoWorkspace.rk_diffsNew2[(i * 6) + (j)];
}
for (j = 0; j < 1; ++j)
{
tmp_index2 = (j) + (i);
rk_eta[tmp_index2 + 30] = acadoWorkspace.rk_diffsNew2[(i * 6) + (j + 5)];
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 1.0000000000000000e+00;
}
for (i = 0; i < 5; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



