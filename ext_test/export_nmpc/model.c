#include "acado_common.h"

void rhs( const real_t *in, real_t *out ){

/* rhs */
out[0] = in[1]; // u

}

void rhs_jac( const real_t *in, real_t *out ){
        
/* jacobian */
out[0] = 0.0; // d/dx u = 0
out[1] = 1.0; // d/du u = 1

}


// void evaluateLSQ(const real_t* in, real_t* out)
// {
// const real_t* xd = in;
// const real_t* u = in + 1;
// 
// /* Compute intermediate quantities: */
// const double a0 = (pow(xd[0],3));
// const double a1 = ((real_t)(3.0000000000000000e+00)*((xd[0])*(xd[0])));
// 
// /* Compute outputs: */
// out[0] = (((real_t)(0.0000000000000000e+00)-a0)+xd[0]);
// out[1] = (u[0]+xd[0]);
// out[2] = (((real_t)(0.0000000000000000e+00)-a1)+(real_t)(1.0000000000000000e+00));
// out[3] = (real_t)(1.0000000000000000e+00);
// }
// 
// void evaluateLSQEndTerm(const real_t* in, real_t* out)
// {
// const real_t* xd = in;
// 
// /* Compute intermediate quantities: */
// const double a0 = (pow(xd[0],3));
// const double a1 = ((real_t)(3.0000000000000000e+00)*((xd[0])*(xd[0])));
// 
// /* Compute outputs: */
// out[0] = (((real_t)(0.0000000000000000e+00)-a0)+xd[0]);
// out[1] = (((real_t)(0.0000000000000000e+00)-a1)+(real_t)(1.0000000000000000e+00));
// }

void evaluateLSQ(const real_t* in, real_t* out)
{

/* Compute outputs: */
out[0] = in[0] - in[0]*in[0]*in[0];
out[1] = in[1] + in[0];
out[2] = 1.0 - 3.0*in[0]*in[0];
out[3] = 1.0;
out[4] = 0.0;
out[5] = 1.0;

}

void evaluateLSQEndTerm(const real_t* in, real_t* out)
{
    
/* Compute outputs: */
out[0] = in[0] - in[0]*in[0]*in[0];
out[1] = 1.0 - 3.0*in[0]*in[0];
out[2] = 0.0;

}