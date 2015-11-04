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