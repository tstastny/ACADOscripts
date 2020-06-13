#include "acado_common.h"
#include <math.h>
#include <string.h>

void lsq_obj_eval( const real_t *in, real_t *out, bool eval_end_term )
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */

    /* shift inputs by ACADO_NU if evaluating the end term */
    /* states */
    const double v_x = in[0];
    const double v_z = in[1];
    const double theta = in[2];
    const double zeta_w = in[3];
    const double T = in[4];

    /* controls */ /* NOTE: these just dont get used if in end term eval */
    const double delta_w = in[5];
    const double T_ref = in[6];
    const double theta_ref = in[7];
    
    /* parameters */
    const double rho = 1.225; /* density of air [kg/m^3] */
    const double S_w = 0.15; /* wing area */
    const double k = 1;
    const double tau_0 = 0.4;
    const double m = 1.85;
    const double g = 9.81;
    const double k_T2L = 0.3;
    const double c_w = M_PI/10;
    const double tau_theta = 0.4;
    const double tau_T = 0.05;
    


    /* online data */


    /* OBJECTIVES - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

    /* state output */
    out[0] = v_x; /* airspeed_x, v_x */
    out[1] = v_z; /* airspeed_z, v_z */
    out[2] = theta; /* theta */
    out[3] = zeta_w; /* zeta_w */
    out[4] = T; /* Thrust, T */

    /* control output */
    if (!eval_end_term) {
        out[5] = delta_w; /* input wing tiltrate, delta_w  */
        out[6] = T_ref; /* input Thrust, T_ref */
        out[7] = theta_ref; /* input theta, theta_ref */
    }
    /* JACOBIANS - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

    if (eval_end_term) {

        /* lsq end term non-zero jacobian evals */
        /* lsq end term jacobian w.r.t. states */

        /* Jacobians of v_x, dv_x/d(states) */
        out[5] = ((S_w*rho*(v_x*v_x+v_z*v_z)*(-1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(v_x*v_x)*v_z*(1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w+atan(v_z/v_x))*2.0)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(1.0/(v_x*v_x)*v_z*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)*4.5E+1)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(1.0/(v_x*v_x)*v_z*cos(zeta_w+atan(v_z/v_x))*sin(zeta_w+atan(v_z/v_x))*(1.19E+2/5.0E+1))/((1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+(1.0/(v_x*v_x)*v_z*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)*4.5E+1)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0))+1.0/(v_x*v_x*v_x)*(v_z*v_z)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)+(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(v_x*v_x)*v_z*(1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(6.3E+1/1.0E+1))/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(1.0/(v_x*v_x)*v_z*cos(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/5.0))/((1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))-(1.0/(v_x*v_x)*v_z*sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(6.3E+1/2.0))/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)+(1.0/(v_x*v_x)*v_z*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)*4.5E+1)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)))/v_x+1.0/(v_x*v_x)*v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))-1.0/(v_x*v_x*v_x*v_x)*(v_z*v_z*v_z)*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))))/2.0+S_w*rho*v_x*(((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)))/v_x))/m;;
        out[6] = (theta-k*theta_ref)/tau_0+(S_w*rho*v_z*(((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)))/v_x)-(S_w*rho*(v_x*v_x+v_z*v_z)*(-1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w+atan(v_z/v_x))*2.0)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)*4.5E+1)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(cos(zeta_w+atan(v_z/v_x))*sin(zeta_w+atan(v_z/v_x))*(1.19E+2/5.0E+1))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)*4.5E+1)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0)))+(1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)))/v_x+(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(6.3E+1/1.0E+1))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(cos(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/5.0))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)*4.5E+1)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(6.3E+1/2.0))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))))/v_x-1.0/(v_x*v_x*v_x)*(v_z*v_z)*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+1.0/(v_x*v_x)*v_z*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)))/2.0)/m;
        out[7] = -g*cos(theta)+v_z/tau_0;
        out[8] = -(T*sin(zeta_w)+T*k_T2L*cos(zeta_w)-(S_w*rho*(v_x*v_x+v_z*v_z)*(1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*2.0+atan(v_z/v_x)*2.0)-exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)*4.5E+1-(cos(zeta_w+atan(v_z/v_x))*sin(zeta_w+atan(v_z/v_x))*(1.19E+2/5.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)+exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)*4.5E+1)+(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((cos(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/5.0))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-(6.3E+1/1.0E+1)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)+sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(6.3E+1/2.0)-exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)*4.5E+1+6.3E+1/1.0E+1))/v_x))/2.0)/m;
        out[9] = (cos(zeta_w)-k_T2L*sin(zeta_w))/m;
        /* Jacobians of v_z, dv_z/d(states) */
        out[10] = -(theta-k*theta_ref)/tau_0+((S_w*rho*(v_x*v_x+v_z*v_z)*(-1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(v_x*v_x)*v_z*(1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(6.3E+1/1.0E+1))/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(1.0/(v_x*v_x)*v_z*cos(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/5.0))/((1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))-(1.0/(v_x*v_x)*v_z*sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(6.3E+1/2.0))/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)+(1.0/(v_x*v_x)*v_z*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)*4.5E+1)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-1.0/(v_x*v_x*v_x*v_x)*(v_z*v_z*v_z)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)+1.0/(v_x*v_x*v_x)*(v_z*v_z)*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+1.0/(v_x*v_x)*v_z*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)+(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(v_x*v_x)*v_z*(1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w+atan(v_z/v_x))*2.0)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(1.0/(v_x*v_x)*v_z*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)*4.5E+1)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(1.0/(v_x*v_x)*v_z*cos(zeta_w+atan(v_z/v_x))*sin(zeta_w+atan(v_z/v_x))*(1.19E+2/5.0E+1))/((1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+(1.0/(v_x*v_x)*v_z*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)*4.5E+1)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)))/v_x))/2.0+S_w*rho*v_x*(1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))-(v_z*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0))/v_x))/m;
        out[11] = -((S_w*rho*(v_x*v_x+v_z*v_z)*(-1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(6.3E+1/1.0E+1))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(cos(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/5.0))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)*4.5E+1)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(6.3E+1/2.0))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0)))+(((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0))/v_x-1.0/(v_x*v_x*v_x)*(v_z*v_z)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)+(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w+atan(v_z/v_x))*2.0)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)*4.5E+1)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(cos(zeta_w+atan(v_z/v_x))*sin(zeta_w+atan(v_z/v_x))*(1.19E+2/5.0E+1))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)*4.5E+1)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))))/v_x+1.0/(v_x*v_x)*v_z*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))))/2.0-S_w*rho*v_z*(1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))-(v_z*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0))/v_x))/m;;
        out[12] = -v_x/tau_0-g*sin(theta);
        out[13] = -(T*cos(zeta_w)-T*k_T2L*sin(zeta_w)+(S_w*rho*(v_x*v_x+v_z*v_z)*(1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((cos(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/5.0))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-(6.3E+1/1.0E+1)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)+sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(6.3E+1/2.0)-exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)*4.5E+1+6.3E+1/1.0E+1)+(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*2.0+atan(v_z/v_x)*2.0)-exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)*4.5E+1-(cos(zeta_w+atan(v_z/v_x))*sin(zeta_w+atan(v_z/v_x))*(1.19E+2/5.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)+exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)*4.5E+1))/v_x))/2.0)/m;;
        out[14] = -(sin(zeta_w)+k_T2L*cos(zeta_w))/m;
        /* Jacobians of theta, dtheta/d(states) */
        out[15] = 0.0;
        out[16] = 0.0;
        out[17] = -1.0/tau_0;
        out[18] = 0.0;
        out[19] = 0.0;
        /* Jacobians of zeta_w, dzeta_w/d(states) */
        out[20] = 0.0;
        out[21] = 0.0;
        out[22] = 0.0;
        out[23] = 0.0;
        out[24] = 0.0;
        /* Jacobians of T, dT/d(states) */
        out[25] = 0.0;
        out[26] = 0.0;
        out[27] = 0.0;
        out[28] = 0.0;
        out[29] = -1.0/tau_T;

    }
    else {

        /* lsq non-zero jacobian evals */
        /* lsq jacobian w.r.t. states */
        /* Jacobians of v_x, dv_x/d(states) */
        out[8] = ((S_w*rho*(v_x*v_x+v_z*v_z)*(-1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(v_x*v_x)*v_z*(1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w+atan(v_z/v_x))*2.0)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(1.0/(v_x*v_x)*v_z*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)*4.5E+1)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(1.0/(v_x*v_x)*v_z*cos(zeta_w+atan(v_z/v_x))*sin(zeta_w+atan(v_z/v_x))*(1.19E+2/5.0E+1))/((1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+(1.0/(v_x*v_x)*v_z*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)*4.5E+1)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0))+1.0/(v_x*v_x*v_x)*(v_z*v_z)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)+(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(v_x*v_x)*v_z*(1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(6.3E+1/1.0E+1))/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(1.0/(v_x*v_x)*v_z*cos(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/5.0))/((1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))-(1.0/(v_x*v_x)*v_z*sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(6.3E+1/2.0))/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)+(1.0/(v_x*v_x)*v_z*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)*4.5E+1)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)))/v_x+1.0/(v_x*v_x)*v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))-1.0/(v_x*v_x*v_x*v_x)*(v_z*v_z*v_z)*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))))/2.0+S_w*rho*v_x*(((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)))/v_x))/m;;
        out[9] = (theta-k*theta_ref)/tau_0+(S_w*rho*v_z*(((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)))/v_x)-(S_w*rho*(v_x*v_x+v_z*v_z)*(-1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w+atan(v_z/v_x))*2.0)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)*4.5E+1)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(cos(zeta_w+atan(v_z/v_x))*sin(zeta_w+atan(v_z/v_x))*(1.19E+2/5.0E+1))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)*4.5E+1)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0)))+(1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)))/v_x+(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(6.3E+1/1.0E+1))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(cos(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/5.0))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)*4.5E+1)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(6.3E+1/2.0))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))))/v_x-1.0/(v_x*v_x*v_x)*(v_z*v_z)*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+1.0/(v_x*v_x)*v_z*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)))/2.0)/m;
        out[10] = -g*cos(theta)+v_z/tau_0;
        out[11] = -(T*sin(zeta_w)+T*k_T2L*cos(zeta_w)-(S_w*rho*(v_x*v_x+v_z*v_z)*(1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*2.0+atan(v_z/v_x)*2.0)-exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)*4.5E+1-(cos(zeta_w+atan(v_z/v_x))*sin(zeta_w+atan(v_z/v_x))*(1.19E+2/5.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)+exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)*4.5E+1)+(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((cos(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/5.0))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-(6.3E+1/1.0E+1)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)+sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(6.3E+1/2.0)-exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)*4.5E+1+6.3E+1/1.0E+1))/v_x))/2.0)/m;
        out[12] = (cos(zeta_w)-k_T2L*sin(zeta_w))/m;
        /* Jacobians of v_z, dv_z/d(states) */
        out[13] = -(theta-k*theta_ref)/tau_0+((S_w*rho*(v_x*v_x+v_z*v_z)*(-1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(v_x*v_x)*v_z*(1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(6.3E+1/1.0E+1))/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(1.0/(v_x*v_x)*v_z*cos(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/5.0))/((1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))-(1.0/(v_x*v_x)*v_z*sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(6.3E+1/2.0))/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)+(1.0/(v_x*v_x)*v_z*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)*4.5E+1)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-1.0/(v_x*v_x*v_x*v_x)*(v_z*v_z*v_z)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)+1.0/(v_x*v_x*v_x)*(v_z*v_z)*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+1.0/(v_x*v_x)*v_z*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)+(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(v_x*v_x)*v_z*(1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w+atan(v_z/v_x))*2.0)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(1.0/(v_x*v_x)*v_z*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)*4.5E+1)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)-(1.0/(v_x*v_x)*v_z*cos(zeta_w+atan(v_z/v_x))*sin(zeta_w+atan(v_z/v_x))*(1.19E+2/5.0E+1))/((1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+(1.0/(v_x*v_x)*v_z*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)*4.5E+1)/(1.0/(v_x*v_x)*(v_z*v_z)+1.0)))/v_x))/2.0+S_w*rho*v_x*(1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))-(v_z*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0))/v_x))/m;
        out[14] = -((S_w*rho*(v_x*v_x+v_z*v_z)*(-1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(6.3E+1/1.0E+1))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(cos(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/5.0))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)*4.5E+1)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(6.3E+1/2.0))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0)))+(((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0))/v_x-1.0/(v_x*v_x*v_x)*(v_z*v_z)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)+(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w+atan(v_z/v_x))*2.0)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)*4.5E+1)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))-(cos(zeta_w+atan(v_z/v_x))*sin(zeta_w+atan(v_z/v_x))*(1.19E+2/5.0E+1))/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))+(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)*4.5E+1)/(v_x*(1.0/(v_x*v_x)*(v_z*v_z)+1.0))))/v_x+1.0/(v_x*v_x)*v_z*1.0/pow(1.0/(v_x*v_x)*(v_z*v_z)+1.0,3.0/2.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))))/2.0-S_w*rho*v_z*(1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)-(sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/1.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))-(v_z*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)-(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0))*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0))/v_x))/m;;
        out[15] = -v_x/tau_0-g*sin(theta);
        out[16] = -(T*cos(zeta_w)-T*k_T2L*sin(zeta_w)+(S_w*rho*(v_x*v_x+v_z*v_z)*(1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((cos(zeta_w*2.0+atan(v_z/v_x)*2.0)*(7.0/5.0))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-(6.3E+1/1.0E+1)/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)+sin(zeta_w*2.0+atan(v_z/v_x)*2.0)*exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(6.3E+1/2.0)-exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(zeta_w*(6.3E+1/1.0E+1)+atan(v_z/v_x)*(6.3E+1/1.0E+1)+1.0/5.0)*4.5E+1+6.3E+1/1.0E+1)+(v_z*1.0/sqrt(1.0/(v_x*v_x)*(v_z*v_z)+1.0)*((1.0/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)-1.0)*(zeta_w*2.0+atan(v_z/v_x)*2.0)-exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(sin(zeta_w+atan(v_z/v_x)),2.0)*(1.19E+2/1.0E+2)+1.0/1.0E+2)*4.5E+1-(cos(zeta_w+atan(v_z/v_x))*sin(zeta_w+atan(v_z/v_x))*(1.19E+2/5.0E+1))/(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0)+exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)*1.0/pow(exp(zeta_w*-4.5E+1-atan(v_z/v_x)*4.5E+1+2.07E+2/2.0E+1)+1.0,2.0)*(pow(zeta_w+atan(v_z/v_x),2.0)+1.0/1.0E+2)*4.5E+1))/v_x))/2.0)/m;;
        out[17] = -(sin(zeta_w)+k_T2L*cos(zeta_w))/m;
        /* Jacobians of theta, dtheta/d(states) */
        out[18] = 0.0;
        out[19] = 0.0;
        out[20] = -1.0/tau_theta;
        out[21] = 0.0;
        out[22] = 0.0;
        /* Jacobians of zeta_w, dzeta_w/d(states) */
        out[23] = 0.0;
        out[24] = 0.0;
        out[25] = 0.0;
        out[26] = 0.0;
        out[27] = 0.0;
        /* Jacobians of T, dT/d(states) */
        out[28] = 0.0;
        out[29] = 0.0;
        out[30] = 0.0;
        out[31] = 0.0;
        out[32] = -1.0/tau_T;
        /* Jacobians of delta_w, ddelta_w/d(states) */
        out[33] = 0.0;
        out[34] = 0.0;
        out[35] = 0.0;
        out[36] = 0.0;
        out[37] = 0.0;
        /* Jacobians of T_ref, dT_ref/d(states) */
        out[38] = 0.0;
        out[39] = 0.0;
        out[40] = 0.0;
        out[41] = 0.0;
        out[42] = 0.0;
        /* Jacobians of theta_ref, dtheta_ref/d(states) */
        out[43] = 0.0;
        out[44] = 0.0;
        out[45] = 0.0;
        out[46] = 0.0;
        out[47] = 0.0;

        /* lsq jacobian w.r.t. controls */
        /* Jacobians of v_x, dv_x/d(controls) */
        out[48] = 0.0;
        out[49] = 0.0;
        out[50] = -(k*v_z)/tau_0;
        /* Jacobians of v_z, dv_z/d(controls) */
        out[51] = 0.0;
        out[52] = 0.0;
        out[53] = (k*v_x)/tau_0;
        /* Jacobians of theta, dtheta/d(controls) */
        out[54] = 0.0;
        out[55] = 0.0;
        out[56] = k/tau_0;
        /* Jacobians of zeta_w, dzeta_w/d(controls) */
        out[57] = c_w;
        out[58] = 0.0;
        out[59] = 0.0;
        /* Jacobians of T, dT/d(controls) */
        out[60] = 0.0;
        out[61] = 1.0/tau_T;
        out[62] = 0.0;
        /* Jacobians of delta_w, ddelta_w/d(controls) */
        out[63] = 1.0;
        out[64] = 0.0;
        out[65] = 0.0;
        /* Jacobians of T_ref, dT_ref/d(controls) */
        out[66] = 0.0;
        out[67] = 1.0;
        out[68] = 0.0;
        /* Jacobians of theta_ref, dtheta_ref/d(controls) */
        out[69] = 0.0;
        out[70] = 0.0;
        out[71] = 1.0;
        
    }
} /* lsq_obj_eval */

void acado_evaluateLSQ( const real_t *in, real_t *out )
{
    lsq_obj_eval(in, out, false);
} /* acado_evaluateLSQ */

void acado_evaluateLSQEndTerm( const real_t *in, real_t *out )
{
    lsq_obj_eval(in, out, true);
} /* acado_evaluateLSQEndTerm */

