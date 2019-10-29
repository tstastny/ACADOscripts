/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState r_n;
    DifferentialState r_e;
    DifferentialState r_d;
    DifferentialState v;
    DifferentialState gamma;
    DifferentialState xi;
    DifferentialState phi;
    DifferentialState theta;
    DifferentialState n_p;
    Control u_T;
    Control phi_ref;
    Control theta_ref;
    OnlineData rho; 
    OnlineData w_n; 
    OnlineData w_e; 
    OnlineData w_d; 
    OnlineData tau_phi; 
    OnlineData tau_theta; 
    OnlineData k_phi; 
    OnlineData k_theta; 
    OnlineData sig_aoa; 
    OnlineData jac_sig_aoa1; 
    OnlineData jac_sig_aoa2; 
    OnlineData sig_h; 
    OnlineData jac_sig_h1; 
    OnlineData jac_sig_h2; 
    OnlineData jac_sig_h3; 
    OnlineData jac_sig_h4; 
    OnlineData sig_r; 
    OnlineData jac_sig_r1; 
    OnlineData jac_sig_r2; 
    OnlineData jac_sig_r3; 
    OnlineData jac_sig_r4; 
    OnlineData jac_sig_r5; 
    OnlineData jac_sig_r6; 
    BMatrix acadodata_M1;
    acadodata_M1.read( "nmpc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "nmpc_data_acadodata_M2.txt" );
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(r_n) == (cos(gamma)*cos(xi)*v+w_n);
    acadodata_f1 << dot(r_e) == (cos(gamma)*sin(xi)*v+w_e);
    acadodata_f1 << dot(r_d) == ((-v)*sin(gamma)+w_d);
    acadodata_f1 << dot(v) == ((((-1.32619999999999987894e-01)/2.80000000000000026645e-01*cos((-3.49065850398865909487e-02-gamma+theta))/n_p*v+1.15210000000000006848e-01)*6.14656000000000229955e-03*cos((-gamma+theta))*pow(n_p,2.00000000000000000000e+00)*rho-((-gamma+theta)*3.47380000000000022098e-01+2.59919999999999984386e-01*pow((-gamma+theta),2.00000000000000000000e+00)+6.03139999999999995572e-02)*4.17999999999999982681e-01*5.00000000000000000000e-01*pow(v,2.00000000000000000000e+00)*rho)*3.20102432778489098819e-01-9.81000000000000049738e+00*sin(gamma));
    acadodata_f1 << dot(gamma) == ((((-1.32619999999999987894e-01)/2.80000000000000026645e-01*cos((-3.49065850398865909487e-02-gamma+theta))/n_p*v+1.15210000000000006848e-01)*6.14656000000000229955e-03*pow(n_p,2.00000000000000000000e+00)*rho*sin((-gamma+theta))+((-gamma+theta)*2.85009999999999985576e+00+5.21549999999999958078e-01)*4.17999999999999982681e-01*5.00000000000000000000e-01*pow(v,2.00000000000000000000e+00)*rho)*cos(phi)-3.06464400000000019020e+01*cos(gamma))*3.20102432778489098819e-01/v;
    acadodata_f1 << dot(xi) == (((-1.32619999999999987894e-01)/2.80000000000000026645e-01*cos((-3.49065850398865909487e-02-gamma+theta))/n_p*v+1.15210000000000006848e-01)*6.14656000000000229955e-03*pow(n_p,2.00000000000000000000e+00)*rho*sin((-gamma+theta))+((-gamma+theta)*2.85009999999999985576e+00+5.21549999999999958078e-01)*4.17999999999999982681e-01*5.00000000000000000000e-01*pow(v,2.00000000000000000000e+00)*rho)/3.12400000000000011013e+00/cos(gamma)*sin(phi)/v;
    acadodata_f1 << dot(phi) == (k_phi*phi_ref-phi)/tau_phi;
    acadodata_f1 << dot(theta) == (k_theta*theta_ref-theta)/tau_theta;
    acadodata_f1 << dot(n_p) == ((-(-1.00000000000000000000e+01+cos((-3.49065850398865909487e-02-gamma+theta))*v)/1.50000000000000000000e+01+1.00000000000000000000e+00)*(1.22767629097648992342e+02*u_T+4.08990375690176648504e+01)+(-1.00000000000000000000e+01+cos((-3.49065850398865909487e-02-gamma+theta))*v)*(1.02693635333290359313e+02+6.09730313333762978800e+01*u_T)/1.50000000000000000000e+01-n_p)/2.00000000000000011102e-01;

    OCP ocp1(0, 5, 50);
    ocp1.minimizeLSQ(acadodata_M1, "evaluateLSQ");
    ocp1.minimizeLSQEndTerm(acadodata_M2, "evaluateLSQEndTerm");
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo(0.00000000000000000000e+00 <= u_T <= 1.00000000000000000000e+00);
    ocp1.subjectTo((-6.10865238198015303439e-01) <= phi_ref <= 6.10865238198015303439e-01);
    ocp1.subjectTo((-2.61799387799149407829e-01) <= theta_ref <= 4.36332312998582383390e-01);
    ocp1.setNOD( 23 );


    ocp1.setNU( 3 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 23 );
    OCPexport ExportModule1( ocp1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule1.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule1.set( SPARSE_QP_SOLUTION, FULL_CONDENSING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_GL4 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 50 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( MAX_NUM_QP_ITERATIONS, 500 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: MAX_NUM_QP_ITERATIONS");
    options_flag = ExportModule1.set( HOTSTART_QP, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HOTSTART_QP");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-10 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    options_flag = ExportModule1.set( CG_HARDCODE_CONSTRAINT_VALUES, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: CG_HARDCODE_CONSTRAINT_VALUES");
    options_flag = ExportModule1.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: CG_USE_VARIABLE_WEIGHTING_MATRIX");
    options_flag = ExportModule1.set( GENERATE_MAKE_FILE, NO );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: GENERATE_MAKE_FILE");
    options_flag = ExportModule1.set( GENERATE_TEST_FILE, NO );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: GENERATE_TEST_FILE");
    options_flag = ExportModule1.set( GENERATE_SIMULINK_INTERFACE, NO );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: GENERATE_SIMULINK_INTERFACE");
    uint export_flag;
    export_flag = ExportModule1.exportCode( "export_nmpc" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

