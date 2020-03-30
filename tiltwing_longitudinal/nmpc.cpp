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
    DifferentialState v_x;
    DifferentialState v_z;
    DifferentialState theta;
    DifferentialState zeta_w;
    Control delta_w;
    Control T_w;
    Control theta_ref;
    BMatrix acadodata_M1;
    acadodata_M1.read( "nmpc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "nmpc_data_acadodata_M2.txt" );
    Function acadodata_f2;
    acadodata_f2 << v_x;
    acadodata_f2 << v_z;
    acadodata_f2 << theta;
    acadodata_f2 << zeta_w;
    acadodata_f2 << delta_w;
    acadodata_f2 << T_w;
    acadodata_f2 << theta_ref;
    Function acadodata_f3;
    acadodata_f3 << v_x;
    acadodata_f3 << v_z;
    acadodata_f3 << theta;
    acadodata_f3 << zeta_w;
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(v_x) == ((((((atan(1/v_x*v_z)+zeta_w)*6.29999999999999982236e+00+2.00000000000000011102e-01)*(1.00000000000000000000e+00-1/(1.00000000000000000000e+00+exp((-2.30000000000000009992e-01+atan(1/v_x*v_z)+zeta_w)*(-4.50000000000000000000e+01))))+1/(1.00000000000000000000e+00+exp((-2.30000000000000009992e-01+atan(1/v_x*v_z)+zeta_w)*(-4.50000000000000000000e+01)))*6.99999999999999955591e-01*sin((atan(1/v_x*v_z)+zeta_w)*2.00000000000000000000e+00))*sin(atan(1/v_x*v_z))-((1.00000000000000000000e+00-1/(1.00000000000000000000e+00+exp((-2.30000000000000009992e-01+atan(1/v_x*v_z)+zeta_w)*(-4.50000000000000000000e+01))))*(1.00000000000000002082e-02+pow((atan(1/v_x*v_z)+zeta_w),2.00000000000000000000e+00))+1/(1.00000000000000000000e+00+exp((-2.30000000000000009992e-01+atan(1/v_x*v_z)+zeta_w)*(-4.50000000000000000000e+01)))*(1.00000000000000002082e-02+1.18999999999999994671e+00*pow(sin((atan(1/v_x*v_z)+zeta_w)),2.00000000000000000000e+00)))*cos(atan(1/v_x*v_z)))*(pow(v_x,2.00000000000000000000e+00)+pow(v_z,2.00000000000000000000e+00))*1.49999999999999994449e-01*6.12500000000000044409e-01-1.81485000000000020748e+01*sin(theta)-2.99999999999999988898e-01*T_w*sin(zeta_w)+T_w*cos(zeta_w))*5.40540540540540459524e-01+(-theta+theta_ref)/4.00000000000000022204e-01*v_z);
    acadodata_f1 << dot(v_z) == ((-((((atan(1/v_x*v_z)+zeta_w)*6.29999999999999982236e+00+2.00000000000000011102e-01)*(1.00000000000000000000e+00-1/(1.00000000000000000000e+00+exp((-2.30000000000000009992e-01+atan(1/v_x*v_z)+zeta_w)*(-4.50000000000000000000e+01))))+1/(1.00000000000000000000e+00+exp((-2.30000000000000009992e-01+atan(1/v_x*v_z)+zeta_w)*(-4.50000000000000000000e+01)))*6.99999999999999955591e-01*sin((atan(1/v_x*v_z)+zeta_w)*2.00000000000000000000e+00))*cos(atan(1/v_x*v_z))-((1.00000000000000000000e+00-1/(1.00000000000000000000e+00+exp((-2.30000000000000009992e-01+atan(1/v_x*v_z)+zeta_w)*(-4.50000000000000000000e+01))))*(1.00000000000000002082e-02+pow((atan(1/v_x*v_z)+zeta_w),2.00000000000000000000e+00))+1/(1.00000000000000000000e+00+exp((-2.30000000000000009992e-01+atan(1/v_x*v_z)+zeta_w)*(-4.50000000000000000000e+01)))*(1.00000000000000002082e-02+1.18999999999999994671e+00*pow(sin((atan(1/v_x*v_z)+zeta_w)),2.00000000000000000000e+00)))*sin(atan(1/v_x*v_z)))*(pow(v_x,2.00000000000000000000e+00)+pow(v_z,2.00000000000000000000e+00))*1.49999999999999994449e-01*6.12500000000000044409e-01+(-T_w)*sin(zeta_w)+1.81485000000000020748e+01*cos(theta)-2.99999999999999988898e-01*T_w*cos(zeta_w))*5.40540540540540459524e-01-(-theta+theta_ref)/4.00000000000000022204e-01*v_x);
    acadodata_f1 << dot(theta) == (-theta+theta_ref)/4.00000000000000022204e-01;
    acadodata_f1 << dot(zeta_w) == 3.14159265358979311600e-01*delta_w;

    OCP ocp1(0, 2, 20);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f3);
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo(0.00000000000000000000e+00 <= zeta_w <= 1.57079632679489655800e+00);
    ocp1.subjectTo((-1.00000000000000000000e+00) <= delta_w <= 1.00000000000000000000e+00);
    ocp1.subjectTo(0.00000000000000000000e+00 <= T_w <= 3.00000000000000000000e+01);
    ocp1.subjectTo((-2.61799387799149407829e-01) <= theta_ref <= 4.36332312998582383390e-01);


    ocp1.setNU( 3 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 0 );
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
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 20 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( MAX_NUM_QP_ITERATIONS, 500 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: MAX_NUM_QP_ITERATIONS");
    options_flag = ExportModule1.set( HOTSTART_QP, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HOTSTART_QP");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-10 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    options_flag = ExportModule1.set( CG_HARDCODE_CONSTRAINT_VALUES, NO );
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

