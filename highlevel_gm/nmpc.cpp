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
    DifferentialState n;
    DifferentialState e;
    DifferentialState d;
    DifferentialState gamma;
    DifferentialState xi;
    DifferentialState phi;
    Control gamma_ref;
    Control phi_ref;
    OnlineData v; 
    OnlineData w_n; 
    OnlineData w_e; 
    OnlineData w_d; 
    OnlineData b_n; 
    OnlineData b_e; 
    OnlineData b_d; 
    OnlineData Gamma_p; 
    OnlineData chi_p; 
    OnlineData terrain_data11; 
    OnlineData terrain_data12; 
    OnlineData terrain_data13; 
    OnlineData terrain_data14; 
    OnlineData terrain_data15; 
    OnlineData terrain_data21; 
    OnlineData terrain_data22; 
    OnlineData terrain_data23; 
    OnlineData terrain_data24; 
    OnlineData terrain_data25; 
    OnlineData terrain_data31; 
    OnlineData terrain_data32; 
    OnlineData terrain_data33; 
    OnlineData terrain_data34; 
    OnlineData terrain_data35; 
    OnlineData terrain_data41; 
    OnlineData terrain_data42; 
    OnlineData terrain_data43; 
    OnlineData terrain_data44; 
    OnlineData terrain_data45; 
    OnlineData terrain_data51; 
    OnlineData terrain_data52; 
    OnlineData terrain_data53; 
    OnlineData terrain_data54; 
    OnlineData terrain_data55; 
    BMatrix acadodata_M1;
    acadodata_M1.read( "nmpc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "nmpc_data_acadodata_M2.txt" );
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(n) == (cos(gamma)*cos(xi)*v+w_n);
    acadodata_f1 << dot(e) == (cos(gamma)*sin(xi)*v+w_e);
    acadodata_f1 << dot(d) == ((-v)*sin(gamma)+w_d);
    acadodata_f1 << dot(gamma) == (-gamma+gamma_ref);
    acadodata_f1 << dot(xi) == 9.81000000000000049738e+00/cos(gamma)*tan(phi)/v;
    acadodata_f1 << dot(phi) == (-phi+phi_ref)/6.99999999999999955591e-01;

    OCP ocp1(0, 10, 100);
    ocp1.minimizeLSQ(acadodata_M1, "evaluateLSQ");
    ocp1.minimizeLSQEndTerm(acadodata_M2, "evaluateLSQEndTerm");
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo((-1.50000000000000000000e+00) <= sin(gamma_ref)*v <= 3.50000000000000000000e+00);
    ocp1.subjectTo((-6.10865238198015303439e-01) <= phi_ref <= 6.10865238198015303439e-01);
    ocp1.setNOD( 34 );


    ocp1.setNU( 2 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 34 );
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
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 100 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
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

