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
    DifferentialState V;
    DifferentialState gamma;
    DifferentialState xi;
    DifferentialState phi;
    DifferentialState theta;
    DifferentialState p;
    DifferentialState q;
    DifferentialState r;
    DifferentialState delta_T;
    DifferentialState sw;
    Control u_T;
    Control phi_ref;
    Control theta_ref;
    OnlineData pparam1; 
    OnlineData pparam2; 
    OnlineData pparam3; 
    OnlineData pparam4; 
    OnlineData pparam5; 
    OnlineData pparam6; 
    OnlineData pparam7; 
    OnlineData pparam8; 
    OnlineData pparam9; 
    OnlineData pparam1_next; 
    OnlineData pparam2_next; 
    OnlineData pparam3_next; 
    OnlineData pparam4_next; 
    OnlineData pparam5_next; 
    OnlineData pparam6_next; 
    OnlineData pparam7_next; 
    OnlineData pparam8_next; 
    OnlineData pparam9_next; 
    OnlineData R_acpt; 
    OnlineData ceta_acpt; 
    OnlineData wn; 
    OnlineData we; 
    OnlineData wd; 
    OnlineData alpha_p_co; 
    OnlineData alpha_m_co; 
    OnlineData alpha_delta_co; 
    OnlineData T_b_ne; 
    OnlineData T_b_d; 
    BMatrix acadodata_M1;
    acadodata_M1.read( "nmpc_ext_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "nmpc_ext_data_acadodata_M2.txt" );
    OCP ocp1(0, 8, 80);
    ocp1.minimizeLSQ(acadodata_M1, "evaluateLSQ");
    ocp1.minimizeLSQEndTerm(acadodata_M2, "evaluateLSQEndTerm");
    ocp1.subjectTo(0.000000E+00 <= u_T <= 1.000000E+00);
    ocp1.subjectTo((-5.235988E-01) <= phi_ref <= 5.235988E-01);
    ocp1.subjectTo((-2.617994E-01) <= theta_ref <= 2.617994E-01);
    ocp1.setNOD( 28 );
    ocp1.setNP( 0 );
    ocp1.setNU( 3 );
    ocp1.setModel( "model", "rhs", "rhs_jac" );
    ocp1.setDimensions( 0, 13, 0, 0, 0, 3, 28, 0 );


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
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 80 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( HOTSTART_QP, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HOTSTART_QP");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-10 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    options_flag = ExportModule1.set( GENERATE_MAKE_FILE, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: GENERATE_MAKE_FILE");
    options_flag = ExportModule1.set( GENERATE_TEST_FILE, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: GENERATE_TEST_FILE");
    options_flag = ExportModule1.set( GENERATE_SIMULINK_INTERFACE, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: GENERATE_SIMULINK_INTERFACE");
    options_flag = ExportModule1.set( CG_HARDCODE_CONSTRAINT_VALUES, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: CG_HARDCODE_CONSTRAINT_VALUES");
    options_flag = ExportModule1.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: CG_USE_VARIABLE_WEIGHTING_MATRIX");
    uint export_flag;
    export_flag = ExportModule1.exportCode( "export_nmpc_3dhl_eta" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

