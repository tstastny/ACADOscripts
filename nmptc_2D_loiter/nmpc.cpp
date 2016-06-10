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
    DifferentialState mu;
    DifferentialState xi;
    DifferentialState mu_dot;
    Control mu_r;
    OnlineData V; 
    OnlineData c_n; 
    OnlineData c_e; 
    OnlineData R; 
    OnlineData ldir; 
    OnlineData w_n; 
    OnlineData w_e; 
    BMatrix acadodata_M1;
    acadodata_M1.read( "nmpc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "nmpc_data_acadodata_M2.txt" );
    Function acadodata_f2;
    acadodata_f2 << (-((-c_e+e)/(1.000000E-03+sqrt(((-c_e+e)*(-c_e+e)+(-c_n+n)*(-c_n+n))))*R+c_e-e)*(-(-c_e+e)/(1.000000E-03+sqrt(((-c_e+e)*(-c_e+e)+(-c_n+n)*(-c_n+n)))))*ldir+((-c_n+n)/(1.000000E-03+sqrt(((-c_e+e)*(-c_e+e)+(-c_n+n)*(-c_n+n))))*R+c_n-n)*(-c_n+n)/(1.000000E-03+sqrt(((-c_e+e)*(-c_e+e)+(-c_n+n)*(-c_n+n))))*ldir);
    acadodata_f2 << ((-(-c_e+e)/(1.000000E-03+sqrt(((-c_e+e)*(-c_e+e)+(-c_n+n)*(-c_n+n)))))*ldir-1/(1.000000E-03+sqrt(((V*cos(xi)+w_n)*(V*cos(xi)+w_n)+(V*sin(xi)+w_e)*(V*sin(xi)+w_e))))*(V*cos(xi)+w_n));
    acadodata_f2 << ((-c_n+n)/(1.000000E-03+sqrt(((-c_e+e)*(-c_e+e)+(-c_n+n)*(-c_n+n))))*ldir-1/(1.000000E-03+sqrt(((V*cos(xi)+w_n)*(V*cos(xi)+w_n)+(V*sin(xi)+w_e)*(V*sin(xi)+w_e))))*(V*sin(xi)+w_e));
    acadodata_f2 << mu;
    acadodata_f2 << mu_dot;
    acadodata_f2 << mu_r;
    acadodata_f2 << mu_r;
    Function acadodata_f3;
    acadodata_f3 << (-((-c_e+e)/(1.000000E-03+sqrt(((-c_e+e)*(-c_e+e)+(-c_n+n)*(-c_n+n))))*R+c_e-e)*(-(-c_e+e)/(1.000000E-03+sqrt(((-c_e+e)*(-c_e+e)+(-c_n+n)*(-c_n+n)))))*ldir+((-c_n+n)/(1.000000E-03+sqrt(((-c_e+e)*(-c_e+e)+(-c_n+n)*(-c_n+n))))*R+c_n-n)*(-c_n+n)/(1.000000E-03+sqrt(((-c_e+e)*(-c_e+e)+(-c_n+n)*(-c_n+n))))*ldir);
    acadodata_f3 << ((-(-c_e+e)/(1.000000E-03+sqrt(((-c_e+e)*(-c_e+e)+(-c_n+n)*(-c_n+n)))))*ldir-1/(1.000000E-03+sqrt(((V*cos(xi)+w_n)*(V*cos(xi)+w_n)+(V*sin(xi)+w_e)*(V*sin(xi)+w_e))))*(V*cos(xi)+w_n));
    acadodata_f3 << ((-c_n+n)/(1.000000E-03+sqrt(((-c_e+e)*(-c_e+e)+(-c_n+n)*(-c_n+n))))*ldir-1/(1.000000E-03+sqrt(((V*cos(xi)+w_n)*(V*cos(xi)+w_n)+(V*sin(xi)+w_e)*(V*sin(xi)+w_e))))*(V*sin(xi)+w_e));
    acadodata_f3 << mu;
    acadodata_f3 << mu_dot;
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(n) == (V*cos(xi)+w_n);
    acadodata_f1 << dot(e) == (V*sin(xi)+w_e);
    acadodata_f1 << dot(mu) == mu_dot;
    acadodata_f1 << dot(xi) == 9.810000E+00/V*tan(mu);
    acadodata_f1 << dot(mu_dot) == (1.348000E+01*mu_r-1.397000E+01*mu-6.577000E+00*mu_dot);

    OCP ocp1(0, 8, 80);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f3);
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo((-6.108652E-01) <= mu_r <= 6.108652E-01);
    ocp1.setNOD( 7 );


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
    export_flag = ExportModule1.exportCode( "export_nmpc" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

