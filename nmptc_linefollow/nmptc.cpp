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
    DifferentialState mu;
    DifferentialState gamma;
    DifferentialState xi;
    Control mu_dot;
    Control gamma_dot;
    OnlineData V; 
    OnlineData wn; 
    OnlineData we; 
    OnlineData wd; 
    OnlineData an; 
    OnlineData ae; 
    OnlineData ad; 
    OnlineData bn; 
    OnlineData be; 
    OnlineData bd; 
    BMatrix acadodata_M1;
    acadodata_M1.read( "nmptc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "nmptc_data_acadodata_M2.txt" );
    Function acadodata_f2;
    acadodata_f2 << sqrt((pow(((-ad+bd)*(an-n)/sqrt(((-ad+bd)*(-ad+bd)+(-ae+be)*(-ae+be)+(-an+bn)*(-an+bn)))-(-an+bn)*(ad-d)/sqrt(((-ad+bd)*(-ad+bd)+(-ae+be)*(-ae+be)+(-an+bn)*(-an+bn)))),2.000000E+00)+pow((-(-ad+bd)*(ae-e)/sqrt(((-ad+bd)*(-ad+bd)+(-ae+be)*(-ae+be)+(-an+bn)*(-an+bn)))+(-ae+be)*(ad-d)/sqrt(((-ad+bd)*(-ad+bd)+(-ae+be)*(-ae+be)+(-an+bn)*(-an+bn)))),2.000000E+00)+pow((-(-ae+be)*(an-n)/sqrt(((-ad+bd)*(-ad+bd)+(-ae+be)*(-ae+be)+(-an+bn)*(-an+bn)))+(-an+bn)*(ae-e)/sqrt(((-ad+bd)*(-ad+bd)+(-ae+be)*(-ae+be)+(-an+bn)*(-an+bn)))),2.000000E+00)));
    acadodata_f2 << (asin(((-V)*sin(gamma)+wd)/sqrt((pow(((-V)*sin(gamma)+wd),2.000000E+00)+pow((V*cos(gamma)*cos(xi)+wn),2.000000E+00)+pow((V*cos(gamma)*sin(xi)+we),2.000000E+00))))-asin((-ad+bd)/sqrt(((-ad+bd)*(-ad+bd)+(-ae+be)*(-ae+be)+(-an+bn)*(-an+bn)))));
    acadodata_f2 << (atan((-ae+be)/(-an+bn)/sqrt(((-ad+bd)*(-ad+bd)+(-ae+be)*(-ae+be)+(-an+bn)*(-an+bn)))*sqrt(((-ad+bd)*(-ad+bd)+(-ae+be)*(-ae+be)+(-an+bn)*(-an+bn))))-atan(1/(V*cos(gamma)*cos(xi)+wn)*(V*cos(gamma)*sin(xi)+we)));
    acadodata_f2 << mu_dot;
    acadodata_f2 << gamma_dot;
    Function acadodata_f3;
    acadodata_f3 << sqrt((pow(((-ad+bd)*(an-n)/sqrt(((-ad+bd)*(-ad+bd)+(-ae+be)*(-ae+be)+(-an+bn)*(-an+bn)))-(-an+bn)*(ad-d)/sqrt(((-ad+bd)*(-ad+bd)+(-ae+be)*(-ae+be)+(-an+bn)*(-an+bn)))),2.000000E+00)+pow((-(-ad+bd)*(ae-e)/sqrt(((-ad+bd)*(-ad+bd)+(-ae+be)*(-ae+be)+(-an+bn)*(-an+bn)))+(-ae+be)*(ad-d)/sqrt(((-ad+bd)*(-ad+bd)+(-ae+be)*(-ae+be)+(-an+bn)*(-an+bn)))),2.000000E+00)+pow((-(-ae+be)*(an-n)/sqrt(((-ad+bd)*(-ad+bd)+(-ae+be)*(-ae+be)+(-an+bn)*(-an+bn)))+(-an+bn)*(ae-e)/sqrt(((-ad+bd)*(-ad+bd)+(-ae+be)*(-ae+be)+(-an+bn)*(-an+bn)))),2.000000E+00)));
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(n) == (V*cos(gamma)*cos(xi)+wn);
    acadodata_f1 << dot(e) == (V*cos(gamma)*sin(xi)+we);
    acadodata_f1 << dot(d) == ((-V)*sin(gamma)+wd);
    acadodata_f1 << dot(mu) == mu_dot;
    acadodata_f1 << dot(gamma) == gamma_dot;
    acadodata_f1 << dot(xi) == 9.810000E+00/V*tan(mu);

    OCP ocp1(0, 4, 20);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f3);
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo((-5.235988E-01) <= mu <= 3.000000E+01);
    ocp1.subjectTo((-2.617994E-01) <= gamma <= 2.617994E-01);
    ocp1.setNOD( 10 );


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
    options_flag = ExportModule1.set( HOTSTART_QP, NO );
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
    uint export_flag;
    export_flag = ExportModule1.exportCode( "export_nmptc" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

