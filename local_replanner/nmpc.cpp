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
    DifferentialState pos_n;
    DifferentialState pos_e;
    DifferentialState pos_d;
    DifferentialState airsp;
    DifferentialState fpa;
    DifferentialState heading;
    DifferentialState roll;
    DifferentialState pitch;
    DifferentialState prop_speed;
    Control throt;
    Control roll_ref;
    Control pitch_ref;
    OnlineData air_density; 
    OnlineData wind_n; 
    OnlineData wind_e; 
    OnlineData wind_d; 
    OnlineData tau_roll; 
    OnlineData tau_pitch; 
    OnlineData k_roll; 
    OnlineData k_pitch; 
    OnlineData tau_prop; 
    OnlineData flaps; 
    OnlineData fpa_ref; 
    OnlineData jac_fpa_ref; 
    OnlineData heading_ref; 
    OnlineData soft_airsp; 
    OnlineData jac_soft_airsp; 
    OnlineData soft_aoa; 
    OnlineData jac_soft_aoa1; 
    OnlineData jac_soft_aoa2; 
    OnlineData soft_hagl; 
    OnlineData jac_soft_hagl1; 
    OnlineData jac_soft_hagl2; 
    OnlineData jac_soft_hagl3; 
    OnlineData jac_soft_hagl4; 
    OnlineData soft_rtd; 
    OnlineData jac_soft_rtd1; 
    OnlineData jac_soft_rtd2; 
    OnlineData jac_soft_rtd3; 
    OnlineData jac_soft_rtd4; 
    OnlineData jac_soft_rtd5; 
    OnlineData jac_soft_rtd6; 
    BMatrix acadodata_M1;
    acadodata_M1.read( "nmpc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "nmpc_data_acadodata_M2.txt" );
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(pos_n) == (airsp*cos(fpa)*cos(heading)+wind_n);
    acadodata_f1 << dot(pos_e) == (airsp*cos(fpa)*sin(heading)+wind_e);
    acadodata_f1 << dot(pos_d) == ((-airsp)*sin(fpa)+wind_d);
    acadodata_f1 << dot(airsp) == ((-(((-8.98999999999999938050e-02)*flaps+5.07639999999999980140e-01*pow(flaps,2.00000000000000000000e+00))*2.00000000000000000000e+00+(-fpa+pitch)*3.47380000000000022098e-01+2.59919999999999984386e-01*pow((-fpa+pitch),2.00000000000000000000e+00)+6.03139999999999995572e-02)*4.17999999999999982681e-01*5.00000000000000000000e-01*air_density*pow(airsp,2.00000000000000000000e+00)+((-1.32619999999999987894e-01)/2.80000000000000026645e-01*airsp*cos((-3.49065850398865909487e-02-fpa+pitch))/prop_speed+1.15210000000000006848e-01)*6.14656000000000229955e-03*air_density*cos((-fpa+pitch))*pow(prop_speed,2.00000000000000000000e+00))*3.20102432778489098819e-01-9.81000000000000049738e+00*sin(fpa));
    acadodata_f1 << dot(fpa) == ((((-1.32619999999999987894e-01)/2.80000000000000026645e-01*airsp*cos((-3.49065850398865909487e-02-fpa+pitch))/prop_speed+1.15210000000000006848e-01)*6.14656000000000229955e-03*air_density*pow(prop_speed,2.00000000000000000000e+00)*sin((-fpa+pitch))+((-2.03799999999999981171e+00)*flaps+(-fpa+pitch)*2.85009999999999985576e+00+5.21549999999999958078e-01)*4.17999999999999982681e-01*5.00000000000000000000e-01*air_density*pow(airsp,2.00000000000000000000e+00))*cos(roll)-3.06464400000000019020e+01*cos(fpa))*3.20102432778489098819e-01/airsp;
    acadodata_f1 << dot(heading) == (((-1.32619999999999987894e-01)/2.80000000000000026645e-01*airsp*cos((-3.49065850398865909487e-02-fpa+pitch))/prop_speed+1.15210000000000006848e-01)*6.14656000000000229955e-03*air_density*pow(prop_speed,2.00000000000000000000e+00)*sin((-fpa+pitch))+((-2.03799999999999981171e+00)*flaps+(-fpa+pitch)*2.85009999999999985576e+00+5.21549999999999958078e-01)*4.17999999999999982681e-01*5.00000000000000000000e-01*air_density*pow(airsp,2.00000000000000000000e+00))/3.12400000000000011013e+00/airsp/cos(fpa)*sin(roll);
    acadodata_f1 << dot(roll) == (k_roll*roll_ref-roll)/tau_roll;
    acadodata_f1 << dot(pitch) == (k_pitch*pitch_ref-pitch)/tau_pitch;
    acadodata_f1 << dot(prop_speed) == ((-(-1.00000000000000000000e+01+airsp*cos((-3.49065850398865909487e-02-fpa+pitch)))/1.50000000000000000000e+01+1.00000000000000000000e+00)*(1.22767629097648992342e+02*throt+4.08990375690176648504e+01)+(-1.00000000000000000000e+01+airsp*cos((-3.49065850398865909487e-02-fpa+pitch)))*(1.02693635333290359313e+02+6.09730313333762978800e+01*throt)/1.50000000000000000000e+01-prop_speed)/tau_prop;

    OCP ocp1(0, 5, 50);
    ocp1.minimizeLSQ(acadodata_M1, "evaluateLSQ");
    ocp1.minimizeLSQEndTerm(acadodata_M2, "evaluateLSQEndTerm");
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo(0.00000000000000000000e+00 <= throt <= 1.00000000000000000000e+00);
    ocp1.subjectTo((-6.10865238198015303439e-01) <= roll_ref <= 6.10865238198015303439e-01);
    ocp1.subjectTo((-2.61799387799149407829e-01) <= pitch_ref <= 4.36332312998582383390e-01);
    ocp1.setNOD( 30 );


    ocp1.setNU( 3 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 30 );
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

