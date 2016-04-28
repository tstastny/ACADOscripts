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
    DifferentialState Va;
    DifferentialState beta;
    DifferentialState alpha;
    DifferentialState p;
    DifferentialState q;
    DifferentialState r;
    DifferentialState phi;
    DifferentialState theta;
    DifferentialState delT;
    DifferentialState intg_e_Va;
    DifferentialState intg_e_theta;
    DifferentialState intg_e_phi;
    DifferentialState x_w2_uT;
    DifferentialState x_w2_uE;
    DifferentialState x_w2_uA;
    DifferentialState x_w2_uR;
    Control uT;
    Control uE;
    Control uA;
    Control uR;
    OnlineData Aw2; 
    OnlineData Bw2; 
    OnlineData Cw2; 
    OnlineData Dw2; 
    OnlineData Va_cmd; 
    OnlineData theta_cmd; 
    OnlineData phi_cmd; 
    OnlineData beta_cmd; 
    OnlineData alpha_co; 
    BMatrix acadodata_M1;
    acadodata_M1.read( "nmpc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "nmpc_data_acadodata_M2.txt" );
    Function acadodata_f2;
    acadodata_f2 << (-Va+Va_cmd);
    acadodata_f2 << (-theta+theta_cmd);
    acadodata_f2 << (-phi+phi_cmd);
    acadodata_f2 << (-beta+beta_cmd);
    acadodata_f2 << p;
    acadodata_f2 << q;
    acadodata_f2 << r;
    acadodata_f2 << intg_e_Va;
    acadodata_f2 << intg_e_theta;
    acadodata_f2 << intg_e_phi;
    acadodata_f2 << 1/(1.000000E+00+exp((1/1.800000E+02*3.141593E+00*alpha_co-alpha)*3.000000E+00));
    acadodata_f2 << (Cw2*x_w2_uT+Dw2*uT);
    acadodata_f2 << (Cw2*x_w2_uE+Dw2*uE);
    acadodata_f2 << (Cw2*x_w2_uA+Dw2*uA);
    acadodata_f2 << (Cw2*x_w2_uR+Dw2*uR);
    Function acadodata_f3;
    acadodata_f3 << (-Va+Va_cmd);
    acadodata_f3 << (-theta+theta_cmd);
    acadodata_f3 << (-phi+phi_cmd);
    acadodata_f3 << (-beta+beta_cmd);
    acadodata_f3 << p;
    acadodata_f3 << q;
    acadodata_f3 << r;
    acadodata_f3 << intg_e_Va;
    acadodata_f3 << intg_e_theta;
    acadodata_f3 << intg_e_phi;
    acadodata_f3 << 1/(1.000000E+00+exp((1/1.800000E+02*3.141593E+00*alpha_co-alpha)*3.000000E+00));
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(Va) == (((((-((-1.257000E+00)*alpha+1.248000E+00)/(1.000000E+00+exp((-1.000000E+02)*(-2.570000E-01+alpha)))-((-4.683240E+01)*pow(alpha,2.000000E+00)+1.080600E+01*alpha+2.126600E-01+6.060170E+01*pow(alpha,3.000000E+00))*(1.000000E+00-1/(1.000000E+00+exp((-1.000000E+02)*(-2.570000E-01+alpha)))))*(-sin(alpha))+(-(-6.737000E-01)*alpha-1.360200E-01-5.454600E+00*pow(alpha,2.000000E+00))*cos(alpha))*4.700000E-01*5.900000E-01*pow(Va,2.000000E+00)+1.472170E+01*delT)/2.650000E+00-9.810000E+00*sin(theta)-Va*cos(beta)*q*sin(alpha)+Va*r*sin(beta))*Va*cos(alpha)*cos(beta)+(((-((-1.257000E+00)*alpha+1.248000E+00)/(1.000000E+00+exp((-1.000000E+02)*(-2.570000E-01+alpha)))-((-4.683240E+01)*pow(alpha,2.000000E+00)+1.080600E+01*alpha+2.126600E-01+6.060170E+01*pow(alpha,3.000000E+00))*(1.000000E+00-1/(1.000000E+00+exp((-1.000000E+02)*(-2.570000E-01+alpha)))))*cos(alpha)+(-(-6.737000E-01)*alpha-1.360200E-01-5.454600E+00*pow(alpha,2.000000E+00))*sin(alpha))*4.700000E-01*5.900000E-01*pow(Va,2.000000E+00)/2.650000E+00+9.810000E+00*cos(phi)*cos(theta)+Va*cos(alpha)*cos(beta)*q-Va*p*sin(beta))*Va*cos(beta)*sin(alpha)+((-3.073300E-01)*beta/2.650000E+00*4.700000E-01*5.900000E-01*pow(Va,2.000000E+00)+9.810000E+00*cos(theta)*sin(phi)-Va*cos(alpha)*cos(beta)*r+Va*cos(beta)*p*sin(alpha))*Va*sin(beta))/Va;
    acadodata_f1 << dot(beta) == (-(((((-((-1.257000E+00)*alpha+1.248000E+00)/(1.000000E+00+exp((-1.000000E+02)*(-2.570000E-01+alpha)))-((-4.683240E+01)*pow(alpha,2.000000E+00)+1.080600E+01*alpha+2.126600E-01+6.060170E+01*pow(alpha,3.000000E+00))*(1.000000E+00-1/(1.000000E+00+exp((-1.000000E+02)*(-2.570000E-01+alpha)))))*(-sin(alpha))+(-(-6.737000E-01)*alpha-1.360200E-01-5.454600E+00*pow(alpha,2.000000E+00))*cos(alpha))*4.700000E-01*5.900000E-01*pow(Va,2.000000E+00)+1.472170E+01*delT)/2.650000E+00-9.810000E+00*sin(theta)-Va*cos(beta)*q*sin(alpha)+Va*r*sin(beta))*Va*cos(alpha)*cos(beta)+(((-((-1.257000E+00)*alpha+1.248000E+00)/(1.000000E+00+exp((-1.000000E+02)*(-2.570000E-01+alpha)))-((-4.683240E+01)*pow(alpha,2.000000E+00)+1.080600E+01*alpha+2.126600E-01+6.060170E+01*pow(alpha,3.000000E+00))*(1.000000E+00-1/(1.000000E+00+exp((-1.000000E+02)*(-2.570000E-01+alpha)))))*cos(alpha)+(-(-6.737000E-01)*alpha-1.360200E-01-5.454600E+00*pow(alpha,2.000000E+00))*sin(alpha))*4.700000E-01*5.900000E-01*pow(Va,2.000000E+00)/2.650000E+00+9.810000E+00*cos(phi)*cos(theta)+Va*cos(alpha)*cos(beta)*q-Va*p*sin(beta))*Va*cos(beta)*sin(alpha)+((-3.073300E-01)*beta/2.650000E+00*4.700000E-01*5.900000E-01*pow(Va,2.000000E+00)+9.810000E+00*cos(theta)*sin(phi)-Va*cos(alpha)*cos(beta)*r+Va*cos(beta)*p*sin(alpha))*Va*sin(beta))*Va/Va*sin(beta)+((-3.073300E-01)*beta/2.650000E+00*4.700000E-01*5.900000E-01*pow(Va,2.000000E+00)+9.810000E+00*cos(theta)*sin(phi)-Va*cos(alpha)*cos(beta)*r+Va*cos(beta)*p*sin(alpha))*Va)/cos(beta)/pow(Va,2.000000E+00);
    acadodata_f1 << dot(alpha) == (-((((-((-1.257000E+00)*alpha+1.248000E+00)/(1.000000E+00+exp((-1.000000E+02)*(-2.570000E-01+alpha)))-((-4.683240E+01)*pow(alpha,2.000000E+00)+1.080600E+01*alpha+2.126600E-01+6.060170E+01*pow(alpha,3.000000E+00))*(1.000000E+00-1/(1.000000E+00+exp((-1.000000E+02)*(-2.570000E-01+alpha)))))*(-sin(alpha))+(-(-6.737000E-01)*alpha-1.360200E-01-5.454600E+00*pow(alpha,2.000000E+00))*cos(alpha))*4.700000E-01*5.900000E-01*pow(Va,2.000000E+00)+1.472170E+01*delT)/2.650000E+00-9.810000E+00*sin(theta)-Va*cos(beta)*q*sin(alpha)+Va*r*sin(beta))*Va*cos(beta)*sin(alpha)+(((-((-1.257000E+00)*alpha+1.248000E+00)/(1.000000E+00+exp((-1.000000E+02)*(-2.570000E-01+alpha)))-((-4.683240E+01)*pow(alpha,2.000000E+00)+1.080600E+01*alpha+2.126600E-01+6.060170E+01*pow(alpha,3.000000E+00))*(1.000000E+00-1/(1.000000E+00+exp((-1.000000E+02)*(-2.570000E-01+alpha)))))*cos(alpha)+(-(-6.737000E-01)*alpha-1.360200E-01-5.454600E+00*pow(alpha,2.000000E+00))*sin(alpha))*4.700000E-01*5.900000E-01*pow(Va,2.000000E+00)/2.650000E+00+9.810000E+00*cos(phi)*cos(theta)+Va*cos(alpha)*cos(beta)*q-Va*p*sin(beta))*Va*cos(alpha)*cos(beta))/(pow(Va*cos(alpha)*cos(beta),2.000000E+00)+pow(Va*cos(beta)*sin(alpha),2.000000E+00));
    acadodata_f1 << dot(p) == (((-1.541900E-02)*beta+(-1.646900E-01)/2.000000E+00*2.590000E+00/Va*p+1.168500E-02/2.000000E+00*2.590000E+00/Va*r+5.700000E-02*uA)*2.590000E+00*4.700000E-01*5.243000E-01*5.900000E-01*pow(Va,2.000000E+00)-((-2.270436E-02)*p+7.616617E-02*r)*q+((-8.267800E-02)/2.000000E+00*2.590000E+00/Va*r+(-8.385300E-02)/2.000000E+00*2.590000E+00/Va*p+4.298700E-02*beta+6.000000E-02*uR)*2.590000E+00*4.700000E-01*5.900000E-01*7.550000E-02*pow(Va,2.000000E+00))/8.150133E-02;
    acadodata_f1 << dot(q) == (((-1.061541E+02)*1.800000E-01/2.000000E+00/Va*q+(-2.969000E+00)*alpha+(-6.130800E+00)*uE+4.350100E-02)*1.800000E-01*4.700000E-01*5.900000E-01*pow(Va,2.000000E+00)-(-3.579800E-01)*p*r-(pow(p,2.000000E+00)-pow(r,2.000000E+00))*7.550000E-02)/3.899000E-01;
    acadodata_f1 << dot(r) == (((-1.541900E-02)*beta+(-1.646900E-01)/2.000000E+00*2.590000E+00/Va*p+1.168500E-02/2.000000E+00*2.590000E+00/Va*r+5.700000E-02*uA)*2.590000E+00*4.700000E-01*5.900000E-01*7.550000E-02*pow(Va,2.000000E+00)+((-2.270436E-02)*r+(-3.148558E-02)*p)*q+((-8.267800E-02)/2.000000E+00*2.590000E+00/Va*r+(-8.385300E-02)/2.000000E+00*2.590000E+00/Va*p+4.298700E-02*beta+6.000000E-02*uR)*1.663200E-01*2.590000E+00*4.700000E-01*5.900000E-01*pow(Va,2.000000E+00))/8.150133E-02;
    acadodata_f1 << dot(phi) == ((cos(phi)*r+q*sin(phi))*tan(theta)+p);
    acadodata_f1 << dot(theta) == (cos(phi)*q-r*sin(phi));
    acadodata_f1 << dot(delT) == (-delT+uT)/8.800000E-02;
    acadodata_f1 << dot(intg_e_Va) == (-Va+Va_cmd);
    acadodata_f1 << dot(intg_e_theta) == (-theta+theta_cmd);
    acadodata_f1 << dot(intg_e_phi) == (-phi+phi_cmd);
    acadodata_f1 << dot(x_w2_uT) == (Aw2*x_w2_uT+Bw2*uT);
    acadodata_f1 << dot(x_w2_uE) == (Aw2*x_w2_uE+Bw2*uE);
    acadodata_f1 << dot(x_w2_uA) == (Aw2*x_w2_uA+Bw2*uA);
    acadodata_f1 << dot(x_w2_uR) == (Aw2*x_w2_uR+Bw2*uR);

    OCP ocp1(0, 1, 20);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f3);
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo(0.000000E+00 <= uT <= 1.000000E+00);
    ocp1.subjectTo((-3.490659E-01) <= uE <= 3.490659E-01);
    ocp1.subjectTo((-3.490659E-01) <= uA <= 3.490659E-01);
    ocp1.subjectTo((-3.490659E-01) <= uR <= 3.490659E-01);
    ocp1.subjectTo((-7.853982E-01) <= phi <= 7.853982E-01);
    ocp1.subjectTo((-7.853982E-01) <= theta <= 7.853982E-01);
    ocp1.subjectTo(alpha <= 1.745329E-01);
    ocp1.setNOD( 9 );


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
    export_flag = ExportModule1.exportCode( "export_nmpc" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

