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
    DifferentialState u;
    DifferentialState v;
    DifferentialState w;
    DifferentialState p;
    DifferentialState q;
    DifferentialState r;
    DifferentialState phi;
    DifferentialState theta;
    DifferentialState dummy;
    DifferentialState intg_V;
    DifferentialState x_w1_V;
    DifferentialState x_w1_theta;
    DifferentialState x_w1_phi;
    DifferentialState x_w1_beta;
    DifferentialState x_w2_uT;
    DifferentialState x_w2_uE;
    DifferentialState x_w2_uA;
    DifferentialState x_w2_uR;
    DifferentialState x_w2_sv;
    DifferentialState x_w3_p;
    DifferentialState x_w3_q;
    DifferentialState x_w3_r;
    Control uT;
    Control uE;
    Control uA;
    Control uR;
    Control sv;
    OnlineData Aw1; 
    OnlineData Bw1; 
    OnlineData Cw1; 
    OnlineData Dw1; 
    OnlineData Aw2; 
    OnlineData Bw2; 
    OnlineData Cw2; 
    OnlineData Dw2; 
    OnlineData Aw3; 
    OnlineData Bw3; 
    OnlineData Cw3; 
    OnlineData Dw3; 
    OnlineData V_cmd; 
    OnlineData theta_cmd; 
    OnlineData phi_cmd; 
    OnlineData kiV; 
    BMatrix acadodata_M1;
    acadodata_M1.read( "nmpc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "nmpc_data_acadodata_M2.txt" );
    Function acadodata_f2;
    acadodata_f2 << ((V_cmd+intg_V-sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00))))*Dw1+Cw1*x_w1_V);
    acadodata_f2 << ((-theta+theta_cmd)*Dw1+Cw1*x_w1_theta);
    acadodata_f2 << ((-phi+phi_cmd)*Dw1+Cw1*x_w1_phi);
    acadodata_f2 << (Cw1*x_w1_beta+Dw1*asin(1/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))*v));
    acadodata_f2 << (Cw3*x_w3_p+Dw3*p);
    acadodata_f2 << (Cw3*x_w3_q+Dw3*q);
    acadodata_f2 << (Cw3*x_w3_r+Dw3*r);
    acadodata_f2 << (Cw2*x_w2_uT+Dw2*uT);
    acadodata_f2 << (Cw2*x_w2_uE+Dw2*uE);
    acadodata_f2 << (Cw2*x_w2_uA+Dw2*uA);
    acadodata_f2 << (Cw2*x_w2_uR+Dw2*uR);
    acadodata_f2 << (Cw2*x_w2_sv+Dw2*sv);
    Function acadodata_f3;
    acadodata_f3 << ((V_cmd+intg_V-sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00))))*Dw1+Cw1*x_w1_V);
    acadodata_f3 << ((-theta+theta_cmd)*Dw1+Cw1*x_w1_theta);
    acadodata_f3 << ((-phi+phi_cmd)*Dw1+Cw1*x_w1_phi);
    acadodata_f3 << (Cw1*x_w1_beta+Dw1*asin(1/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))*v));
    acadodata_f3 << (Cw3*x_w3_p+Dw3*p);
    acadodata_f3 << (Cw3*x_w3_q+Dw3*q);
    acadodata_f3 << (Cw3*x_w3_r+Dw3*r);
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(u) == ((((-(-1.175400E-01)-(-3.492780E+01)*pow(atan(1/u*w),2.000000E+00)-1.085200E+01*atan(1/u*w)-3.453380E+01*pow(atan(1/u*w),3.000000E+00))*(-sin(atan(1/u*w)))+(-1.199300E+00*pow(atan(1/u*w),2.000000E+00)-4.619400E-01*atan(1/u*w)-6.287400E-02)*cos(atan(1/u*w)))*4.700000E-01*5.900000E-01*pow(sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00))),2.000000E+00)+1.249070E+01*uT)/2.650000E+00-9.810000E+00*sin(theta)-q*w+r*v);
    acadodata_f1 << dot(v) == ((-3.073300E-01)*asin(1/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))*v)/2.650000E+00*4.700000E-01*5.900000E-01*pow(sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00))),2.000000E+00)+9.810000E+00*cos(theta)*sin(phi)+p*w-r*u);
    acadodata_f1 << dot(w) == (((-(-1.175400E-01)-(-3.492780E+01)*pow(atan(1/u*w),2.000000E+00)-1.085200E+01*atan(1/u*w)-3.453380E+01*pow(atan(1/u*w),3.000000E+00))*cos(atan(1/u*w))+(-1.199300E+00*pow(atan(1/u*w),2.000000E+00)-4.619400E-01*atan(1/u*w)-6.287400E-02)*sin(atan(1/u*w)))*4.700000E-01*5.900000E-01*pow(sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00))),2.000000E+00)/2.650000E+00+9.810000E+00*cos(phi)*cos(theta)-p*v+q*u);
    acadodata_f1 << dot(p) == ((((-1.541900E-02)*asin(1/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))*v)+(-1.646900E-01)/2.000000E+00*2.590000E+00*p/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))+(-5.772300E-02)*uA+1.168500E-02/2.000000E+00*2.590000E+00*r/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00))))*cos(atan(1/u*w))+((-6.094500E-02)*uR+(-8.267800E-02)/2.000000E+00*2.590000E+00*r/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))+(-8.385300E-02)/2.000000E+00*2.590000E+00*p/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))+4.298700E-02*asin(1/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))*v))*(-sin(atan(1/u*w))))*2.590000E+00*4.700000E-01*5.243000E-01*5.900000E-01*pow(sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00))),2.000000E+00)+(((-1.541900E-02)*asin(1/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))*v)+(-1.646900E-01)/2.000000E+00*2.590000E+00*p/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))+(-5.772300E-02)*uA+1.168500E-02/2.000000E+00*2.590000E+00*r/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00))))*sin(atan(1/u*w))+((-6.094500E-02)*uR+(-8.267800E-02)/2.000000E+00*2.590000E+00*r/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))+(-8.385300E-02)/2.000000E+00*2.590000E+00*p/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))+4.298700E-02*asin(1/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))*v))*cos(atan(1/u*w)))*2.590000E+00*4.700000E-01*5.900000E-01*7.550000E-02*pow(sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00))),2.000000E+00)-((-2.270436E-02)*p+7.616617E-02*r)*q)/8.150133E-02;
    acadodata_f1 << dot(q) == (((-2.062100E+00)*atan(1/u*w)+(-3.526200E+00)*uE+(-4.997230E+01)*1.800000E-01/2.000000E+00*q/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))+9.891100E-02)*1.800000E-01*4.700000E-01*5.900000E-01*pow(sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00))),2.000000E+00)-(-3.579800E-01)*p*r-(pow(p,2.000000E+00)-pow(r,2.000000E+00))*7.550000E-02)/3.899000E-01;
    acadodata_f1 << dot(r) == ((((-1.541900E-02)*asin(1/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))*v)+(-1.646900E-01)/2.000000E+00*2.590000E+00*p/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))+(-5.772300E-02)*uA+1.168500E-02/2.000000E+00*2.590000E+00*r/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00))))*cos(atan(1/u*w))+((-6.094500E-02)*uR+(-8.267800E-02)/2.000000E+00*2.590000E+00*r/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))+(-8.385300E-02)/2.000000E+00*2.590000E+00*p/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))+4.298700E-02*asin(1/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))*v))*(-sin(atan(1/u*w))))*2.590000E+00*4.700000E-01*5.900000E-01*7.550000E-02*pow(sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00))),2.000000E+00)+(((-1.541900E-02)*asin(1/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))*v)+(-1.646900E-01)/2.000000E+00*2.590000E+00*p/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))+(-5.772300E-02)*uA+1.168500E-02/2.000000E+00*2.590000E+00*r/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00))))*sin(atan(1/u*w))+((-6.094500E-02)*uR+(-8.267800E-02)/2.000000E+00*2.590000E+00*r/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))+(-8.385300E-02)/2.000000E+00*2.590000E+00*p/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))+4.298700E-02*asin(1/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))*v))*cos(atan(1/u*w)))*1.663200E-01*2.590000E+00*4.700000E-01*5.900000E-01*pow(sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00))),2.000000E+00)+((-2.270436E-02)*r+(-3.148558E-02)*p)*q)/8.150133E-02;
    acadodata_f1 << dot(phi) == ((cos(phi)*r+q*sin(phi))*tan(theta)+p);
    acadodata_f1 << dot(theta) == (cos(phi)*q-r*sin(phi));
    acadodata_f1 << dot(dummy) == sv;
    acadodata_f1 << dot(intg_V) == kiV*sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)));
    acadodata_f1 << dot(x_w1_V) == ((V_cmd+intg_V-sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00))))*Bw1+Aw1*x_w1_V);
    acadodata_f1 << dot(x_w1_theta) == ((-theta+theta_cmd)*Bw1+Aw1*x_w1_theta);
    acadodata_f1 << dot(x_w1_phi) == ((-phi+phi_cmd)*Bw1+Aw1*x_w1_phi);
    acadodata_f1 << dot(x_w1_beta) == (Aw1*x_w1_beta+Bw1*asin(1/sqrt((pow(u,2.000000E+00)+pow(v,2.000000E+00)+pow(w,2.000000E+00)))*v));
    acadodata_f1 << dot(x_w2_uT) == (Aw2*x_w2_uT+Bw2*uT);
    acadodata_f1 << dot(x_w2_uE) == (Aw2*x_w2_uE+Bw2*uE);
    acadodata_f1 << dot(x_w2_uA) == (Aw2*x_w2_uA+Bw2*uA);
    acadodata_f1 << dot(x_w2_uR) == (Aw2*x_w2_uR+Bw2*uR);
    acadodata_f1 << dot(x_w2_sv) == (Aw2*x_w2_sv+Bw2*sv);
    acadodata_f1 << dot(x_w3_p) == (Aw3*x_w3_p+Bw3*p);
    acadodata_f1 << dot(x_w3_q) == (Aw3*x_w3_q+Bw3*q);
    acadodata_f1 << dot(x_w3_r) == (Aw3*x_w3_r+Bw3*r);

    OCP ocp1(0, 1, 20);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f3);
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo(0.000000E+00 <= uT <= 1.000000E+00);
    ocp1.subjectTo((-3.490659E-01) <= uE <= 3.490659E-01);
    ocp1.subjectTo((-3.490659E-01) <= uA <= 3.490659E-01);
    ocp1.subjectTo((-3.490659E-01) <= uR <= 3.490659E-01);
    ocp1.subjectTo((-1.047198E+00) <= phi <= 1.047198E+00);
    ocp1.subjectTo((-7.853982E-01) <= theta <= 7.853982E-01);
    ocp1.subjectTo((-3.490659E-02) <= (atan(1/u*w)+sv));
    ocp1.subjectTo((atan(1/u*w)-sv) <= 1.745329E-01);
    ocp1.subjectTo(0.000000E+00 <= sv <= 8.726646E-02);
    ocp1.setNOD( 16 );


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

