% -------------------------------------------------------------------------
% Tiltwing MPC
% -------------------------------------------------------------------------
clc;
clear all;
close all;

addpath('functions');

Ts = 0.1;   % model discretization step [0.1 s]
N = 20;     % horizon

% states
DifferentialState v_x;
DifferentialState v_z;
DifferentialState theta;
DifferentialState zeta_w;

% controls
Control delta_w;
Control T_w;
Control theta_ref;

% MODEL - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

% parameters
m = 1.85;       % mass of the plane [kg]
g = 9.81;       % gravity [m/s^2]
rho = 1.225;    % density of air [kg/m^3]
S_w = 0.15;   % surface area of wing [m^2]
k_T2L = 0.3;    % control augmented slipstream gain
k = 1;          % control augmented atitude gain
tau_0 = 0.4;    % control augmente time constant 
c_w = pi/10;    % slew rate of the wing [rad/s]

% intermediate states
alpha = atan(v_z/v_x);

% state differentials
dot_v_x = 1/m*(T_w*cos(zeta_w)-m*g*sin(theta)-k_T2L*T_w*sin(zeta_w)...
    +0.5*rho*(v_x^2+v_z^2)*S_w*(C_Ltotal(alpha+zeta_w)*sin(alpha)...
    -C_Dtotal(alpha+zeta_w)*cos(alpha)))+k*(theta_ref - theta)/tau_0*v_z;
dot_v_z = 1/m*(-T_w*sin(zeta_w)+m*g*cos(theta)-k_T2L*T_w*cos(zeta_w)...
    -0.5*rho*(v_x^2+v_z^2)*S_w*(C_Ltotal(alpha+zeta_w)*cos(alpha)...
    -C_Dtotal(alpha+zeta_w)*sin(alpha)))-k*(theta_ref - theta)/tau_0*v_x;
dot_theta = k*(theta_ref - theta)/tau_0;
dot_zeta_w = c_w*delta_w;

% ode
f = acado.DifferentialEquation();
f.add(dot(v_x) == dot_v_x);
f.add(dot(v_z) == dot_v_z);
f.add(dot(theta) == dot_theta);
f.add(dot(zeta_w) == dot_zeta_w);

% OBJECTIVES - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

% state (only) objectives
y_s = [v_x; v_z; theta; zeta_w];

% control dependent objectives
y_c = [delta_w; T_w; theta_ref];

% OPTIMAL CONTROL PROBLEM - - - - - - - - - - - - - - - - - - - - - - - - -

% lengths
n_X = length(diffStates);   % states
n_U = length(controls);     % controls
n_Ys = length(y_s);         % state (only) objectives
n_Yc = length(y_c);         % control dependent objectives

acadoSet('problemname', 'nmpc');

% optimal control problem
ocp = acado.OCP( 0.0, N*Ts, N );

% objective weight
Q = eye(n_Ys+n_Yc,n_Ys+n_Yc);
%Q(1,1) = 10.0;
%Q(2,2) = 10.0;
Q = acado.BMatrix(Q);
QN = eye(n_Ys,n_Ys);
%QN(1,1) = 10;
%QN(2,2) = 10.0;
QN = acado.BMatrix(QN);

% objective function
ocp.minimizeLSQ( Q, [y_s; y_c] );
ocp.minimizeLSQEndTerm( QN, y_s );

% constraints
ocp.subjectTo( f );

% NOTE: in the case of non-hardcoded constraints, these values are only
%       used for initializing the solver
ocp.subjectTo( 0.0 <= zeta_w <= pi/2);
ocp.subjectTo( -1.0 <= delta_w <= 1.0 );
ocp.subjectTo( 0.0 <= T_w <= 30.0 );
ocp.subjectTo( deg2rad(-15) <= theta_ref <= deg2rad(25));

% export settings
nmpc = acado.OCPexport( ocp );
nmpc.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON' );
nmpc.set( 'DISCRETIZATION_TYPE', 'MULTIPLE_SHOOTING' );
nmpc.set( 'SPARSE_QP_SOLUTION', 'FULL_CONDENSING' );
nmpc.set( 'INTEGRATOR_TYPE', 'INT_IRK_GL4' );
nmpc.set( 'NUM_INTEGRATOR_STEPS', N  );
nmpc.set( 'QP_SOLVER', 'QP_QPOASES' );
nmpc.set( 'MAX_NUM_QP_ITERATIONS', 500 );
nmpc.set( 'HOTSTART_QP', 'YES' );
nmpc.set( 'LEVENBERG_MARQUARDT', 1e-10 );
nmpc.set( 'CG_HARDCODE_CONSTRAINT_VALUES', 'NO' ); % !!! if you want hard-coded constraints, set this to yes, otherwise they must be input !!!
nmpc.set( 'CG_USE_VARIABLE_WEIGHTING_MATRIX', 'YES' );
nmpc.set( 'GENERATE_MAKE_FILE', 'NO' );
nmpc.set( 'GENERATE_TEST_FILE', 'NO' );
nmpc.set( 'GENERATE_SIMULINK_INTERFACE', 'NO' );

% export -- this generates the solver in c code and puts it in the export
%           folder "export_nmpc". a mex is also generated for simulation.
copyfile('../acado/external_packages/qpoases', ...
    'export_nmpc/qpoases')
nmpc.exportCode( 'export_nmpc' );

% this actually compiles the code for the mex solver
cd export_nmpc
make_acado_solver('../acado_nmpc_step')
cd ..
