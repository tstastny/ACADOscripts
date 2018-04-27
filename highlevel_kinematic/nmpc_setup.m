% -------------------------------------------------------------------------
% Fixed-wing Local Re-planning w/ Obstacle Avoidance
% -------------------------------------------------------------------------
clc;
clear all;
close all;

% BEGIN_ACADO;

Ts = 0.1; % model discretization step [0.1 s]
N = 100; % horizon

% STATES - - - - - - - -
DifferentialState n; % northing [m]
DifferentialState e; % easting [m]
DifferentialState d; % downing [m]
DifferentialState gamma; % flight path angle [rad]
DifferentialState xi; % heading angle [rad]
DifferentialState mu; % bank angle [rad]

% CONTROLS - - - - - - -
Control gamma_ref; % flight path angle reference
Control mu_ref; % bank angle reference

% ONLINE DATA - - - - - -

% flight condition
OnlineData v;

% disturbances
OnlineData wn;
OnlineData we;
OnlineData wd;

% path reference
OnlineData lpn;
OnlineData lpe;
OnlineData lpd;
OnlineData lvn;
OnlineData lve;
OnlineData lvd;

% OnlineData ddot_clmb;
% OnlineData ddot_sink;
% OnlineData v_min;
% OnlineData v_max;
% OnlineData mu_lim;

% DIFFERENTIAL EQUATIONS --------------------------------------------------

% model parameters
tau_gamma = 1;
tau_mu = 0.7;

% constraints
ddot_clmb = 4;
ddot_sink = 2;
v_min = 8;
v_max = 13;
mu_lim = deg2rad(45);

% constants
g = 9.81; % acceleration of gravity [m/s2]

% ode
f = acado.DifferentialEquation;
f.add(dot(n) == v*cos(gamma)*cos(xi) + wn);
f.add(dot(e) == v*cos(gamma)*sin(xi) + we);
f.add(dot(d) == -v*sin(gamma));
f.add(dot(gamma) == (gamma_ref - gamma)/tau_gamma);
f.add(dot(xi) == g*tan(mu)/v/cos(gamma));
f.add(dot(mu) == (mu_ref - mu)/tau_mu);

% OPTIMAL CONTROL PROBLEM -------------------------------------------------

% objectives
e_lat = lvn * (lpe-e) - (lpn-n) * lve;
e_lon = lvd * (lpd-d);

% state output
y = [ e_lat; e_lon];

% control output
z = [ gamma_ref; mu_ref ];

% lengths
n_X = length(diffStates);   % states
n_U = length(controls);     % controls
n_Y = length(y);            % outputs
n_Z = length(z);            % objectives

Q = eye(n_Y+n_Z,n_Y+n_Z);
Q = acado.BMatrix(Q);

QN = eye(n_Y,n_Y);
QN = acado.BMatrix(QN);

% objective
h = [ y; z ];

acadoSet('problemname', 'nmpc');

% optimal control problem
ocp = acado.OCP( 0.0, N*Ts, N );

% minimization
ocp.minimizeLSQ( Q, h );
ocp.minimizeLSQEndTerm( QN, y );

ocp.subjectTo( f );
ocp.subjectTo( -ddot_clmb <= v * sin(gamma_ref) <= ddot_sink );
ocp.subjectTo( -mu_lim <= mu_ref <= mu_lim );

setNOD(ocp, 10);

% export settings
nmpc = acado.OCPexport( ocp );
nmpc.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON' );
nmpc.set( 'DISCRETIZATION_TYPE', 'MULTIPLE_SHOOTING' );
nmpc.set( 'SPARSE_QP_SOLUTION', 'FULL_CONDENSING' );
nmpc.set( 'INTEGRATOR_TYPE', 'INT_IRK_GL4' );
nmpc.set( 'NUM_INTEGRATOR_STEPS', N  );
nmpc.set( 'QP_SOLVER', 'QP_QPOASES' );
nmpc.set( 'HOTSTART_QP', 'YES' );
nmpc.set( 'LEVENBERG_MARQUARDT', 1e-10 );
nmpc.set( 'CG_HARDCODE_CONSTRAINT_VALUES', 'YES' );
nmpc.set( 'CG_USE_VARIABLE_WEIGHTING_MATRIX', 'YES' );
nmpc.set( 'GENERATE_MAKE_FILE', 'NO' );
nmpc.set( 'GENERATE_TEST_FILE', 'NO' );
nmpc.set( 'GENERATE_SIMULINK_INTERFACE', 'NO' );

% export
copyfile('../acado/external_packages/qpoases', ...
    'export_nmpc/qpoases')
nmpc.exportCode( 'export_nmpc' );

cd export_nmpc
make_acado_solver('../acado_nmpc_step')
cd ..
