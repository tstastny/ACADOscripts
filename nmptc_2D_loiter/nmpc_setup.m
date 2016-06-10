% -------------------------------------------------------------------------
% Nonlinear Model Predictive Tracking Control Setup
% -------------------------------------------------------------------------
clc;
clear all;
close all;

Ts = 0.1;	% model discretization step
N  = 80;    % horizon length

% states
DifferentialState n;            % (northing)                    [m]
DifferentialState e;            % (easting)                     [m]
DifferentialState mu;           % (bank angle)                  [rad]
DifferentialState xi;           % (heading angle)               [rad]
DifferentialState mu_dot;       % (bank angle rate)             [rad/s]

% controls
Control mu_r;                   % (reference bank angle)        [rad]

% online data
OnlineData V;                   % (airspeed)                    [m/s]
OnlineData c_n;                 % (easting loiter center)       [m]        
OnlineData c_e;                 % (northing loiter center)      [m]
OnlineData R;                   % (loiter radius)               [m]
OnlineData ldir;                % (loiter direction)            [~]
OnlineData w_n;                 % (northing wind)               [m/s]
OnlineData w_e;                 % (easting wind)                [m/s]

% MODEL -------------------------------------------------------------------

% gravity
g = 9.81;

% second order bank coefficients
a0 = 13.97;
a1 = 6.577;
b0 = 13.48;

% state differentials
n_dot = V * cos(xi) + w_n;
e_dot = V * sin(xi) + w_e;
xi_dot = g * tan(mu) / V;
% mu_dot
mu_dot_dot = b0 * mu_r - a1 * mu_dot - a0 * mu;

% f(x,u)
f = acado.DifferentialEquation;
f.add(dot(n) == n_dot);
f.add(dot(e) == e_dot);
f.add(dot(mu) == mu_dot);
f.add(dot(xi) == xi_dot);
f.add(dot(mu_dot) == mu_dot_dot);

% OBJECTIVE ---------------------------------------------------------------

epsilon = 0.001;

cp_n = n - c_n;
cp_e = e - c_e;
normcp = sqrt(cp_n * cp_n + cp_e * cp_e);
cp_n_unit = cp_n / (normcp + epsilon);
cp_e_unit = cp_e / (normcp + epsilon);
d_n = R * cp_n_unit + c_n;
d_e = R * cp_e_unit + c_e;
Td_n = -cp_e_unit * ldir;
Td_e = cp_n_unit * ldir;
Vg = sqrt(n_dot * n_dot + e_dot * e_dot);

% track error
e_t = (d_n - n) * Td_e - (d_e - e) * Td_n;

% velocity error
e_vn = Td_n - n_dot / (Vg + epsilon);
e_ve = Td_e - e_dot / (Vg + epsilon);

y = [e_t; e_vn; e_ve; mu; mu_dot];
z = [mu_r; mu_r]; % NOTE: one is for regulation, one is for reference tracking

% OPTIMAL CONTROL PROBLEM -------------------------------------------------

% lengths
n_X = length(diffStates);   % states
n_U = length(controls);     % controls
n_Y = length(y);            % state output objectives
n_Z = length(z);            % control dependent objectives
n_OD = 7;                  % onlinedata

Q = eye(n_Y+n_Z,n_Y+n_Z);
Q = acado.BMatrix(Q);

QN = eye(n_Y,n_Y);
QN = acado.BMatrix(QN);

acadoSet('problemname', 'nmpc');

% optimal control problem
ocp = acado.OCP( 0.0, N*Ts, N );

% minimization
ocp.minimizeLSQ( Q, [y; z] );
ocp.minimizeLSQEndTerm( QN, y );

% constraints
ocp.subjectTo( f );
ocp.subjectTo( -35*pi/180 <= mu_r <= 35*pi/180 );

setNOD(ocp, n_OD);

% export settings
nmpc = acado.OCPexport( ocp );
nmpc.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON' );
nmpc.set( 'DISCRETIZATION_TYPE', 'MULTIPLE_SHOOTING' );
nmpc.set( 'SPARSE_QP_SOLUTION', 'FULL_CONDENSING' );
nmpc.set( 'INTEGRATOR_TYPE', 'INT_IRK_GL4' );
nmpc.set( 'NUM_INTEGRATOR_STEPS', N );
nmpc.set( 'QP_SOLVER', 'QP_QPOASES' );
nmpc.set( 'HOTSTART_QP', 'YES' );
nmpc.set( 'LEVENBERG_MARQUARDT', 1e-10 );
nmpc.set( 'GENERATE_MAKE_FILE', 'YES' );
nmpc.set( 'GENERATE_TEST_FILE', 'YES' );
nmpc.set( 'GENERATE_SIMULINK_INTERFACE', 'YES' );
nmpc.set( 'CG_HARDCODE_CONSTRAINT_VALUES', 'YES' );
nmpc.set( 'CG_USE_VARIABLE_WEIGHTING_MATRIX', 'YES' );

% export
copyfile('../acado/external_packages/qpoases', ...
    'export_nmpc/qpoases')
nmpc.exportCode( 'export_nmpc' );

cd export_nmpc
make_acado_solver('../acado_nmpc_step')
cd ..

