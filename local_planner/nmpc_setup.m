% -------------------------------------------------------------------------
% Fixed-wing Local Planning w/ Obstacle Avoidance
% -------------------------------------------------------------------------
clc;
clear all;
close all;

% load model config
load('model_config/model_params.mat');
run('model_config/sysid_config_Techpod.m');

Ts = 0.1;   % model discretization step [0.1 s]
N = 70;     % horizon

% STATES - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
DifferentialState r_n;      % northing [m]
DifferentialState r_e;    	% easting [m]
DifferentialState r_d;    	% downing [m]
DifferentialState v;        % airspeed [m/s]
DifferentialState gamma;    % flight path angle [rad]
DifferentialState xi;       % heading angle [rad]
DifferentialState phi;      % bank angle [rad]
DifferentialState theta;    % pitch angle [rad]
DifferentialState n_p;      % propeller speed [rps]

% CONTROLS - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Control u_T;                % throttle input [rad]
Control phi_ref;            % bank angle reference [rad]
Control theta_ref;          % pitch angle reference [rad]

% ONLINE DATA - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

% disturbances
OnlineData w_n;
OnlineData w_e;
OnlineData w_d;

% path reference
OnlineData b_n;
OnlineData b_e;
OnlineData b_d;
OnlineData Gamma_p;
OnlineData chi_p;

% guidance
OnlineData T_b_lat;
OnlineData T_b_lon;

% control augmented attitude time constants and gains
OnlineData tau_phi;
OnlineData tau_theta;
OnlineData k_phi;
OnlineData k_theta;

% soft constraints
OnlineData delta_aoa;
OnlineData aoa_m;
OnlineData aoa_p;

% terrain
OnlineData delta_h;
OnlineData terr_local_origin_n;
OnlineData terr_local_origin_e;
OnlineData terr_dis;
LEN_TERR_DATA = 3249;%19881;
OnlineData terrain_data(LEN_TERR_DATA);

% MODEL -------------------------------------------------------------------

% environmental parameters
g = 9.81;       % acceleration of gravity [m/s2]
rho = 1.15;     % air density [kg/m^3]

% model parameters

% salient characteristics
mass = sysid_config.mass;   % mass [kg]

% aerodynamic and thrusting parameters
for i = 1:length(model_params)
    eval([model_params(i).Name,' = ',num2str(model_params(i).Value),';']);
end

% prop speed time constant
tau_n = 0.2;

% prop advance ratio scaling
vmin = 10;                                                                  % minimum stabilizable airspeed on trim map
vmax = 25;                                                                  % maximum stabilizable airspeed on trim map
n_T0_vmin = (-cT_1*vmin/sysid_config.d_prop + ...                           % zero-thrust prop speed at minimum airspeed
    sqrt((cT_1*vmin/sysid_config.d_prop)^2 - cT_0*4))/2/cT_0;
n_T0_vmax = (-cT_1*vmax/sysid_config.d_prop + ...                           % zero-thrust prop speed at maximum airspeed
    sqrt((cT_1*vmax/sysid_config.d_prop)^2 - cT_0*4))/2/cT_0;

% static modeling
qbarS = 1/2 * rho * v^2 * sysid_config.S_w;                                 % dynamic pressure
aoa = theta - gamma;                                                        % angle of attack
L = qbarS * (cL_0 + cL_aoa*aoa);                                            % lift
D = qbarS * (cD_0 + cD_aoa*aoa + cD_aoa2*aoa^2);                            % drag
vp = v * cos(aoa - sysid_config.epsilon_T);                                 % inflow at propeller
sig_vp = (vp - vmin)/(vmax - vmin);                                         % prop inflow linear interpolater
u_n = ...                                                                   % prop speed input (converted from throttle input considering inflow and zero thrusting conditions)
    (n_T0_vmin + u_T * (sysid_config.rpm_max/60 - n_T0_vmin)) * (1 - sig_vp) + ...          
    (n_T0_vmax + u_T * (sysid_config.rpm_max/60 - n_T0_vmax)) * sig_vp;                       
T = rho * n_p^2 * sysid_config.d_prop^4 * ...                               % thrust (assuming n_p min > 0! -- should not be possible if control constraints are respected)
    (cT_0 + cT_1 * vp / n_p / sysid_config.d_prop); 

% state differentials
dot_r_n = v * cos(gamma) * cos(xi) + w_n;
dot_r_e = v * cos(gamma) * sin(xi) + w_e;
dot_r_d = -v * sin(gamma) + w_d;
dot_v = 1/mass * (T * cos(aoa) - D) - g * sin(gamma);
dot_gamma = 1/mass/v * ( (T * sin(aoa) + L) * cos(phi) - mass * g * cos(gamma) ); 
dot_xi = sin(phi)/mass/v/cos(gamma) * (T * sin(aoa) + L);
dot_phi = (k_phi * phi_ref - phi) / tau_phi;
dot_theta = (k_theta * theta_ref - theta) / tau_theta;
dot_n_p = (u_n - n_p) / tau_n;

% ode
f = acado.DifferentialEquation;
f.add(dot(r_n) == dot_r_n);
f.add(dot(r_e) == dot_r_e);
f.add(dot(r_d) == dot_r_d);
f.add(dot(v) == dot_v);
f.add(dot(gamma) == dot_gamma);
f.add(dot(xi) == dot_xi);
f.add(dot(phi) == dot_phi);
f.add(dot(theta) == dot_theta);
f.add(dot(n_p) == dot_n_p);


% OPTIMAL CONTROL PROBLEM -------------------------------------------------

% lengths
n_X = length(diffStates);   % states
n_U = length(controls);     % controls
n_Y = 9;                    % outputs
n_Z = 3;                    % objectives
n_OD = 21+LEN_TERR_DATA;    % online data

acadoSet('problemname', 'nmpc');

% optimal control problem
ocp = acado.OCP( 0.0, N*Ts, N );

% external objective function
Q = eye(n_Y+n_Z,n_Y+n_Z);
Q = acado.BMatrix(Q);
QN = eye(n_Y,n_Y);
QN = acado.BMatrix(QN);

ocp.minimizeLSQ( Q, 'evaluateLSQ' );
ocp.minimizeLSQEndTerm( QN, 'evaluateLSQEndTerm' );
% ocp.setModel('model', 'rhs', 'rhs_jac');
% ocp.setDimensions( n_X, n_U, n_OD, 0 );

% hard coded constraints
phi_lim = deg2rad(35);
theta_lim_pos = deg2rad(25);
theta_lim_neg = deg2rad(-15);

ocp.subjectTo( f );
ocp.subjectTo( 0.0 <= u_T <= 1.0 );
ocp.subjectTo( -phi_lim <= phi_ref <= phi_lim );
ocp.subjectTo( theta_lim_neg <= theta_ref <= theta_lim_pos );

setNOD(ocp, n_OD);

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
make_acado_solver('../acado_nmpc_step', 'lsq_objective.c')
cd ..
