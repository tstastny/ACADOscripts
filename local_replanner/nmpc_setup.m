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
N = 50;     % horizon

% STATES - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
DifferentialState pos_n;        % position north [m]
DifferentialState pos_e;    	% position east [m]
DifferentialState pos_d;    	% position down [m]
DifferentialState airsp;        % airspeed [m/s]
DifferentialState fpa;          % flight path angle [rad]
DifferentialState heading;      % heading angle [rad]
DifferentialState roll;         % roll angle [rad]
DifferentialState pitch;        % pitch angle [rad]
DifferentialState prop_speed;   % propeller speed [rps]

% CONTROLS - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Control throt;      % throttle input [rad]
Control roll_ref;   % roll angle reference [rad]
Control pitch_ref;  % pitch angle reference [rad]

% ONLINE DATA - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

% environment
OnlineData air_density;     % air density [kg/m^3]

% disturbances
OnlineData wind_n;          % wind north [m/s]
OnlineData wind_e;          % wind east [m/s]
OnlineData wind_d;          % wind down [m/s]

% control augmented attitude time constants and gains
OnlineData tau_roll;
OnlineData tau_pitch;
OnlineData k_roll;
OnlineData k_pitch;

% throttle parameters
OnlineData tau_prop;

% flap setting
OnlineData flaps;

% heading objective reference
OnlineData heading_ref;

% soft airspeed constraint
OnlineData soft_airsp;
OnlineData jac_soft_airsp;

% soft angle of attack (AoA) constraint
OnlineData soft_aoa;
OnlineData jac_soft_aoa(2);

% soft height above ground level (HAGL) constraint
OnlineData soft_hagl;
OnlineData jac_soft_hagl(4);

% soft radial terrain distance (RTD) constraint
OnlineData soft_rtd;
OnlineData jac_soft_rtd(6);

% MODEL -------------------------------------------------------------------

% environmental parameters
g = 9.81;       % acceleration of gravity [m/s2]

% model parameters

% salient characteristics
mass = sysid_config.mass;   % mass [kg]

% aerodynamic and thrusting parameters
for i = 1:length(model_params)
    eval([model_params(i).Name,' = ',num2str(model_params(i).Value),';']);
end

% prop advance ratio scaling
vmin = 10;                                                                  % minimum stabilizable airspeed on trim map
vmax = 25;                                                                  % maximum stabilizable airspeed on trim map
n_T0_vmin = (-cT_1*vmin/sysid_config.d_prop + ...                           % zero-thrust prop speed at minimum airspeed
    sqrt((cT_1*vmin/sysid_config.d_prop)^2 - cT_0*4))/2/cT_0;
n_T0_vmax = (-cT_1*vmax/sysid_config.d_prop + ...                           % zero-thrust prop speed at maximum airspeed
    sqrt((cT_1*vmax/sysid_config.d_prop)^2 - cT_0*4))/2/cT_0;

% static modeling
qbarS = 1/2 * air_density * airsp^2 * sysid_config.S_w;                    	% dynamic pressure
aoa = pitch - fpa;                                                          % angle of attack
L = qbarS * (cL_0 + cL_aoa*aoa + 2.0*cL_delta_F*flaps);                     % lift
D = qbarS * (cD_0 + cD_aoa*aoa + cD_aoa2*aoa^2 + ...
    2.0*(cD_delta_F*flaps + cD_delta_F2*flaps^2));                          % drag
vp = airsp * cos(aoa - sysid_config.epsilon_T);                             % inflow at propeller
sig_vp = (vp - vmin)/(vmax - vmin);                                         % prop inflow linear interpolater
u_n = ...                                                                   % prop speed input (converted from throttle input considering inflow and zero thrusting conditions)
    (n_T0_vmin + throt * (sysid_config.rpm_max/60 - n_T0_vmin)) * (1 - sig_vp) + ...          
    (n_T0_vmax + throt * (sysid_config.rpm_max/60 - n_T0_vmax)) * sig_vp;                       
T = air_density * prop_speed^2 * sysid_config.d_prop^4 * ...                       % thrust (assuming prop_peed min > 0! -- should not be possible if control constraints are respected)
    (cT_0 + cT_1 * vp / prop_speed / sysid_config.d_prop); 

% state differentials
dot_pos_n = airsp * cos(fpa) * cos(heading) + wind_n;
dot_pos_e = airsp * cos(fpa) * sin(heading) + wind_e;
dot_pos_d = -airsp * sin(fpa) + wind_d;
dot_airsp = 1/mass * (T * cos(aoa) - D) - g * sin(fpa);
dot_fpa = 1/mass/airsp * ( (T * sin(aoa) + L) * cos(roll) - mass * g * cos(fpa) ); 
dot_heading = sin(roll)/mass/airsp/cos(fpa) * (T * sin(aoa) + L);
dot_roll = (k_roll * roll_ref - roll) / tau_roll;
dot_pitch = (k_pitch * pitch_ref - pitch) / tau_pitch;
dot_prop_speed = (u_n - prop_speed) / tau_prop;

% ode
f = acado.DifferentialEquation;
f.add(dot(pos_n) == dot_pos_n);
f.add(dot(pos_e) == dot_pos_e);
f.add(dot(pos_d) == dot_pos_d);
f.add(dot(airsp) == dot_airsp);
f.add(dot(fpa) == dot_fpa);
f.add(dot(heading) == dot_heading);
f.add(dot(roll) == dot_roll);
f.add(dot(pitch) == dot_pitch);
f.add(dot(prop_speed) == dot_prop_speed);


% OPTIMAL CONTROL PROBLEM -------------------------------------------------

% lengths
n_X = length(diffStates);   % states
n_U = length(controls);     % controls
n_Ys = 9;                   % state (only) objectives
n_Yc = 3;                   % control dependent objectives
n_OD = 28;                  % online data

acadoSet('problemname', 'nmpc');

% optimal control problem
ocp = acado.OCP( 0.0, N*Ts, N );

% external objective function
Q = eye(n_Ys+n_Yc,n_Ys+n_Yc);
Q = acado.BMatrix(Q);
QN = eye(n_Ys,n_Ys);
QN = acado.BMatrix(QN);

ocp.minimizeLSQ( Q, 'evaluateLSQ' );
ocp.minimizeLSQEndTerm( QN, 'evaluateLSQEndTerm' );
% ocp.setModel('model', 'rhs', 'rhs_jac');
% ocp.setDimensions( n_X, n_U, n_OD, 0 );

% hard coded constraints
roll_lim = deg2rad(35);
pitch_lim_pos = deg2rad(25);
pitch_lim_neg = deg2rad(-15);

ocp.subjectTo( f );

% NOTE: in the case of non-hardcoded constraints, these values are only
%       used for initializing the solver
ocp.subjectTo( 0.0 <= throt <= 1.0 );
ocp.subjectTo( -roll_lim <= roll_ref <= roll_lim );
ocp.subjectTo( pitch_lim_neg <= pitch_ref <= pitch_lim_pos );

setNOD(ocp, n_OD);

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
nmpc.set( 'CG_HARDCODE_CONSTRAINT_VALUES', 'NO' );
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
