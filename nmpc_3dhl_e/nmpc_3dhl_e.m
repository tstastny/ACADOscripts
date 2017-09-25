% -------------------------------------------------------------------------
% Nonlinear Model Predictive Tracking Control Setup
% -------------------------------------------------------------------------
clc;
clear all;
close all;

% BEGIN_ACADO;

Ts = 0.1;	% model discretization step
N  = 80;    % horizon length

% STATES - - - - - - - -
DifferentialState n;        % (northing)
DifferentialState e;       	% (easting)
DifferentialState d;        % (down)
DifferentialState V;        % (airspeed)
DifferentialState gamma;    % (flight path angle)
DifferentialState xi;     	% (heading angle)
DifferentialState phi;     	% (roll angle)
DifferentialState theta;	% (pitch angle)
DifferentialState p;        % (roll rate)
DifferentialState q;        % (pitch rate)
DifferentialState r;        % (yaw rate)
DifferentialState delta_T;  % (throttle setting)

% CONTROLS - - - - - - -
Control u_T;                % (throttle input)              [~]
Control phi_ref;            % (roll angle reference)        [rad]
Control theta_ref;          % (pitch angle reference)       [rad]

% ONLINE DATA - - - - - -
OnlineData wn;              % (northing wind)       [m/s]
OnlineData we;              % (easting wind)        [m/s]
OnlineData wd;              % (easting wind)        [m/s]

OnlineData alpha_p_co;      % angle of attack upper cutoff
OnlineData alpha_m_co;      % angle of attack lower cutoff 
OnlineData alpha_delta_co; 	% angle of attack cutoff transition length

OnlineData p_n;
OnlineData p_e;
OnlineData p_d;

% DIFFERENTIAL EQUATIONS --------------------------------------------------

alpha = theta - gamma; % assume no beta
mu = phi; % assume no beta

% position differentials

n_dot = V * cos(xi) * cos(gamma) + wn;
e_dot = V * sin(xi) * cos(gamma) + we;
d_dot = -V * sin(gamma) + wd;

% velocity frame differentials

g = 9.81;
m = 2.65;
rho = 1.225;
S = 0.39;
qbarS = 0.5*rho*V^2*S;

load parameters_20161209.mat;
for i = 1:length(parameters)
    eval([parameters(i).Name,'=',num2str(parameters(i).Value),';']);
end

cD = cD0 + cDa * alpha + cDa2 * alpha^2;
cL = cL0 + cLa * alpha + cLa2 * alpha^2;

T = (cT1*delta_T+cT2*delta_T^2+cT3*delta_T^3)/V/cos(alpha); % should probably have a check for alpha=90.. but really shouldnt happen
D = qbarS*cD;
L = qbarS*cL;

V_dot = (T*cos(alpha)-D)/m-g*sin(gamma);
gamma_dot = ((T*sin(alpha)+L)*cos(mu) - m*g*cos(gamma))/m/V;
xi_dot = (T*sin(alpha)+L)*sin(mu)/V/m/cos(gamma); % same as with alpha

% attitude differentials
phi_dot = p;
theta_dot = q*cos(phi)-r*sin(phi);

% body rate differentials

Lm = Lp * p + Lr * r + LeR * (phi_ref-phi);
Mm = V^2 * (M0 + Ma * alpha + Mq * q + MeP * (theta_ref-theta));
Nm = Nr * r + NR * phi + NRR * phi_ref;

p_dot = Lm;
q_dot = Mm;
r_dot = Nm;

% throttle delay

delta_T_dot = (u_T - delta_T) / tauT;

% ode
f = acado.DifferentialEquation;
f.add(dot(n) == n_dot);
f.add(dot(e) == e_dot);
f.add(dot(d) == d_dot);
f.add(dot(V) == V_dot);
f.add(dot(gamma) == gamma_dot);
f.add(dot(xi) == xi_dot);
f.add(dot(phi) == phi_dot);
f.add(dot(theta) == theta_dot);
f.add(dot(p) == p_dot);
f.add(dot(q) == q_dot);
f.add(dot(r) == r_dot);
f.add(dot(delta_T) == delta_T_dot);

% OPTIMAL CONTROL PROBLEM -------------------------------------------------

% lengths
n_X = length(diffStates);   % states
n_U = length(controls);     % controls
n_Y = 7;                    % state objectives
n_Z = 4;                    % control dependent objectives
n_OD = 9;                  % onlinedata

Q = eye(n_Y+n_Z,n_Y+n_Z);
Q = acado.BMatrix(Q);

QN = eye(n_Y,n_Y);
QN = acado.BMatrix(QN);

acadoSet('problemname', 'nmpc_ext');

% optimal control problem
ocp = acado.OCP( 0.0, N*Ts, N );

% minimization
ocp.minimizeLSQ( Q, 'evaluateLSQ' );
ocp.minimizeLSQEndTerm( QN, 'evaluateLSQEndTerm' );

% external model
% ocp.setModel('model', 'rhs', 'rhs_jac');
ocp.setDimensions( n_X, n_U, n_OD, 0 );

ocp.subjectTo( f );
ocp.subjectTo( 0 <= u_T <= 1.0 );
ocp.subjectTo( -30*pi/180 <= phi_ref <= 30*pi/180 );
ocp.subjectTo( -15*pi/180 <= theta_ref <= 15*pi/180 );

setNOD(ocp, n_OD);

% export settings
nmpc = acado.OCPexport( ocp );
nmpc.set( 'HESSIAN_APPROXIMATION',          'GAUSS_NEWTON'          );
nmpc.set( 'DISCRETIZATION_TYPE',            'MULTIPLE_SHOOTING'     );
nmpc.set( 'SPARSE_QP_SOLUTION',             'FULL_CONDENSING'    );
nmpc.set( 'INTEGRATOR_TYPE',                'INT_IRK_GL4'           );
nmpc.set( 'NUM_INTEGRATOR_STEPS',           N                       );
nmpc.set( 'QP_SOLVER',                      'QP_QPOASES'            );
nmpc.set( 'HOTSTART_QP',                    'YES'                    );
nmpc.set( 'LEVENBERG_MARQUARDT',            1e-10                   );
% nmpc.set( 'MAX_NUM_QP_ITERATIONS',          20                       );
nmpc.set( 'GENERATE_MAKE_FILE',             'YES'                   );
nmpc.set( 'GENERATE_TEST_FILE',             'YES'                   );
nmpc.set( 'GENERATE_SIMULINK_INTERFACE',    'YES'                   );
nmpc.set( 'CG_HARDCODE_CONSTRAINT_VALUES',  'YES'                    );
nmpc.set( 'CG_USE_VARIABLE_WEIGHTING_MATRIX',  'YES'                    );
% nmpc.set( 'USE_SINGLE_PRECISION',              'YES'                 );

% export
copyfile('../acado/external_packages/qpoases', ...
    'export_nmpc_3dhl_e/qpoases')
nmpc.exportCode( 'export_nmpc_3dhl_e' );

cd export_nmpc_3dhl_e
make_acado_solver('../acado_nmpc_3dhl_e_step', 'model.c')
cd ..

