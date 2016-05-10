% -------------------------------------------------------------------------
% Nonlinear Model Predictive Tracking Control Setup
% -------------------------------------------------------------------------
clc;
clear all;
close all;

% BEGIN_ACADO;

Ts = 0.2;	% model discretization step
N  = 20;    % horizon length

% STATES - - - - - - - -
DifferentialState n;        % (northing)                [m]
DifferentialState e;        % (easting)                 [m]
DifferentialState d;        % (down)                    [m]
DifferentialState mu;       % (bank angle)              [rad]
DifferentialState gamma;    % (flight path angle, fpa)  [rad]
DifferentialState xi;       % (heading angle)           [rad]

% CONTROLS - - - - - - -
Control mu_dot;            % (commanded bank angle)     [rad]
Control gamma_dot;         % (commanded fpa)            [rad]

% ONLINE DATA - - - - - -
OnlineData V;           % (airspeed)            [m/s]
OnlineData wn;          % (northing wind)       [m/s]
OnlineData we;          % (easting wind)        [m/s]
OnlineData wd;          % (down wind)           [m/s]
OnlineData an;          % (waypoint a)          [m]
OnlineData ae;          % (waypoint a)          [m]
OnlineData ad;          % (waypoint a)          [m]
OnlineData bn;          % (waypoint b)          [m]
OnlineData be;          % (waypoint b)          [m]
OnlineData bd;          % (waypoint b)          [m]

% STATE DIFFERENTIALS -----------------------------------------------------
n_dot       = V * cos(gamma) * cos(xi) + wn;
e_dot       = V * cos(gamma) * sin(xi) + we;
d_dot       = -V * sin(gamma) + wd;
% mu_dot      = (mu_cmd - mu) / tau_mu;
% gamma_dot   = (gamma_cmd - gamma) / tau_gam;
xi_dot      = 9.81 * tan(mu) / V;

% STATE OUTPUT ------------------------------------------------------------

% calculate vector from waypoint a to b
abn = bn - an;
abe = be - ae;
abd = bd - ad;
norm_ab2 = abn*abn + abe*abe + abd*abd;
norm_ab = sqrt(norm_ab2);
abn_unit = abn / norm_ab;
abe_unit = abe / norm_ab;
abd_unit = abd / norm_ab;

% xi/gamma desired
Gamma_d = -asin(abd_unit);
chi_d = atan(abe_unit/abn_unit);

% ground referenced angles
V_g = sqrt(n_dot^2 + e_dot^2 + d_dot^2);
Gamma = -asin(d_dot/V_g);
chi = atan(e_dot/n_dot);

% track position error
pan = an - n;
pae = ae - e;
pad = ad - d;
cx = abe_unit*pad - pae*abd_unit;
cy = -(abn_unit*pad - pan*abd_unit);
cz = abn_unit*pae - pan*abe_unit;
et = sqrt( cx^2 + cy^2 + cz^2 );

% error rate
% et_dot = (ae^2*d*d_dot - ae^2*bd*d_dot - ad*bn^2*d_dot - an^2*bd*d_dot - ae*bd^2*e_dot - ad^2*be*e_dot - ae*bn^2*e_dot - an^2*be*e_dot - ad*be^2*d_dot + an^2*d*d_dot + be^2*d*d_dot + bn^2*d*d_dot + ad^2*e*e_dot + an^2*e*e_dot + bd^2*e*e_dot + bn^2*e*e_dot - an*bd^2*n_dot - ad^2*bn*n_dot - an*be^2*n_dot - ae^2*bn*n_dot + ad^2*n*n_dot + ae^2*n*n_dot + bd^2*n*n_dot + be^2*n*n_dot + ad*ae*be*d_dot + ad*an*bn*d_dot + ad*ae*bd*e_dot + ae*bd*be*d_dot + an*bd*bn*d_dot + ae*an*bn*e_dot + ad*bd*be*e_dot + an*be*bn*e_dot - ad*ae*d*e_dot - ad*ae*d_dot*e - 2*ae*be*d*d_dot - 2*an*bn*d*d_dot + ad*be*d*e_dot + ad*be*d_dot*e + ae*bd*d*e_dot + ae*bd*d_dot*e - 2*ad*bd*e*e_dot - bd*be*d*e_dot - bd*be*d_dot*e - 2*an*bn*e*e_dot + ad*an*bd*n_dot + ae*an*be*n_dot + ad*bd*bn*n_dot + ae*be*bn*n_dot - ad*an*d*n_dot - ad*an*d_dot*n + ad*bn*d*n_dot + ad*bn*d_dot*n + an*bd*d*n_dot + an*bd*d_dot*n - ae*an*e*n_dot - ae*an*e_dot*n - bd*bn*d*n_dot - bd*bn*d_dot*n + ae*bn*e*n_dot + ae*bn*e_dot*n + an*be*e*n_dot + an*be*e_dot*n - be*bn*e*n_dot - be*bn*e_dot*n - 2*ad*bd*n*n_dot - 2*ae*be*n*n_dot)/(((ad - bd)^2 + (ae - be)^2 + (an - bn)^2)*((ad*be - ae*bd + ae*d - ad*e - be*d + bd*e)^2/((ad - bd)^2 + (ae - be)^2 + (an - bn)^2) + (ad*bn - an*bd + an*d - bn*d - ad*n + bd*n)^2/((ad - bd)^2 + (ae - be)^2 + (an - bn)^2) + (ae*bn - an*be + an*e - bn*e - ae*n + be*n)^2/((ad - bd)^2 + (ae - be)^2 + (an - bn)^2))^(1/2));

% OPTIMAL CONTROL PROBLEM -------------------------------------------------

% state output
y = [ et ];%; et_dot ];

% control output
z = [ Gamma_d-Gamma; chi_d-chi; mu_dot; gamma_dot ];

% lengths
n_X = length(diffStates);   % states
n_U = length(controls);     % controls
n_Y = length(y);            % state obj
n_Z = length(z);            % control obj

% ode
f = acado.DifferentialEquation;
f.add(dot(n) == n_dot);
f.add(dot(e) == e_dot);
f.add(dot(d) == d_dot);
f.add(dot(mu) == mu_dot);
f.add(dot(gamma) == gamma_dot);
f.add(dot(xi) == xi_dot);

Q = eye(n_Y+n_Z,n_Y+n_Z);
Q = acado.BMatrix(Q);

QN = eye(n_Y,n_Y);
QN = acado.BMatrix(QN);

% objective
h = [ y; z ];

acadoSet('problemname', 'nmptc');

% optimal control problem
ocp = acado.OCP( 0.0, N*Ts, N );

% minimization
ocp.minimizeLSQ( Q, h );
ocp.minimizeLSQEndTerm( QN, y );

ocp.subjectTo( f );
ocp.subjectTo( -30*pi/180 <= mu <= 30 );
ocp.subjectTo( -15*pi/180 <= gamma <= 15*pi/180 );
% ocp.subjectTo( -100*pi/180 <= mu_dot <= 100*pi/180 );
% ocp.subjectTo( -100*pi/180 <= gamma_dot <= 100*pi/180 );

setNOD(ocp, 10);

% export settings
nmpc = acado.OCPexport( ocp );
nmpc.set( 'HESSIAN_APPROXIMATION',          'GAUSS_NEWTON'          );
nmpc.set( 'DISCRETIZATION_TYPE',            'MULTIPLE_SHOOTING'     );
nmpc.set( 'SPARSE_QP_SOLUTION',             'FULL_CONDENSING'    );
nmpc.set( 'INTEGRATOR_TYPE',                'INT_IRK_GL4'           );
nmpc.set( 'NUM_INTEGRATOR_STEPS',           N                       );
nmpc.set( 'QP_SOLVER',                      'QP_QPOASES'            );
nmpc.set( 'HOTSTART_QP',                    'NO'                    );
nmpc.set( 'LEVENBERG_MARQUARDT',            1e-10                   );
nmpc.set( 'GENERATE_MAKE_FILE',             'YES'                   );
nmpc.set( 'GENERATE_TEST_FILE',             'YES'                   );
nmpc.set( 'GENERATE_SIMULINK_INTERFACE',    'YES'                   );
nmpc.set( 'CG_HARDCODE_CONSTRAINT_VALUES',  'YES'                    );

% export
copyfile('../acado/external_packages/qpoases', ...
    'export_nmptc/qpoases')
nmpc.exportCode( 'export_nmptc' );

cd export_nmptc
make_acado_solver('../acado_nmpc_step')
cd ..
