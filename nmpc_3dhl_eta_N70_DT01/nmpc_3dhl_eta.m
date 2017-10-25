% -------------------------------------------------------------------------
% Nonlinear Model Predictive Tracking Control Setup
% -------------------------------------------------------------------------
clc;
clear all;
close all;

% BEGIN_ACADO;

Ts = 0.1;	% model discretization step
N  = 40;    % horizon length

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
DifferentialState sw;       % (segment switching state)

% CONTROLS - - - - - - -
Control u_T;                % (throttle input)              [~]
Control phi_ref;            % (roll angle reference)        [rad]
Control theta_ref;          % (pitch angle reference)       [rad]

% ONLINE DATA - - - - - -
OnlineData pparam1;         % 
OnlineData pparam2;         % 
OnlineData pparam3;         % 
OnlineData pparam4;         % 
OnlineData pparam5;         % 
OnlineData pparam6;         % 
OnlineData pparam7;         % 

OnlineData pparam1_next;   	% 
OnlineData pparam2_next;   	% 
OnlineData pparam3_next;   	% 
OnlineData pparam4_next;  	% 
OnlineData pparam5_next;  	% 
OnlineData pparam6_next;  	% 
OnlineData pparam7_next;   	% 

OnlineData R_acpt;          % (acceptance radius)   [m]
OnlineData ceta_acpt;       % (cosine of acceptance angle)

OnlineData wn;              % (northing wind)       [m/s]
OnlineData we;              % (easting wind)        [m/s]
OnlineData wd;              % (easting wind)        [m/s]

OnlineData alpha_p_co;      % angle of attack upper cutoff
OnlineData alpha_m_co;      % angle of attack lower cutoff 
OnlineData alpha_delta_co; 	% angle of attack cutoff transition length

OnlineData T_b_lat;         % lateral-directional track-error boundary constant
OnlineData T_b_lon;         % longitudinal track-error boundary

OnlineData ddot_clmb;       % max climb rate
OnlineData ddot_sink;       % max sink rate

% OPTIMAL CONTROL PROBLEM -------------------------------------------------

% lengths
n_X = length(diffStates);   % states
n_U = length(controls);     % controls
n_Y = 7;                    % state objectives
n_Z = 4;                    % control dependent objectives
n_OD = 26;                  % onlinedata

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
ocp.setModel('model', 'rhs', 'rhs_jac');
ocp.setDimensions( n_X, n_U, n_OD, 0 );

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
    'export_nmpc_3dhl_eta/qpoases')
nmpc.exportCode( 'export_nmpc_3dhl_eta' );

cd export_nmpc_3dhl_eta
make_acado_solver('../acado_nmpc_3dhl_eta_step', 'model.c')
cd ..

