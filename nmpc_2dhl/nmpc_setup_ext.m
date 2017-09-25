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
DifferentialState n;            % (northing)
DifferentialState e;            % (easting)
DifferentialState mu;           % (bank angle)
DifferentialState xi;           % (heading angle)
DifferentialState mu_dot;       % (bank angle rate)
DifferentialState intg_e_t;     % (integral of track error)
DifferentialState intg_e_chi;   % (integral of course error)
DifferentialState sw;

% CONTROLS - - - - - - -
Control mu_r;         % (reference bank angle)

% ONLINE DATA - - - - - -
OnlineData V;           % (airspeed)            [m/s]

OnlineData pparam1;     %   type    type
OnlineData pparam2;     %   aa_n    cc_n
OnlineData pparam3;     %   aa_e    cc_e
OnlineData pparam4;     %   aa_d    cc_d
OnlineData pparam5;     %   bb_n    R
OnlineData pparam6;     %   bb_e    dir
OnlineData pparam7;     %   bb_d    gam
OnlineData pparam8;     %   --      xi0
OnlineData pparam9;     %   --      dxi

OnlineData pparam1_next;     %   type    type
OnlineData pparam2_next;     %   aa_n    cc_n
OnlineData pparam3_next;     %   aa_e    cc_e
OnlineData pparam4_next;     %   aa_d    cc_d
OnlineData pparam5_next;     %   bb_n    R
OnlineData pparam6_next;     %   bb_e    dir
OnlineData pparam7_next;     %   bb_d    gam
OnlineData pparam8_next;     %   --      xi0
OnlineData pparam9_next;     %   --      dxi

OnlineData wn;          % (northing wind)       [m/s]
OnlineData we;          % (easting wind)        [m/s]

OnlineData k_chi;

OnlineData mu_r_prev;

% OPTIMAL CONTROL PROBLEM -------------------------------------------------

% lengths
n_X = length(diffStates);   % states
n_U = length(controls);     % controls
n_Y = 8;                    % outputs
n_Z = 0;                    % objectives
n_OD = 23;                  % onlinedata

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

ocp.subjectTo( -35*pi/180 <= mu_r <= 35*pi/180 );

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
    'export_nmpc_ext/qpoases')
nmpc.exportCode( 'export_nmpc_ext' );

cd export_nmpc_ext
make_acado_solver('../acado_nmpc_ext_step', 'model.c')
cd ..

