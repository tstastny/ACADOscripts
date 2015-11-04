clc;
clear all;
close all;

% BEGIN_ACADO;

Ts = 0.1;

% STATES - - - - - - - -
DifferentialState x;

% CONTROLS - - - - - - -
Control u;

% OUTPUTS - - - - - - -
y = x;

% lengths
N   = 10;                   % horizon
n_X = length(diffStates);   % states
n_U = length(controls);     % controls
n_Y = length(y);            % outputs

% differentials
f = acado.DifferentialEquation;
f.add(dot(x) == u);

% weights
Q = eye(n_Y+n_U,n_Y+n_U);
Q = acado.BMatrix(Q);

QN = eye(n_Y,n_Y);
QN = acado.BMatrix(QN);

% objective
h = [ y; u ];

acadoSet('problemname', 'nmpc');

% optimal control problem
ocp = acado.OCP( 0.0, N*Ts, N );

% minimization
ocp.minimizeLSQ( Q, h );
ocp.minimizeLSQEndTerm( QN, y );

% constraints
ocp.subjectTo( f );
ocp.subjectTo( -3 <= u <= 3 );

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
    'export_nmpc/qpoases')
nmpc.exportCode( 'export_nmpc' );

cd export_nmpc
make_acado_solver('../acado_nmpc_step')
cd ..

