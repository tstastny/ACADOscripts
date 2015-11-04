% -------------------------------------------------------------------------
% Techpod Model Setup
% -------------------------------------------------------------------------
clc;
clear all;
close all;

% BEGIN_ACADO;

Ts = 0.05;	% model discretization step [50 ms]

% STATES - - - - - - - -
DifferentialState u;            %	(x-body velocity)           [m/s]
DifferentialState v;            %	(y-body velocity)           [rad]
DifferentialState w;            %   (z-body velocity)           [rad]
DifferentialState p;            %   (roll rate)                 [rad/s]
DifferentialState q;            %   (pitch rate)                [rad/s]   
DifferentialState r;            %   (yaw rate)                  [rad/s]   
DifferentialState phi;          %   (roll angle)                [rad] 
DifferentialState theta;        %   (pitch angle)               [rad]  

DifferentialState dummy;        %   (dummy slack state)         [~]

DifferentialState intg_V;       %   (integral of airspeed)      [m]

DifferentialState x_w2_uT;      %   (throtte w2)                [~]
DifferentialState x_w2_uE;      %   (elevator w2)               [~]
DifferentialState x_w2_uA;      %   (aileron w2)                [~]
DifferentialState x_w2_uR;      %   (rudder w2)                 [~]
DifferentialState x_w2_sv;      %   (rudder w2)                 [~]

% CONTROLS - - - - - - -
Control uT;     %   (throttle setting)      [0,1]
Control uE;     %   (elevator deflection)   [rad]
Control uA;     %   (aileron deflection)	[rad]
Control uR;     %   (rudder deflection)     [rad]

Control sv;     %   (slack variable)        [~]

% ONLINE DATA - - - - - -
OnlineData Aw2;
OnlineData Bw2;
OnlineData Cw2;
OnlineData Dw2;

OnlineData kiV;

% intermediate states
V       = sqrt(u^2+v^2+w^2);
beta    = asin(v/V);
alpha   = atan(w/u);

% state output
y = [ V + intg_V; theta; phi; beta; p; q; r ];

% % MIXED SENSITIVITY LOOP SHAPING ------------------------------------------
z_w2_uT = Cw2*x_w2_uT + Dw2*uT;
z_w2_uE = Cw2*x_w2_uE + Dw2*uE;
z_w2_uA = Cw2*x_w2_uA + Dw2*uA;
z_w2_uR = Cw2*x_w2_uR + Dw2*uR;
z_w2_sv = Cw2*x_w2_sv + Dw2*sv;

% OPTIMAL CONTROL PROBLEM -------------------------------------------------

% control output
z = [ z_w2_uT; z_w2_uE; z_w2_uA; z_w2_uR; z_w2_sv ];

% lengths
N   = 10;                   % horizon
n_X = length(diffStates);   % states
n_U = length(controls);     % controls
n_Y = length(y);            % outputs
n_Z = length(z);            % objectives
n_OD = 5;

Q = eye(n_Y+n_Z,n_Y+n_Z);
Q = acado.BMatrix(Q);

QN = eye(n_Y,n_Y);
QN = acado.BMatrix(QN);

% objective
h = [ y; z ];

acadoSet('problemname', 'nmpc_ext');

% optimal control problem
ocp = acado.OCP( 0.0, N*Ts, N );

% minimization
ocp.minimizeLSQ( Q, h );
ocp.minimizeLSQEndTerm( QN, y );

% external model
ocp.setModel('model', 'rhs', 'rhs_jac');
ocp.setDimensions( n_X, n_U, n_OD, 0 );

ocp.subjectTo( 0.1 <= uT <= 1 );
ocp.subjectTo( -20*pi/180 <= uE <= 20*pi/180 );
ocp.subjectTo( -20*pi/180 <= uA <= 20*pi/180 );
ocp.subjectTo( -20*pi/180 <= uR <= 20*pi/180 );
ocp.subjectTo( -60*pi/180 <= phi <= 60*pi/180 );
ocp.subjectTo( -45*pi/180 <= theta <= 45*pi/180 );
ocp.subjectTo( -2*pi/180 <= alpha + sv );
ocp.subjectTo( alpha - sv <= 10*pi/180 );
ocp.subjectTo( 0 <= sv <= 3*pi/180 );

setNOD(ocp, n_OD);

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
copyfile('/home/thomas/ACADOtoolkit/external_packages/qpoases', ...
    '/home/thomas/ACADOtoolkit/interfaces/matlab/techpod_6DoF_nmpc_w2_P_ext/export_nmpc_ext/qpoases')
nmpc.exportCode( 'export_nmpc_ext' );

cd export_nmpc_ext
make_acado_solver('../acado_nmpc_ext_step')
cd ..

