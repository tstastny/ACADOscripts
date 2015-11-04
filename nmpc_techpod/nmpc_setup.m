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

% MODEL CONSTRUCTION ------------------------------------------------------

% geometry
S_wing          = 0.47;     % wing surface              [m^2]
b_wing          = 2.59;     % wingspan                  [m]
c_chord         = 0.180;    % mean chord length         [m]
i_thrust        = 0*pi/180;	% thrust incidence angle    [rad] -->NOTE: i is defined from body x in positive pitch (up)

% mass / inertia
mass    = 2.65;         % total mass of airplane    [kg]
Ixx     = 0.1512*1.1;	% inertias                  [kg m^2]
Iyy     = 0.2785*1.4;	% inertias                  [kg m^2]
Izz     = 0.3745*1.4;	% inertias                  [kg m^2]
Ixz     = 0.0755;       % inertias                  [kg m^2]

% environment
g       = 9.81;     % gravitational acceleration    [m/s^2]
rho_air = 1.18;     % air density

% wind to body transform
ca = cos(alpha);
sa = sin(alpha);
H_W2B = [ca,	0,	-sa;
         0,     1,	0;
         sa,    0,	ca];

% non-dimensionalization
p_hat	= p*b_wing /(2*V);
q_hat	= q*c_chord /(2*V);
r_hat	= r*b_wing /(2*V);
q_bar_S = 1/2*rho_air*V^2*S_wing;

% load aircraft parameters
load parameters_2015.09.10_1956_6DoF.mat
for i = 1:length(parameters)
    eval([parameters(i).Name,' = ',num2str(parameters(i).Value),';'])
end

% stability axis aerodynamic force coefficients
cD      = cD0 + cDa*alpha + cDa2*alpha^2;
cY_s    = cYb*beta;
cL      = cL0 + cLa*alpha + cLa2*alpha^2 + cLa3*alpha^3;

% body frame aerodynamic forces
cXYZ    = H_W2B*[-cD; cY_s; -cL];
X       = q_bar_S * cXYZ(1);
Y       = q_bar_S * cXYZ(2);
Z       = q_bar_S * cXYZ(3);

% stability axis aerodynamic moment coefficients
cl_s    = clb*beta + clp*p_hat + clr*r_hat + clda*uA;
cm_s    = cm0 + cma*alpha + cmq*q_hat + cmde*uE;
cn_s    = cnb*beta + cnp*p_hat + cnr*r_hat + cndr*uR;

% body frame aerodynamic moments
clmn    = H_W2B*[cl_s; cm_s; cn_s];
Lm      = q_bar_S * b_wing * clmn(1);
Mm      = q_bar_S * c_chord * clmn(2);
Nm      = q_bar_S * b_wing * clmn(3);
     
% thrust force
T   = cT1*uT;                                                              % neglecting 2nd order term

% intermediate state differentials
u_dot       = r*v - q*w - g*sin(theta) + (X+T*cos(i_thrust))/mass;
v_dot       = p*w - r*u + g*sin(phi)*cos(theta) + Y/mass;
w_dot       = q*u - p*v + g*cos(phi)*cos(theta) + (Z+T*sin(i_thrust))/mass;

I1          = Ixz*(Iyy-Ixx-Izz);
I2          = (Ixx*Izz-Ixz^2);
p_dot       = (Izz*Lm+Ixz*Nm-(I1*p+(Ixz^2+Izz*(Izz-Iyy))*r)*q)/I2;
q_dot       = (Mm-(Ixx-Izz)*p*r-Ixz*(p^2-r^2))/Iyy;                        % neglecting thrusting moments
r_dot       = (Ixz*Lm+Ixx*Nm+(I1*r+(Ixz^2+Ixx*(Ixx-Iyy))*p)*q)/I2;

phi_dot     = p + (q*sin(phi) + r*cos(phi))*tan(theta);
theta_dot   = q*cos(phi) - r*sin(phi);

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

% ode
f = acado.DifferentialEquation;
f.add(dot(u) == u_dot);
f.add(dot(v) == v_dot);
f.add(dot(w) == w_dot);
f.add(dot(p) == p_dot);
f.add(dot(q) == q_dot);
f.add(dot(r) == r_dot);
f.add(dot(phi) == phi_dot);
f.add(dot(theta) == theta_dot);
f.add(dot(dummy) == sv);
f.add(dot(intg_V) == V * kiV);
f.add(dot(x_w2_uT) == Aw2*x_w2_uT + Bw2*uT);
f.add(dot(x_w2_uE) == Aw2*x_w2_uE + Bw2*uE);
f.add(dot(x_w2_uA) == Aw2*x_w2_uA + Bw2*uA);
f.add(dot(x_w2_uR) == Aw2*x_w2_uR + Bw2*uR);
f.add(dot(x_w2_sv) == Aw2*x_w2_sv + Bw2*sv);


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
ocp.subjectTo( 0.1 <= uT <= 1 );
ocp.subjectTo( -20*pi/180 <= uE <= 20*pi/180 );
ocp.subjectTo( -20*pi/180 <= uA <= 20*pi/180 );
ocp.subjectTo( -20*pi/180 <= uR <= 20*pi/180 );
ocp.subjectTo( -60*pi/180 <= phi <= 60*pi/180 );
ocp.subjectTo( -45*pi/180 <= theta <= 45*pi/180 );
ocp.subjectTo( -2*pi/180 <= alpha + sv );
ocp.subjectTo( alpha - sv <= 10*pi/180 );
ocp.subjectTo( 0 <= sv <= 3*pi/180 );

setNOD(ocp, 5);

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
    '/export_nmpc/qpoases')
nmpc.exportCode( 'export_nmpc' );

cd export_nmpc
make_acado_solver('../acado_nmpc_step')
cd ..

