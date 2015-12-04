% -------------------------------------------------------------------------
% Techpod Model Setup
% -------------------------------------------------------------------------
clc;
clear all;
close all;

% BEGIN_ACADO;

Ts = 0.05;	% model discretization step [50 ms]

% STATES - - - - - - - -
DifferentialState V;            %	(airspeed)                  [m/s]
DifferentialState beta;         %	(sideslip angle)            [rad]
DifferentialState alpha;        %   (angle of attack)           [rad]
DifferentialState p;            %   (roll rate)                 [rad/s]
DifferentialState q;            %   (pitch rate)                [rad/s]   
DifferentialState r;            %   (yaw rate)                  [rad/s]   
DifferentialState phi;          %   (roll angle)                [rad] 
DifferentialState theta;        %   (pitch angle)               [rad]  
DifferentialState psi;          %   (yaw angle)                 [rad]   
DifferentialState n;            %   (local position north)      [m]   
DifferentialState e;            %   (local position east)       [m]  
DifferentialState d;            %   (local position down)       [m]

DifferentialState dummy;        %   (dummy slack state)         [~]

DifferentialState intg_lon;     %   (longitudinal integrator)   [rad*s]
DifferentialState intg_lat;     %   (lateral integrator)        [rad*s]
DifferentialState intg_V;       %   (airspeed integrator)       [m]

DifferentialState x_w2_uT;      %   (throtte weighting state)   [~]
DifferentialState x_w2_uE;      %   (elevator weighting state)	[~]
DifferentialState x_w2_uA;      %   (aileron weighting state)   [~]
DifferentialState x_w2_uR;      %   (rudder weighting state)	[~]

% CONTROLS - - - - - - -
Control uT;     %   (throttle setting)      [0,1]
Control uE;     %   (elevator deflection)   [rad]
Control uA;     %   (aileron deflection)	[rad]
Control uR;     %   (rudder deflection)     [rad]

Control sv;     %   (slack variable)        [~]

% ONLINE DATA - - - - - -
OnlineData kplon;
OnlineData kilon;
OnlineData kplat;
OnlineData kilat;
OnlineData ddr;

OnlineData aa_n;
OnlineData aa_e;
OnlineData aa_d;
OnlineData oo_n;
OnlineData oo_e;
OnlineData oo_d;

OnlineData V_cmd;
OnlineData kpV;
OnlineData kiV;
OnlineData swV;

OnlineData Aw2;
OnlineData Bw2;
OnlineData Cw2;
OnlineData Dw2;

OnlineData wn;
OnlineData we;
OnlineData wd;

% intermediate states
u       = V*cos(alpha)*cos(beta);
v       = V*sin(beta);
w       = V*sin(alpha)*cos(beta);

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

% state differentials
V_dot       = (u*u_dot + v*v_dot + w*w_dot)/V;
beta_dot    = (V*v_dot - v*V_dot)/(V^2*cos(beta));
alpha_dot   = (u*w_dot - w*u_dot)/(u^2 + w^2);

I1          = Ixz*(Iyy-Ixx-Izz);
I2          = (Ixx*Izz-Ixz^2);
p_dot       = (Izz*Lm+Ixz*Nm-(I1*p+(Ixz^2+Izz*(Izz-Iyy))*r)*q)/I2;
q_dot       = (Mm-(Ixx-Izz)*p*r-Ixz*(p^2-r^2))/Iyy;                        % neglecting thrusting moments
r_dot       = (Ixz*Lm+Ixx*Nm+(I1*r+(Ixz^2+Ixx*(Ixx-Iyy))*p)*q)/I2;

phi_dot     = p + (q*sin(phi) + r*cos(phi))*tan(theta);
theta_dot   = q*cos(phi) - r*sin(phi);
psi_dot     = (q*sin(phi) + r*cos(phi))/cos(theta);

n_dot       = u*cos(theta)*cos(psi)+v*(sin(phi)*sin(theta)*cos(psi)-...
    cos(phi)*sin(psi))+w*(sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi));
e_dot       = u*cos(theta)*sin(psi)+v*(cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi))+...
    w*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
d_dot       = -u*sin(theta)+v*sin(phi)*cos(theta)+w*cos(phi)*cos(theta);

% AUGMENTED GUIDANCE LOGIC ------------------------------------------------

% closest point on track
oo_aa_n     = oo_n - aa_n;
oo_aa_e     = oo_e - aa_e;
oo_aa_d     = oo_d - aa_d;

sum2oo_aa   = oo_aa_n^2 + oo_aa_e^2 + oo_aa_d^2;

n_aa_n      = n - aa_n;
e_aa_e      = e - aa_e;
d_aa_d      = d - aa_d;

dot_sum2    = (n_aa_n*oo_aa_n + e_aa_e*oo_aa_e + d_aa_d*oo_aa_d)/sum2oo_aa;

dd_n        = aa_n + dot_sum2*oo_aa_n;
dd_e        = aa_e + dot_sum2*oo_aa_e;
dd_d        = aa_d + dot_sum2*oo_aa_d;

% imaginary point rr
normoo_aa   = sqrt(sum2oo_aa);

bb_oo_n     = oo_aa_n*1000;
bb_oo_e     = oo_aa_e*1000;
bb_oo_d     = oo_aa_d*1000;

bb_n        = oo_n + bb_oo_n;
bb_e        = oo_e + bb_oo_e;
bb_d        = oo_d + bb_oo_d;

bb_dd_n     = bb_n - dd_n;
bb_dd_e     = bb_e - dd_e;
bb_dd_d     = bb_d - dd_d;

normbb_dd   = sqrt(bb_dd_n^2+bb_dd_e^2+bb_dd_d^2);
normbb_oo   = sqrt(bb_oo_n^2+bb_oo_e^2+bb_oo_d^2);

rr_n        = dd_n + oo_aa_n/normoo_aa*ddr;
rr_e        = dd_e + oo_aa_e/normoo_aa*ddr;
rr_d        = dd_d + oo_aa_d/normoo_aa*ddr;

% LOS vector
LL_n    = rr_n - n;
LL_e    = rr_e - e;
LL_d    = rr_d - d;

V_lat    = sqrt((n_dot+wn)^2 + (e_dot+we)^2);
LL_lat   = sqrt(LL_n^2 + LL_e^2);
LL_lon   = sqrt(LL_n^2 + LL_e^2 + LL_d^2);

% longitudinal error angle
V_lon           = sqrt((n_dot+wn)^2 + (e_dot+we)^2 + (d_dot+wd)^2);
normVL_lon      = V_lon*LL_lon;
crossVL_lon     = -(V_lat*LL_d-(d_dot+wd)*LL_lat)/normVL_lon;
lgstc_cross_lon	= 2/(1+exp(-607*crossVL_lon))-1;
dotVL_lon       = (V_lat*LL_lat+(d_dot+wd)*LL_d)/normVL_lon;
lgstc_dot_lon   = 1/(1+exp(-607*(dotVL_lon)));
etalon          = asin(crossVL_lon*0.999)*1.0293*lgstc_dot_lon + ...
    (1-lgstc_dot_lon)*lgstc_cross_lon*pi/2;

% lateral error angle
normVL_lat      = V_lat*LL_lat;
crossVL_lat     = ((n_dot+wn)*LL_e-(e_dot+we)*LL_n)/normVL_lat;
lgstc_cross_lat	= 2/(1+exp(-607*crossVL_lat))-1;
dotVL_lat       = ((n_dot+wn)*LL_n+(e_dot+we)*LL_e)/normVL_lat;
lgstc_dot_lat   = 1/(1+exp(-607*(dotVL_lat)));
etalat          = asin(crossVL_lat*0.999)*1.0293*lgstc_dot_lat + ...
    (1-lgstc_dot_lat)*lgstc_cross_lat*pi/2;

% error angle PI control
etalonPI    = etalon*kplon + intg_lon;
etalatPI    = etalat*kplat + intg_lat;

% guidance commands
gamma_cmd   = atan(2*V_lon^2*etalonPI/LL_lon/g)/2;
phi_cmd     = atan(2*V_lat^2*etalatPI/LL_lat/g);
beta_cmd    = 0;
e_V = V_cmd + swV*atan(kpV*(normbb_dd-normbb_oo))*2/pi - V;

% % MIXED SENSITIVITY LOOP SHAPING ------------------------------------------
z_w2_uT = Cw2*x_w2_uT + Dw2*uT;
z_w2_uE = Cw2*x_w2_uE + Dw2*uE;
z_w2_uA = Cw2*x_w2_uA + Dw2*uA;
z_w2_uR = Cw2*x_w2_uR + Dw2*uR;

% OPTIMAL CONTROL PROBLEM -------------------------------------------------

% state output
gamma = -asin((d_dot+wd)/V_lon);
y = [ e_V + intg_V; gamma_cmd-gamma; phi_cmd-phi; beta_cmd-beta ];

% control output
z = [ z_w2_uT; z_w2_uE; z_w2_uA; z_w2_uR; sv ];

% lengths
N   = 10;                   % horizon
n_X = length(diffStates);   % states
n_U = length(controls);     % controls
n_Y = length(y);            % outputs
n_Z = length(z);            % objectives

% ode
f = acado.DifferentialEquation;
f.add(dot(V) == V_dot);
f.add(dot(beta) == beta_dot);
f.add(dot(alpha) == alpha_dot);
f.add(dot(p) == p_dot);
f.add(dot(q) == q_dot);
f.add(dot(r) == r_dot);
f.add(dot(phi) == phi_dot);
f.add(dot(theta) == theta_dot);
f.add(dot(psi) == psi_dot);
f.add(dot(n) == n_dot);
f.add(dot(e) == e_dot);
f.add(dot(d) == d_dot);
f.add(dot(dummy) == sv);
f.add(dot(intg_lon) == etalon*kilon);
f.add(dot(intg_lat) == etalat*kilat);
f.add(dot(intg_V) == e_V*kiV);
f.add(dot(x_w2_uT) == Aw2*x_w2_uT + Bw2*uT);
f.add(dot(x_w2_uE) == Aw2*x_w2_uE + Bw2*uE);
f.add(dot(x_w2_uA) == Aw2*x_w2_uA + Bw2*uA);
f.add(dot(x_w2_uR) == Aw2*x_w2_uR + Bw2*uR);

Q = eye(n_Y+n_Z,n_Y+n_Z);
% Q = eye(n_Y+n_U,n_Y+n_U);
Q = acado.BMatrix(Q);

QN = eye(n_Y,n_Y);
QN = acado.BMatrix(QN);

% objective
h = [ y; z ];

acadoSet('problemname', 'nmpc_augm');

% optimal control problem
ocp = acado.OCP( 0.0, N*Ts, N );

% minimization
ocp.minimizeLSQ( Q, h );
ocp.minimizeLSQEndTerm( QN, y );

ocp.subjectTo( f );
ocp.subjectTo( 0 <= uT <= 1 );
ocp.subjectTo( -20*pi/180 <= uE <= 20*pi/180 );
ocp.subjectTo( -20*pi/180 <= uA <= 20*pi/180 );
ocp.subjectTo( -20*pi/180 <= uR <= 20*pi/180 );
ocp.subjectTo( -60*pi/180 <= phi <= 60*pi/180 );
ocp.subjectTo( -40*pi/180 <= theta <= 40*pi/180 );
ocp.subjectTo( -2*pi/180 <= alpha + sv );
ocp.subjectTo( alpha - sv <= 11*pi/180 );
ocp.subjectTo( 0 <= sv <= 2*pi/180 );

setNOD(ocp, 22);

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
copyfile('../../../external_packages/qpoases', 'export_nmpc_augm/qpoases')
nmpc.exportCode( 'export_nmpc_augm' );

cd export_nmpc_augm
make_acado_solver('../acado_nmpc_augm_step')
cd ..

