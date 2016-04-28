function [d_states, output]  = techpod_nonlin_model_13DoF(time, states, ctrls,...
    cD0, cDa, cDa2, ...
    cL0, cLa, cLa2, cLa3, ...
    cm0, cma, cmq, cmde, ...
    cT0, cT1, cT2, tauT, ...
    clb, clp, clr, clda, ...
    cYb, ...
    cnb, cnp, cnr, cndr, ...
    wn, we, wd, ...
    varargin)
%--------------------------------------------------------------------------
% TECHPOD NONLINEAR 6DoF AIRCRAFT MODEL
%
% last update 08.03.16, Thomas Stastny

%--------------------------------------------------------------------------

if time > 55
    stopppp=1;
end

% states
V       = states(1);    % x-body velocity
beta       = states(2);    % y-body velocity
alpha       = states(3);    % z-body velocity

p       = states(4);    % roll rate
q       = states(5);    % pitch rate
r       = states(6);    % yaw rate

phi 	= states(7);    % roll angle
theta   = states(8);    % pitch angle
psi     = states(9);    % flight path angle

n       = states(10);   % inertial north position
e       = states(11);   % inertial east position
d       = states(12);   % inertial down position

uT      = states(13);   % throttle setting

% controls
uTcmd   = ctrls(1);     % throttle setting command
uE      = ctrls(2);     % elevator deflection
uA      = ctrls(3);     % aileron deflection (uniform)
uR      = ctrls(4);     % rudder deflection

if uTcmd>1, uTcmd=1; end;
if uTcmd<0, uTcmd=0; end;
if uE>20*pi/180, uE=20*pi/180; end;
if uE<-20*pi/180, uE=-20*pi/180; end;
if uA>20*pi/180, uA=20*pi/180; end;
if uA<-20*pi/180, uA=-20*pi/180; end;
if uR>20*pi/180, uR=20*pi/180; end;
if uR<-20*pi/180, uR=-20*pi/180; end;

% MODEL CONSTRUCTION ------------------------------------------------------

% intermediate states
% V       = sqrt(u^2+v^2+w^2);
% beta    = asin(v/V);
% alpha   = atan(w/u);

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

% stability axis aerodynamic force coefficients
cD      = cD0 + cDa*alpha + cDa2*alpha^2;
cY_s    = cYb*beta;
cL_poly = cL0 + cLa*alpha + cLa2*alpha^2 + cLa3*alpha^3;
cL_stall= -1.257*alpha + 1.248;
lgstc   = 1/(1+exp(-100*(alpha-0.257)));
cL      = (1-lgstc)*cL_poly + lgstc*cL_stall;

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
clmn    = [cl_s; cm_s; cn_s];
Lm      = q_bar_S * b_wing * clmn(1);
Mm      = q_bar_S * c_chord * clmn(2);
Nm      = q_bar_S * b_wing * clmn(3);
     
% thrust force
T   = cT0 + cT1*uT + cT2*uT^2;                                                              % neglecting 2nd order term

% intermediate states
u           = V*cos(alpha)*cos(beta);
v           = V*sin(beta);
w           = V*sin(alpha)*cos(beta);

% intermediate state differentials
u_dot       = r*v - q*w - g*sin(theta) + (X+T*cos(i_thrust))/mass;
v_dot       = p*w - r*u + g*sin(phi)*cos(theta) + Y/mass;
w_dot       = q*u - p*v + g*cos(phi)*cos(theta) + (Z+T*sin(i_thrust))/mass;

% state differentials
V_dot       = (u*u_dot + v*v_dot + w*w_dot)/V;
beta_dot    = (V*v_dot - v*V_dot)/(V^2*cos(beta));
alpha_dot   = (u*w_dot - w*u_dot)/(u^2 + w^2);

if time >= 1 && time < 1.5
     p_dot_ext = 25;
     q_dot_ext = 25;
     r_dot_ext = 25;
     wn = 5;
     we = 5;
     wd = 5;
else
     p_dot_ext = 0;
     q_dot_ext = 0;
     r_dot_ext = 0;
     wn = 0;
     we = 0;
     wd = 0;
end

I1          = Ixz*(Iyy-Ixx-Izz);
I2          = (Ixx*Izz-Ixz^2);
p_dot       = (Izz*Lm+Ixz*Nm-(I1*p+(Ixz^2+Izz*(Izz-Iyy))*r)*q)/I2 + p_dot_ext;
q_dot       = (Mm-(Ixx-Izz)*p*r-Ixz*(p^2-r^2))/Iyy + q_dot_ext;                        % neglecting thrusting moments
r_dot       = (Ixz*Lm+Ixx*Nm+(I1*r+(Ixz^2+Ixx*(Ixx-Iyy))*p)*q)/I2 + r_dot_ext;

phi_dot     = p + (q*sin(phi) + r*cos(phi))*tan(theta);
theta_dot   = q*cos(phi) - r*sin(phi);
psi_dot     = (q*sin(phi) + r*cos(phi))/cos(theta);

n_dot       = u*cos(theta)*cos(psi)+v*(sin(phi)*sin(theta)*cos(psi)-...
    cos(phi)*sin(psi))+w*(sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi)) + wn;
e_dot       = u*cos(theta)*sin(psi)+v*(cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi))+...
    w*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)) + we;
d_dot       = -u*sin(theta)+v*sin(phi)*cos(theta)+w*cos(phi)*cos(theta) + wd;

uT_dot      = (uTcmd - uT)/tauT;

% DIFFERENTIAL STATES/OUTPUTS ---------------------------------------------

% d_states
d_states(1)     = V_dot;
d_states(2)     = beta_dot;
d_states(3)     = alpha_dot;
d_states(4)     = p_dot;
d_states(5)     = q_dot;
d_states(6)     = r_dot;
d_states(7)     = phi_dot;
d_states(8)     = theta_dot;
d_states(9)     = psi_dot;
d_states(10)    = n_dot;
d_states(11)    = e_dot;
d_states(12)    = d_dot;
d_states(13)    = uT_dot;

% output
output(1)	= V;
output(2)	= beta;
output(3)	= alpha;
output(4)	= p;
output(5)	= q;
output(6)	= r;
output(7)	= phi;
output(8)	= theta;
output(9)   = psi;
output(10)  = n;
output(11)  = e;
output(12)  = d;
output(13)  = uT;
