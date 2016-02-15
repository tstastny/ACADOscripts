% -------------------------------------------------------------------------
% Techpod Model Setup
% -------------------------------------------------------------------------

tracked_expr = sym('tracked_expr',0);
idx_tracked_expr = 0;

atan2_args = sym('atan2_args',0);
atan2s = sym('atan2s',0);
idx_atan2s = 1;
idx_atan2s_last = idx_atan2s;

% STATES ------------------------------------------------------------------
syms u;            %    (x-body velocity)           [m/s]
syms v;            %    (y-body velocity)           [rad]
syms w;            %    (z-body velocity)           [rad]
syms p;            %    (roll rate)                 [rad/s]
syms q;            %    (pitch rate)                [rad/s]   
syms r;            %    (yaw rate)                  [rad/s]   
syms phi;          %    (roll angle)                [rad] 
syms theta;        %    (pitch angle)               [rad]  
syms psi;          %    (yaw angle)                 [rad]   

syms dummy;        %    (dummy slack state)         [~]

syms intg_V;       %    (integral of airspeed)      [m]

syms x_w2_uT;      %    (throtte w2)                [~]
syms x_w2_uE;      %    (elevator w2)               [~]
syms x_w2_uA;      %    (aileron w2)                [~]
syms x_w2_uR;      %    (rudder w2)                 [~]
syms x_w2_sv;      %    (slack w2)                  [~]

states = [u,v,w,p,q,r,phi,theta,dummy,intg_V,x_w2_uT,x_w2_uE,x_w2_uA,x_w2_uR,x_w2_sv];
n_X     = length(states);

% CONTROLS ----------------------------------------------------------------
syms uT;     %   (throttle setting)      [0,1]
syms uE;     %   (elevator deflection)   [rad]
syms uA;     %   (aileron deflection)	[rad]
syms uR;     %   (rudder deflection)     [rad]

syms sv;     %   (slack variable)        [~]

ctrls = [uT,uE,uA,uR,sv];
n_U     = length(ctrls);

% ONLINE DATA -------------------------------------------------------------
syms Aw2;
syms Bw2;
syms Cw2;
syms Dw2;

syms kiV;

onlinedata = [Aw2,Bw2,Cw2,Dw2,kiV];
n_OD    = length(onlinedata);

% MODEL CONSTRUCTION ------------------------------------------------------

% intermediate states
V       = sqrt(u^2+v^2+w^2);
beta    = asin(v/V);
alpha   = atan(w/u);

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

% MIXED SENSITIVITY LOOP SHAPING ------------------------------------------

z_w2_uT = Cw2*x_w2_uT + Dw2*uT;
z_w2_uE = Cw2*x_w2_uE + Dw2*uE;
z_w2_uA = Cw2*x_w2_uA + Dw2*uA;
z_w2_uR = Cw2*x_w2_uR + Dw2*uR;
z_w2_sv = Cw2*x_w2_sv + Dw2*sv;

% REMAINING DIFFERENTIALS -------------------------------------------------

dummy_dot   = sv;
intg_V_dot	= V * kiV;
x_w2_uT_dot	= Aw2*x_w2_uT + Bw2*uT;
x_w2_uE_dot	= Aw2*x_w2_uE + Bw2*uE;
x_w2_uA_dot	= Aw2*x_w2_uA + Bw2*uA;
x_w2_uR_dot	= Aw2*x_w2_uR + Bw2*uR;
x_w2_sv_dot	= Aw2*x_w2_sv + Bw2*sv;

% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////

% OPTIMAL CONTROL PROBLEM -------------------------------------------------

% ode - right-hand side
for i = 1:n_X
    eval(['frhs(i,1) = ',char(states(i)),'_dot;']);
end

% state output
y = [ V + intg_V; theta; phi; beta; p; q; r ];
n_Y     = length(y);

% objectives
z = [ z_w2_uT; z_w2_uE; z_w2_uA; z_w2_uR; z_w2_sv ];
n_Z     = length(z);

% lsq objective functions
objectives = [ y; z];

% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////

% CODE GENERATION ---------------------------------------------------------

% define symbolic "in"s
for i = 1:(n_X+n_U+n_OD)
    if i>10                                                                % double digits
        eval(['syms in',int2str(i-1)]);
        ins(i) = eval(['in',int2str(i-1)]);
        eval(['clear in',int2str(i-1)]);
    else
        eval(['syms in0',int2str(i-1)]);                                   % single digits
        ins(i) = eval(['in0',int2str(i-1)]);
        eval(['clear in0',int2str(i-1)]);
    end
end

% substitute generic "in"s
if ~isempty(tracked_expr)
    tracked_expr = subs( tracked_expr, [states ctrls onlinedata], ins );
end
frhs = subs( frhs, [states ctrls onlinedata], ins );
objectives = subs( objectives, [states ctrls onlinedata], ins );
y = subs( y, [states ctrls onlinedata], ins );

% generate optimized c code
if ~isempty(tracked_expr)
    ccode([tracked_expr(:,2); frhs],'file','ext_rhs_.c');
    ccode([tracked_expr(:,2); objectives],'file','ext_lsq_obj_.c')
    ccode([tracked_expr(:,2);  y],'file','ext_lsq_obj_N_.c');
else
    ccode(frhs,'file','ext_rhs_.c');
    ccode(objectives,'file','ext_lsq_obj_.c')
    ccode(y,'file','ext_lsq_obj_N_.c');
end
