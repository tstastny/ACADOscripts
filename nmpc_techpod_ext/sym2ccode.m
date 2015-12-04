% -------------------------------------------------------------------------
% Techpod Model Setup
% -------------------------------------------------------------------------

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

% CONTROLS ----------------------------------------------------------------
syms uT;     %   (throttle setting)      [0,1]
syms uE;     %   (elevator deflection)   [rad]
syms uA;     %   (aileron deflection)	[rad]
syms uR;     %   (rudder deflection)     [rad]

syms sv;     %   (slack variable)        [~]

ctrls = [uT,uE,uA,uR,sv];

% ONLINE DATA -------------------------------------------------------------
syms Aw2;
syms Bw2;
syms Cw2;
syms Dw2;

syms kiV;

onlinedata = [Aw2,Bw2,Cw2,Dw2,kiV];

% ONLINE DATA -------------------------------------------------------------
dstbs   = [];

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

% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////

% OPTIMAL CONTROL PROBLEM -------------------------------------------------

% ode - right-hand side
frhs(1,1)	= u_dot;
frhs(2,1)	= v_dot;
frhs(3,1)	= w_dot;
frhs(4,1)	= p_dot;
frhs(5,1)	= q_dot;
frhs(6,1)	= r_dot;
frhs(7,1)	= phi_dot;
frhs(8,1)	= theta_dot;
frhs(9,1)	= sv;
frhs(10,1)	= V * kiV;
frhs(11,1)	= Aw2*x_w2_uT + Bw2*uT;
frhs(12,1)	= Aw2*x_w2_uE + Bw2*uE;
frhs(13,1)	= Aw2*x_w2_uA + Bw2*uA;
frhs(14,1)	= Aw2*x_w2_uR + Bw2*uR;
frhs(15,1)	= Aw2*x_w2_sv + Bw2*sv;

% state output
y = [ V + intg_V; theta; phi; beta; p; q; r ];

% objectives
z = [ z_w2_uT; z_w2_uE; z_w2_uA; z_w2_uR; z_w2_sv ];

% lsq objective functions
objectives = [ y; z];

% lengths
n_X     = length(frhs);
n_U     = length(ctrls);
n_OD    = length(onlinedata);
n_W     = length(dstbs);
n_Y     = length(y);
n_Z     = length(z);

% jacobians
Jrhs	= reshape(transpose(jacobian(frhs,[states,ctrls])),n_X*(n_X+n_U),1);	% rhs
Jobj_x  = reshape(transpose(jacobian(objectives,states)),(n_Y+n_Z)*n_X,1);      % obj (wrt states)
Jobj_u  = reshape(transpose(jacobian(objectives,ctrls)),(n_Y+n_Z)*n_U,1);       % obj (wrt ctrls)

% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////

% CODE GENERATION ---------------------------------------------------------

% define symbolic "in"s
for i = 1:(n_X+n_U+n_OD+n_W)
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
frhs = subs( frhs, [states ctrls onlinedata dstbs], ins );
Jrhs = subs( Jrhs, [states ctrls onlinedata dstbs], ins );
objectives = subs( objectives, [states ctrls onlinedata dstbs], ins );
Jobj_x = subs( Jobj_x, [states ctrls onlinedata dstbs], ins );
Jobj_u = subs( Jobj_u, [states ctrls onlinedata dstbs], ins );
y = subs( y, [states ctrls onlinedata dstbs], ins );

% generate optimized C code
ccode(frhs,'file','ext_rhs_.c');
ccode(Jrhs,'file','ext_Jrhs_.c');
ccode([ objectives; Jobj_x; Jobj_u ],'file','ext_lsq_obj_.c');
ccode([ y; Jobj_x ],'file','ext_lsq_obj_N_.c');

