% /////////////////////////////////////////////////////////////////////////
% NMPC Model and Objective Setup //////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////

tracked_expr = sym('tracked_expr',0);
idx_tracked_expr = 0;

atan2_args = sym('atan2_args',0);
atan2s = sym('atan2s',0);
idx_atan2s = 1;
idx_atan2s_last = idx_atan2s;

% STATES //////////////////////////////////////////////////////////////////
syms n;        	% (northing)
syms e;       	% (easting)
syms d;        	% (down)
syms V;        	% (airspeed)
syms gamma;    	% (flight path angle)
syms xi;     	% (heading angle)
syms phi;     	% (roll angle)
syms theta;   	% (pitch angle)
syms p;         % (roll rate)
syms q;         % (pitch rate)
syms r;         % (yaw rate)
syms delta_T;   % (throttle setting)

states  = [n,e,d,V,gamma,xi,phi,theta,p,q,r,delta_T];
n_X     = length(states);

assume(states,'real');

% auxillary states
alpha_expr = theta - gamma; % assume no beta

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
alpha = sym('alpha','real');
tracked_expr(idx_tracked_expr, :) = [alpha, alpha_expr];

mu = phi; % assume no beta

% CONTROLS ////////////////////////////////////////////////////////////////
syms u_T;       % (throttle input)              [~]
syms phi_ref;	% (roll angle reference)        [rad]
syms theta_ref;	% (pitch angle reference)       [rad]

ctrls   = [u_T,phi_ref,theta_ref];
n_U     = length(ctrls);

assume(ctrls,'real');

% ONLINE DATA /////////////////////////////////////////////////////////////
syms wn;                % northing wind
syms we;                % easting wind
syms wd;                % down wind
syms alpha_p_co;        % angle of attack upper cut-off
syms alpha_m_co;        % angle of attack lower cut-off
syms alpha_delta_co;    % angle of attack cut-off transition length\
syms p_n;               % northing position setpoint
syms p_e;               % easting position setpoint
syms p_d;               % down position setpoint

onlinedata  = [...
    wn,we,wd,...
    alpha_p_co,alpha_m_co,alpha_delta_co,...
    p_n,p_e,p_d];
n_OD = length(onlinedata);

assume(onlinedata,'real');

% /////////////////////////////////////////////////////////////////////////
% STATE DIFFERENTIALS /////////////////////////////////////////////////////

% position differentials

n_dot = V * cos(xi) * cos(gamma) + wn;
e_dot = V * sin(xi) * cos(gamma) + we;
d_dot = -V * sin(gamma) + wd;

% velocity frame differentials

g = 9.81;
m = 2.65;
rho = 1.225;
S = 0.39;
qbarS = 0.5*rho*V^2*S;

load parameters_20161209.mat;
for i = 1:length(parameters)
    eval([parameters(i).Name,'=',num2str(parameters(i).Value),';']);
end

cD = cD0 + cDa * alpha + cDa2 * alpha^2;
cL = cL0 + cLa * alpha + cLa2 * alpha^2;

T = (cT1*delta_T+cT2*delta_T^2+cT3*delta_T^3)/V/cos(alpha); % should probably have a check for alpha=90.. but really shouldnt happen
D = qbarS*cD;
L = qbarS*cL;

V_dot = (T*cos(alpha)-D)/m-g*sin(gamma);
gamma_dot = ((T*sin(alpha)+L)*cos(mu) - m*g*cos(gamma))/m/V;
xi_dot = (T*sin(alpha)+L)*sin(mu)/V/m/cos(gamma); % same as with alpha

% attitude differentials
phi_dot = p;
theta_dot = q*cos(phi)-r*sin(phi);

% body rate differentials

Lm = Lp * p + Lr * r + LeR * (phi_ref-phi);
Mm = V^2 * (M0 + Ma * alpha + Mq * q + MeP * (theta_ref-theta));
Nm = Nr * r + NR * phi + NRR * phi_ref;

p_dot = Lm;
q_dot = Mm;
r_dot = Nm;

% throttle delay

delta_T_dot = (u_T - delta_T) / tauT;

% track position error

rp_n = p_n - n;
rp_e = p_e - e;
rp_d = p_d - d;

e_lat = sqrt(rp_e^2+rp_n^2);
e_lon = rp_d;

% soft constraints

a_soft_p_expr = ((alpha-(alpha_p_co-alpha_delta_co))/alpha_delta_co)^2;
a_soft_m_expr = ((alpha-(alpha_m_co+alpha_delta_co))/alpha_delta_co)^2;
% double a_soft;
% if (alpha>(alpha_p_co-alpha_delta_co)) {
%     a_soft=((alpha-(alpha_p_co-alpha_delta_co))/alpha_delta_co)^2;
% }
% else if (x(i)>(alpha_m_co+alpha_delta_co))
%     a_soft=0.0;
% }
% else {
%     a_soft=((alpha-(alpha_m_co+alpha_delta_co))/alpha_delta_co)^2;
% end

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
syms a_soft_p;
tracked_expr(idx_tracked_expr, :) = [a_soft_p, a_soft_p_expr];

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
syms a_soft_m;
tracked_expr(idx_tracked_expr, :) = [a_soft_m, a_soft_m_expr];

syms a_soft;

% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% OPTIMAL CONTROL PROBLEM /////////////////////////////////////////////////

% ode - right-hand side
for i = 1:n_X
    eval(['frhs(i,1) = ',char(states(i)),'_dot;']);
end

% state output
y   = [ e_lat; e_lon; V; p; q; r; a_soft ];
n_Y = length(y);

% ctrl output
z   = [ delta_T_dot; u_T; phi_ref; theta_ref];
n_Z = length(z);

% lsq objective functions
objectives = [ y; z];

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
tracked_expr_s = subs( tracked_expr, [states ctrls onlinedata], ins );
tracked_expr_y_s = subs( tracked_expr, [states onlinedata], ins(1:(end-n_U)) );
frhs_s = subs( frhs, [states ctrls onlinedata], ins );
objectives_s = subs( objectives, [states ctrls onlinedata], ins );
y_s = subs( y, [states onlinedata], ins(1:(end-n_U)) );

% generate optimized c code
ccode([tracked_expr_s(:,2); frhs_s],'file','ext_rhs_.c');
ccode([tracked_expr_s(:,2); objectives_s],'file','ext_lsq_obj_.c')
ccode([tracked_expr_y_s(:,2);  y_s],'file','ext_lsq_obj_N_.c');

