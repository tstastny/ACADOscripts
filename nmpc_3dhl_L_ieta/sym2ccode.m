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
syms sw;        % (segment switching state)

states  = [n,e,d,V,gamma,xi,phi,theta,p,q,r,delta_T,sw];
n_X     = length(states);

assume(states,'real');

% auxillary states
alpha_expr = theta - gamma; % assume no beta

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
alpha = sym('alpha','real');
tracked_expr(idx_tracked_expr, :) = [alpha, alpha_expr];

mu = phi; % assume no beta
Vsafe_expr = V;
% if (Vsafe<1.0) Vsafe = 1.0;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
Vsafe = sym('Vsafe','real');
tracked_expr(idx_tracked_expr, :) = [Vsafe, Vsafe_expr];  

% CONTROLS ////////////////////////////////////////////////////////////////
syms u_T;       % (throttle input)              [~]
syms phi_ref;	% (roll angle reference)        [rad]
syms theta_ref;	% (pitch angle reference)       [rad]

ctrls   = [u_T,phi_ref,theta_ref];
n_U     = length(ctrls);

assume(ctrls,'real');

% ONLINE DATA /////////////////////////////////////////////////////////////
syms pparam1;           %   type    type
syms pparam2;           %   aa_n    cc_n
syms pparam3;           %   aa_e    cc_e
syms pparam4;           %   aa_d    cc_d
syms pparam5;           %   bb_n    R
syms pparam6;           %   bb_e    dir
syms pparam7;           %   bb_d    gam
syms pparam8;           %   --      xi0
syms pparam9;           %   --      dxi
syms pparam1_next;      %   type    type
syms pparam2_next;      %   aa_n    cc_n
syms pparam3_next;      %   aa_e    cc_e
syms pparam4_next;      %   aa_d    cc_d
syms pparam5_next;      %   bb_n    R
syms pparam6_next;      %   bb_e    dir
syms pparam7_next;      %   bb_d    gam
syms pparam8_next;      %   --      xi0
syms pparam9_next;      %   --      dxi
syms R_acpt;            % switching acceptance radius
syms ceta_acpt;         % switching acceptance cosine of error angle
syms wn;                % northing wind
syms we;                % easting wind
syms wd;                % down wind
syms alpha_p_co;        % angle of attack upper cut-off
syms alpha_m_co;        % angle of attack lower cut-off
syms alpha_delta_co;    % angle of attack cut-off transition length\
syms T_b_ne;            % lateral-directional track-error boundary constant
syms T_b_d;             % longitudinal track-error boundary constant
syms vG_min;            % minimum ground speed for track-error boundary

onlinedata  = [...
    pparam1,pparam2,pparam3,pparam4,pparam5,pparam6,pparam7,pparam8,pparam9,...
    pparam1_next,pparam2_next,pparam3_next,pparam4_next,pparam5_next,pparam6_next,pparam7_next,pparam8_next,pparam9_next,...
    R_acpt,ceta_acpt,...
    wn,we,wd,...
    alpha_p_co,alpha_m_co,alpha_delta_co,...
    T_b_ne,T_b_d,vG_min];
n_OD = length(onlinedata);

assume(onlinedata,'real');

% /////////////////////////////////////////////////////////////////////////
% STATE DIFFERENTIALS /////////////////////////////////////////////////////

% position differentials

n_dot_expr = Vsafe * cos(xi) * cos(gamma) + wn;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
n_dot = sym('n_dot','real');
tracked_expr(idx_tracked_expr, :) = [n_dot, n_dot_expr];   

e_dot_expr = Vsafe * sin(xi) * cos(gamma) + we;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
e_dot = sym('e_dot','real');
tracked_expr(idx_tracked_expr, :) = [e_dot, e_dot_expr];   

d_dot_expr = -Vsafe * sin(gamma) + wd;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
d_dot = sym('d_dot','real');
tracked_expr(idx_tracked_expr, :) = [d_dot, d_dot_expr];  

% velocity frame differentials

g = 9.81;
m = 2.65;
rho = 1.225;
S = 0.39;
qbarS = 0.5*rho*Vsafe^2*S;

load parameters_20161209.mat;
for i = 1:length(parameters)
    eval([parameters(i).Name,'=',num2str(parameters(i).Value),';']);
end

cD = cD0 + cDa * alpha + cDa2 * alpha^2;
cL = cL0 + cLa * alpha + cLa2 * alpha^2;

T = (cT1*delta_T+cT2*delta_T^2+cT3*delta_T^3)/Vsafe/cos(alpha); % should probably have a check for alpha=90.. but really shouldnt happen
D = qbarS*cD;
L = qbarS*cL;

V_dot = (T*cos(alpha)-D)/m-g*sin(gamma);
gamma_dot = ((T*sin(alpha)+L)*cos(mu) - m*g*cos(gamma))/m/Vsafe;
xi_dot = (T*sin(alpha)+L)*sin(mu)/Vsafe/m/cos(gamma); % same as with alpha

% attitude differentials
phi_dot = p;
theta_dot = q*cos(phi)-r*sin(phi);

% body rate differentials

Lm = Lp * p + Lr * r + LeR * (phi_ref-phi);
Mm = Vsafe^2 * (M0 + Ma * alpha + Mq * q + MeP * (theta_ref-theta));
Nm = Nr * r + NR * phi + NRR * phi_ref;

p_dot = Lm;
q_dot = Mm;
r_dot = Nm;

% throttle delay

delta_T_dot = (u_T - delta_T) / tauT;

% switching state differential

syms sw_dot_expr;
sw_dot = sw_dot_expr;

% /////////////////////////////////////////////////////////////////////////
% OBJECTIVES //////////////////////////////////////////////////////////////

% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% --> see manual_input.c

% end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% manual output !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
p_n = sym('p_n','real');
p_e = sym('p_e','real');
p_d = sym('p_d','real');
tP_n = sym('tP_n','real');
tP_e = sym('tP_e','real');
tP_d = sym('tP_d','real');
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% track position error

rp_n = p_n - n;
rp_e = p_e - e;
rp_d = p_d - d;

norm_rp_ne_expr = sqrt(rp_n^2 + rp_e^2);

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
norm_rp_ne = sym('norm_rp_ne','real');
tracked_expr(idx_tracked_expr, :) = [norm_rp_ne, norm_rp_ne_expr];

cz = tP_n * rp_e - rp_n * tP_e;

e_ne_expr = cz;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
e_ne = sym('e_ne','real');
tracked_expr(idx_tracked_expr, :) = [e_ne, e_ne_expr];

e_d_expr = rp_d;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
e_d = sym('e_d','real');
tracked_expr(idx_tracked_expr, :) = [e_d, e_d_expr];

% guidance

% if norm(vG)>vGmin
%     e_b = norm(vG)*T_b;                               
% else
%     e_b = T_b/2/vGmin*norm(vG)^2+0.5*T_b*vGmin;
% end

e_b_ne_expr = T_b_ne/2/vG_min*(e_dot^2+n_dot^2)+0.5*T_b_ne*vG_min;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
e_b_ne = sym('e_b_ne','real');
tracked_expr(idx_tracked_expr, :) = [e_b_ne, e_b_ne_expr];

e_b_d_expr = T_b_d/2/vG_min*abs(d_dot)+0.5*T_b_d*vG_min;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
e_b_d = sym('e_b_d','real');
tracked_expr(idx_tracked_expr, :) = [e_b_d, e_b_d_expr];

sat_e_ne_expr = abs(e_ne)/e_b_ne;
% double sat_e_ne = abs(sat_e_ne_expr);
% if (sat_e_ne > 1) {
%   sat_e_ne = 1;
% }

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
sat_e_ne = sym('sat_e_ne','real');
tracked_expr(idx_tracked_expr, :) = [sat_e_ne, sat_e_ne_expr];

sat_e_d_expr = abs(e_d)/e_b_d;
% double sat_e_d = abs(sat_e_d_expr);
% if (sat_e_d > 1) {
%   sat_e_d = 1;
% }

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
sat_e_d = sym('sat_e_d','real');
tracked_expr(idx_tracked_expr, :) = [sat_e_d, sat_e_d_expr];

thetal_lat = pi/2*(sat_e_ne-1)^2;
thetal_lon = pi/2*(sat_e_d-1)^2;

rp_n_unit_expr = rp_n/norm_rp_ne;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
rp_n_unit = sym('rp_n_unit','real');
tracked_expr(idx_tracked_expr, :) = [rp_n_unit, rp_n_unit_expr];

rp_e_unit_expr = rp_e/norm_rp_ne;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
rp_e_unit = sym('rp_e_unit','real');
tracked_expr(idx_tracked_expr, :) = [rp_e_unit, rp_e_unit_expr];

sgn_rp_expr = rp_d;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
sgn_rp = sym('sgn_rp','real');
tracked_expr(idx_tracked_expr, :) = [sgn_rp, sgn_rp_expr];

l_n = cos(thetal_lat)*rp_n_unit + sin(thetal_lat)*tP_n;
l_e = cos(thetal_lat)*rp_e_unit + sin(thetal_lat)*tP_e;

l_ne = sin(thetal_lon)*sqrt(tP_n^2 + tP_e^2);
l_d = cos(thetal_lon)*sgn_rp + sin(thetal_lon)*tP_d;

eta_lat_expr = atan2(l_e,l_n) - atan2(e_dot,n_dot);

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
FaR_expr = eta_lat_expr;
findandreplace_atan2s
for i = idx_atan2s_last:idx_atan2s-1
    idx_tracked_expr = idx_tracked_expr + 1;
    tracked_expr(idx_tracked_expr,:) = [atan2s(i),atan2_args(i,1)];
    idx_tracked_expr = idx_tracked_expr + 1;
    tracked_expr(idx_tracked_expr,:) = [0,atan2_args(i,2)];
end
idx_atan2s_last = idx_atan2s;                    

idx_tracked_expr = idx_tracked_expr + 1;
syms eta_lat;
tracked_expr(idx_tracked_expr, :) = [eta_lat, FaR_expr];

eta_lon_expr = atan2(-l_d,l_ne) - atan2(-d_dot,sqrt(e_dot^2+n_dot^2)); % this is thetaL (defined positive up) and Gamma (defined positive up)

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
FaR_expr = eta_lon_expr;
findandreplace_atan2s
for i = idx_atan2s_last:idx_atan2s-1
    idx_tracked_expr = idx_tracked_expr + 1;
    tracked_expr(idx_tracked_expr,:) = [atan2s(i),atan2_args(i,1)];
    idx_tracked_expr = idx_tracked_expr + 1;
    tracked_expr(idx_tracked_expr,:) = [0,atan2_args(i,2)];
end
idx_atan2s_last = idx_atan2s;                    

idx_tracked_expr = idx_tracked_expr + 1;
syms eta_lon;
tracked_expr(idx_tracked_expr, :) = [eta_lon, FaR_expr];

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
y   = [ eta_lat; eta_lon; Vsafe; p; q; r; a_soft ];
n_Y = length(y);

% ctrl output
z   = [ delta_T_dot; u_T; phi_ref; theta_ref; u_T; phi_ref; theta_ref ];
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

