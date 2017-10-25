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

% AUXILLARY STATES

alpha_expr = theta - gamma; % assume no beta
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
alpha = sym('alpha','real');
tracked_expr(idx_tracked_expr, :) = [alpha, alpha_expr];

mu = phi; % assume no beta

Vsafe_expr = V;
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
Vsafe = sym('Vsafe','real');
tracked_expr(idx_tracked_expr, :) = [Vsafe, Vsafe_expr];  
% if (Vsafe<1.0) Vsafe = 1.0;

% CONTROLS ////////////////////////////////////////////////////////////////
syms u_T;       % (throttle input)              [~]
syms phi_ref;	% (roll angle reference)        [rad]
syms theta_ref;	% (pitch angle reference)       [rad]

ctrls   = [u_T,phi_ref,theta_ref];
n_U     = length(ctrls);

assume(ctrls,'real');

% ONLINE DATA /////////////////////////////////////////////////////////////
syms pparam1;           % 
syms pparam2;           % 
syms pparam3;           % 
syms pparam4;           % 
syms pparam5;           % 
syms pparam6;           % 
syms pparam7;           % 
syms pparam1_next;      % 
syms pparam2_next;      % 
syms pparam3_next;      % 
syms pparam4_next;      % 
syms pparam5_next;      % 
syms pparam6_next;      % 
syms pparam7_next;      % 
syms R_acpt;            % switching acceptance radius
syms ceta_acpt;         % switching acceptance cosine of error angle
syms wn;                % northing wind
syms we;                % easting wind
syms wd;                % down wind
syms alpha_p_co;        % angle of attack upper cut-off
syms alpha_m_co;        % angle of attack lower cut-off
syms alpha_delta_co;    % angle of attack cut-off transition length\
syms T_b_lat;           % lateral-directional track-error boundary constant
syms T_b_lon;           % longitudinal track-error boundary constant
syms ddot_clmb;         % max climb rate
syms ddot_sink;         % max sink rate
vG_min = 1;

onlinedata  = [...
    pparam1,pparam2,pparam3,pparam4,pparam5,pparam6,pparam7,...
    pparam1_next,pparam2_next,pparam3_next,pparam4_next,pparam5_next,pparam6_next,pparam7_next,...
    R_acpt,ceta_acpt,...
    wn,we,wd,...
    alpha_p_co,alpha_m_co,alpha_delta_co,...
    T_b_lat,T_b_lon,...
    ddot_clmb,ddot_sink];
n_OD = length(onlinedata);

assume(onlinedata,'real');

% /////////////////////////////////////////////////////////////////////////
% STATE DIFFERENTIALS /////////////////////////////////////////////////////

% POSITION DIFFERENTIALS

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

% VELOCITY FRAME DIFFERENTIALS

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

% ATTITUDE DIFFERENTIALS
phi_dot = p;
theta_dot = q*cos(phi)-r*sin(phi);

% BODY RATE DIFFERENTIALS

Lm = Lp * p + Lr * r + LeR * (phi_ref-phi);
Mm = Vsafe^2 * (M0 + Ma * alpha + Mq * q + MeP * (theta_ref-theta));
Nm = Nr * r + NR * phi + NRR * phi_ref;

p_dot = Lm;
q_dot = Mm;
r_dot = Nm;

% THROTTLE DELAY

delta_T_dot = (u_T - delta_T) / tauT;

% SWITCHING STATE

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

tP_n_unit = tP_n/sqrt(tP_e^2 + tP_n^2);
tP_e_unit = tP_e/sqrt(tP_e^2 + tP_n^2);

% TRACK POSITION ERROR

rp_n = p_n - n;
rp_e = p_e - e;
rp_d = p_d - d;

norm_rp_ne = sqrt(rp_n^2 + rp_e^2);

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

e_lat_expr = tP_n_unit * rp_e - rp_n * tP_e_unit;
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
e_lat = sym('e_lat','real');
tracked_expr(idx_tracked_expr, :) = [e_lat, e_lat_expr];

e_lon_expr = rp_d;
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
e_lon = sym('e_lon','real');
tracked_expr(idx_tracked_expr, :) = [e_lon, e_lon_expr];

% LATERAL-DIRECTIONAL GUIDANCE

norm_vG_lat_expr = sqrt(e_dot^2+n_dot^2);

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
norm_vG_lat = sym('norm_vG_lat','real');
tracked_expr(idx_tracked_expr, :) = [norm_vG_lat, norm_vG_lat_expr];

e_b_lat_expr = norm_vG_lat*T_b_lat;
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
e_b_lat = sym('e_b_lat','real');
tracked_expr(idx_tracked_expr, :) = [e_b_lat, e_b_lat_expr];

e_b_lat2_expr = T_b_lat/2/vG_min*(e_dot^2+n_dot^2)+0.5*T_b_lat*vG_min;
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
e_b_lat2 = sym('e_b_lat2','real');
tracked_expr(idx_tracked_expr, :) = [e_b_lat2, e_b_lat2_expr];

sat_e_lat_expr = abs(e_lat)/e_b_lat;
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
sat_e_lat = sym('sat_e_lat','real');
tracked_expr(idx_tracked_expr, :) = [sat_e_lat, sat_e_lat_expr];
% if (sat_e_lat > 1.0) sat_e_lat = 1.0;

thetal=-sat_e_lat*(sat_e_lat-2.0);
l_n=(1.0-thetal)*tP_n_unit+thetal*rp_n_unit;
l_e=(1.0-thetal)*tP_e_unit+thetal*rp_e_unit;

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
% if (eta_lat>3.141592653589793) {
%     eta_lat = eta_lat - 6.283185307179586;
% }
% else if (eta_lat<-3.141592653589793) {
%     eta_lat = eta_lat + 6.283185307179586;
% }

% LONGITUDINAL GUIDANCE

norm_vG_lon_expr = sqrt(e_dot^2+n_dot^2+d_dot^2);
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
norm_vG_lon = sym('norm_vG_lon','real');
tracked_expr(idx_tracked_expr, :) = [norm_vG_lon, norm_vG_lon_expr];

ddot_sp_expr = norm_vG_lon*tP_d/sqrt(tP_e*tP_e+tP_n*tP_n);
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
ddot_sp = sym('ddot_sp','real');
tracked_expr(idx_tracked_expr, :) = [ddot_sp, ddot_sp_expr];
% if (ddot_sp>ddot_sink) ddot_sp=ddot_sink;
% if (ddot_sp<-ddot_clmb) ddot_sp=-ddot_clmb;

delta_d_clmb_expr = (-ddot_clmb-0.1-ddot_sp);
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
delta_d_clmb = sym('delta_d_clmb','real');
tracked_expr(idx_tracked_expr, :) = [delta_d_clmb, delta_d_clmb_expr];

delta_d_sink_expr = (ddot_sink+0.1-ddot_sp);
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
delta_d_sink = sym('delta_d_sink','real');
tracked_expr(idx_tracked_expr, :) = [delta_d_sink, delta_d_sink_expr];

syms delta_ddot;
% delta_ddot = (e_lon<0.0) ? delta_d_clmb : delta_d_sink;

sat_e_lon_expr = abs(e_lon/T_b_lon/delta_ddot);
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
sat_e_lon = sym('sat_e_lon','real');
tracked_expr(idx_tracked_expr, :) = [sat_e_lon, sat_e_lon_expr];
% if (sat_e_lon>1.0) sat_e_lon=1.0;
thetal_lon = -sat_e_lon*(sat_e_lon-2.0);
eta_lon = (delta_ddot*thetal_lon + ddot_sp - d_dot)/(ddot_clmb+ddot_sink+0.2);

% SOFT CONSTRAINTS

a_soft_if1_expr = alpha_p_co-alpha_delta_co;
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
a_soft_if1 = sym('a_soft_if1','real');
tracked_expr(idx_tracked_expr, :) = [a_soft_if1, a_soft_if1_expr];

a_soft_if2_expr = alpha_m_co+alpha_delta_co;
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
a_soft_if2 = sym('a_soft_if2','real');
tracked_expr(idx_tracked_expr, :) = [a_soft_if2, a_soft_if2_expr];

a_soft_p_expr = ((alpha-(alpha_p_co-alpha_delta_co))/alpha_delta_co)^2;
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
a_soft_p = sym('a_soft_p','real');
tracked_expr(idx_tracked_expr, :) = [a_soft_p, a_soft_p_expr];

a_soft_m_expr = ((alpha-(alpha_m_co+alpha_delta_co))/alpha_delta_co)^2;
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
a_soft_m = sym('a_soft_m','real');
tracked_expr(idx_tracked_expr, :) = [a_soft_m, a_soft_m_expr];

syms a_soft;
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

