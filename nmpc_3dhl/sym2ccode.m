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
% syms intg_e_t;	% (integral of track error)
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
syms u_T;   % (throttle input)              [~]
syms phi_ref;	% (roll angle reference)        [rad]
syms theta_ref;	% (pitch angle reference)       [rad]

ctrls   = [u_T,phi_ref,theta_ref];
n_U     = length(ctrls);

assume(ctrls,'real');

% ONLINE DATA /////////////////////////////////////////////////////////////
syms pparam1;   %   type    type
syms pparam2;   %   aa_n    cc_n
syms pparam3;   %   aa_e    cc_e
syms pparam4;   %   aa_d    cc_d
syms pparam5;   %   bb_n    R
syms pparam6;   %   bb_e    dir
syms pparam7;   %   bb_d    gam
syms pparam8;   %   --      xi0
syms pparam9;   %   --      dxi
syms pparam1_next;   %   type    type
syms pparam2_next;   %   aa_n    cc_n
syms pparam3_next;   %   aa_e    cc_e
syms pparam4_next;   %   aa_d    cc_d
syms pparam5_next;   %   bb_n    R
syms pparam6_next;   %   bb_e    dir
syms pparam7_next;   %   bb_d    gam
syms pparam8_next;   %   --      xi0
syms pparam9_next;   %   --      dxi
syms R_acpt;    % switching acceptance radius
syms ceta_acpt; % switching acceptance cosine of error angle
syms wn;    % northing wind
syms we;    % easting wind
syms wd;    % down wind
syms k_t_d;     % longitudinal logistic gain
syms e_d_co;    % longitudinal logistic cutoff
syms k_t_ne;    % lateral logistic gain
syms e_ne_co;   % lateral logistic cutoff
syms eps_v;     % unit ground speed threshold
syms alpha_p_co;   % angle of attack upper cutoff
syms alpha_m_co;   % angle of attack lower cutoff
syms alpha_delta_co;   % angle of attack cutoff transition length

onlinedata  = [...
    pparam1,pparam2,pparam3,pparam4,pparam5,pparam6,pparam7,pparam8,pparam9,...
    pparam1_next,pparam2_next,pparam3_next,pparam4_next,pparam5_next,pparam6_next,pparam7_next,pparam8_next,pparam9_next,...
    R_acpt,ceta_acpt,...
    wn,we,wd,...
    k_t_d,e_d_co,k_t_ne,e_ne_co,...
    eps_v,...
    alpha_p_co,alpha_m_co,alpha_delta_co];
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

norm_v_expr = sqrt(n_dot^2 + e_dot^2 + d_dot^2);

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
norm_v = sym('norm_v','real');
tracked_expr(idx_tracked_expr, :) = [norm_v, norm_v_expr]; 

track_eps_v_expr = eps_v;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
track_eps_v = sym('track_eps_v','real');
tracked_expr(idx_tracked_expr, :) = [track_eps_v, track_eps_v_expr]; 

% n_dot,e_dot,d_dot,norm_v,eps_v are needed for switch checks AND directional error calculations
% double vbar_n;
% double vbar_e;
% double vbar_d;
% if (fabs(norm_v)<0.05) {
%     vbar_n=0.0;
%     vbar_e=0.0;
%     vbar_d=0.0;
% }
% else if (fabs(norm_v)<eps_v) {
%     vbar_n=((sin(2*norm_v/eps_v/3.141592653589793)-1)+1)*n_dot;
%     vbar_e=((sin(2*norm_v/eps_v/3.141592653589793)-1)+1)*e_dot;
%     vbar_d=((sin(2*norm_v/eps_v/3.141592653589793)-1)+1)*d_dot;
% }
% else {
%     vbar_n=n_dot/norm_v;
%     vbar_e=e_dot/norm_v;
%     vbar_d=d_dot/norm_v;
% }
vbar_n = sym('vbar_n','real');
vbar_e = sym('vbar_e','real');
vbar_d = sym('vbar_d','real');

% velocity frame differentials

g = 9.81;
m = 2.65;
rho = 1.225;
S = 0.39;
qbarS = 0.5*rho*Vsafe^2*S;

M0=0.000848173994434413;
Ma=-0.164663711873934;
Mq=-0.0410704191099466;
MeP=0.158960374141736;
lp=-6.73578773969040;
lr=0.878182329679164;
leR=11.1467593207791;
Nr=-9.76492309398636;
NR=5.98887765504684;
NRR=1.58960918118893;

tauT=0.179092870541038;
cT1=46.5309685298437;
cT2=133.121145483364;
cT3=194.400407803984;
cD0=0.0623204432055247;
cDa=0.378620335309777;
cDa2=1.62033558854341;
cL0=0.470333508888944;
cLa=6.87094469647060;
cLa2=-21.1069893359182;

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

lm = lp * p + lr * r + leR * (phi_ref-phi);
Mm = Vsafe^2 * (M0 + Ma * alpha + Mq * q + MeP * (theta_ref-theta));
Nm = Nr * r + NR * phi + NRR * phi_ref;

p_dot = lm;
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
d_n = sym('d_n','real');
d_e = sym('d_e','real');
d_d = sym('d_d','real');
Td_n = sym('Td_n','real');
Td_e = sym('Td_e','real');
Td_d = sym('Td_d','real');
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% track position error

pd_n_expr = d_n - n;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
pd_n = sym('pd_n','real');
tracked_expr(idx_tracked_expr, :) = [pd_n, pd_n_expr];  

pd_e_expr = d_e - e;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
pd_e = sym('pd_e','real');
tracked_expr(idx_tracked_expr, :) = [pd_e, pd_e_expr];  

pd_d_expr = d_d - d;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
pd_d = sym('pd_d','real');
tracked_expr(idx_tracked_expr, :) = [pd_d, pd_d_expr];

cz = Td_n * pd_e - pd_n * Td_e;

e_t_ne_expr = cz;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
e_t_ne = sym('e_t_ne','real');
tracked_expr(idx_tracked_expr, :) = [e_t_ne, e_t_ne_expr];

e_t_d_expr = pd_d;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
e_t_d = sym('e_t_d','real');
tracked_expr(idx_tracked_expr, :) = [e_t_d, e_t_d_expr];

% double e_t_1_ne;
% if (e_t_ne>e_ne_co) {
%     e_t_1_ne = 1;
% }
% else if (e_t_ne>-e_ne_co) {
%     e_t_1_ne = sin(e_t_ne/e_ne_co*pi/2);
% }
% else {
%     e_t_1_ne = -1;
% }
e_t_1_ne_expr = sin(e_t_ne/e_ne_co*pi/2);

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
e_t_1_ne = sym('e_t_1_ne','real');
tracked_expr(idx_tracked_expr, :) = [e_t_1_ne, e_t_1_ne_expr];

% double e_t_1_d;
% if (e_t_d>e_d_co) {
%     e_t_1_d = 1;
% }
% else if (e_t_d>-e_d_co) {
%     e_t_1_d = sin(e_t_d/e_d_co*pi/2);
% }
% else {
%     e_t_1_d = -1;
% }
e_t_1_d_expr = sin(e_t_d/e_d_co*pi/2);

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
e_t_1_d = sym('e_t_1_d','real');
tracked_expr(idx_tracked_expr, :) = [e_t_1_d, e_t_1_d_expr];

% cost bounds + extra guidance

% double Tpd_n;
% double Tpd_e;
% double Tpd_d;
% if (fabs(e_t_ne)<1.0) {
%     Tpd_n = 0.0;
%     Tpd_e = 0.0;
% }
% else {
%     Tpd_n = pd_n/fabs(e_t_ne);
%     Tpd_e = pd_e/fabs(e_t_ne);
% }
% if (fabs(e_t_d)<1.0) {
%     Tpd_d = 0.0;
% }
% else {
%     Tpd_d = pd_d/fabs(e_t_d);
% }
Tpd_n = sym('Tpd_n','real');
Tpd_e = sym('Tpd_e','real');
Tpd_d = sym('Tpd_d','real');

% if (fabs(norm_v)<0.05) {
%     vbar_n=0.0;
%     vbar_e=0.0;
%     vbar_d=0.0;
% }
% else if (fabs(norm_v)<in[45]) {
%     vbar_n=((sin(2*norm_v/in[45]/3.14159265359)-1)+1)*n_dot;
%     vbar_e=((sin(2*norm_v/in[45]/3.14159265359)-1)+1)*e_dot;
%     vbar_d=((sin(2*norm_v/in[45]/3.14159265359)-1)+1)*d_dot;
% }
% else {
%     vbar_n=n_dot/norm_v;
%     vbar_e=e_dot/norm_v;
%     vbar_d=d_dot/norm_v;
% }

Jlon = 1/(1+exp(-k_t_d*(e_t_d-e_d_co))) + 1/(1+exp(k_t_d*(e_t_d+e_d_co)));
Jlat = 1/(1+exp(-k_t_ne*(e_t_ne-e_ne_co))) + 1/(1+exp(k_t_ne*(e_t_ne+e_ne_co)));

vbar_n_ref = Td_n * (1 - Jlat) + Tpd_n * Jlat;
vbar_e_ref = Td_e * (1 - Jlat) + Tpd_e * Jlat;
vbar_d_ref = Td_d * (1 - Jlon) + Tpd_d * Jlon;

e_vbar_1_n = vbar_n_ref - vbar_n;
e_vbar_1_e = vbar_e_ref - vbar_e;
e_vbar_1_d = vbar_d_ref - vbar_d;

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
y   = [ e_t_1_ne; e_t_1_d; e_vbar_1_n; e_vbar_1_e; e_vbar_1_d; Vsafe; p; q; r; a_soft ];
n_Y = length(y);

% ctrl output
z   = [ delta_T; phi_ref; theta_ref; delta_T; phi_ref; theta_ref ];
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

