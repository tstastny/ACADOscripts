clear; clc;

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

X = [n,e,d,V,gamma,xi,phi,theta,p,q,r,delta_T];

% auxillary states
alpha = theta - gamma; % assume no beta

% CONTROLS ////////////////////////////////////////////////////////////////
syms u_T;   % (throttle input)              [~]
syms phi_ref;	% (roll angle reference)        [rad]
syms theta_ref;	% (pitch angle reference)       [rad]

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

% /////////////////////////////////////////////////////////////////////////
% STATE DIFFERENTIALS /////////////////////////////////////////////////////

% position differentials

n_dot = V * cos(xi) * cos(gamma) + wn;
e_dot = V * sin(xi) * cos(gamma) + we;
d_dot = -V * sin(gamma) + wd;

norm_v = sqrt(n_dot^2 + e_dot^2 + d_dot^2);

vbar_n=n_dot/norm_v;
vbar_e=e_dot/norm_v;
vbar_d=d_dot/norm_v;

% velocity frame differentials

g = 9.81;
m = 2.65;
rho = 1.225;
S = 0.39;
qbarS = 0.5*rho*V^2*S;

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

T = (cT1*delta_T+cT2*delta_T^2+cT3*delta_T^3)/V/cos(alpha); % should probably have a check for alpha=90.. but really shouldnt happen
D = qbarS*cD;
L = qbarS*cL;

V_dot = (T*cos(alpha)-D)/m-g*sin(gamma);
gamma_dot = ((T*sin(alpha)+L)*cos(phi) - m*g*cos(gamma))/m/V;
xi_dot = (T*sin(alpha)+L)*sin(phi)/V/m/cos(gamma); % same as with alpha

% attitude differentials
phi_dot = p;
theta_dot = q*cos(phi)-r*sin(phi);

% body rate differentials

lm = lp * p + lr * r + leR * (phi_ref-phi);
Mm = V^2 * (M0 + Ma * alpha + Mq * q + MeP * (theta_ref-theta));
Nm = Nr * r + NR * phi + NRR * phi_ref;

p_dot = lm;
q_dot = Mm;
r_dot = Nm;

% throttle delay

delta_T_dot = (u_T - delta_T) / tauT;

% /////////////////////////////////////////////////////////////////////////
% OBJECTIVES //////////////////////////////////////////////////////////////

% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% variable definitions
pparam_aa_n = pparam2;
pparam_aa_e = pparam3;
pparam_aa_d = pparam4;
pparam_bb_n = pparam5;
pparam_bb_e = pparam6;
pparam_bb_d = pparam7;

% calculate vector from waypoint a to b
abn = pparam_bb_n - pparam_aa_n;
abe = pparam_bb_e - pparam_aa_e;
abd = pparam_bb_d - pparam_aa_d;
norm_ab = sqrt(abn*abn + abe*abe + abd*abd);

% calculate tangent
Td_n = abn / norm_ab;
Td_e = abe / norm_ab;
Td_d = abd / norm_ab;

% dot product
dot_abunit_ap = Td_n*(n - pparam_aa_n) + Td_e*(e - pparam_aa_e) + Td_d*(d - pparam_aa_d);

% point on track
d_n = pparam_aa_n + dot_abunit_ap * Td_n;
d_e = pparam_aa_e + dot_abunit_ap * Td_e;
d_d = pparam_aa_d + dot_abunit_ap * Td_d;

% end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% track position error

pd_n = d_n - n;
pd_e = d_e - e;
pd_d = d_d - d;

cz = Td_n * pd_e - pd_n * Td_e;

e_t_ne = cz;
e_t_d = pd_d;

e_t_1_ne = sin(e_t_ne/e_ne_co*pi/2);
e_t_1_d = sin(e_t_d/e_d_co*pi/2);

% cost bounds + extra guidance

Tpd_n = 0.0;
Tpd_e = 0.0;
Tpd_d = 0.0;

Jlon = 1/(1+exp(-k_t_d*(e_t_d-e_d_co))) + 1/(1+exp(k_t_d*(e_t_d+e_d_co)));
Jlat = 1/(1+exp(-k_t_ne*(e_t_ne-e_ne_co))) + 1/(1+exp(k_t_ne*(e_t_ne+e_ne_co)));

vbar_n_ref = Td_n * (1 - Jlat) + Tpd_n * Jlat;
vbar_e_ref = Td_e * (1 - Jlat) + Tpd_e * Jlat;
vbar_d_ref = Td_d * (1 - Jlon) + Tpd_d * Jlon;

e_vbar_1_n = vbar_n_ref - vbar_n;
e_vbar_1_e = vbar_e_ref - vbar_e;
e_vbar_1_d = vbar_d_ref - vbar_d;

% soft constraints

a_soft = 0.0;

% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% OPTIMAL CONTROL PROBLEM /////////////////////////////////////////////////

% state output
h   = [ e_t_1_ne; e_t_1_d; e_vbar_1_n; e_vbar_1_e; e_vbar_1_d; V; p; q; r; ];
syms e_t_1_ne_ e_t_1_d_ e_vbar_1_n_ e_vbar_1_e_ e_vbar_1_d_ V_ p_ q_ r_
h_ = [e_t_1_ne_; e_t_1_d_; e_vbar_1_n_; e_vbar_1_e_; e_vbar_1_d_; V_; p_; q_; r_];

hh(1) = subs(h(1),h(2:end),h_(2:end));

% jacobian
% dh_dx = jacobian(h,);

% rewrite




