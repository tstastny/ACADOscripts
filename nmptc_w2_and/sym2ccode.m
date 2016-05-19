% /////////////////////////////////////////////////////////////////////////
% Dubins Aircraft Model Setup /////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////

tracked_expr = sym('tracked_expr',0);
idx_tracked_expr = 0;

atan2_args = sym('atan2_args',0);
atan2s = sym('atan2s',0);
idx_atan2s = 1;
idx_atan2s_last = idx_atan2s;

% STATES //////////////////////////////////////////////////////////////////
syms n;            % (northing)
syms e;            % (easting)
syms d;            % (down)
syms mu;           % (bank angle)
syms gamma;        % (flight path angle)
syms xi;           % (heading angle)
syms mu_dot;       % (bank angle rate)
syms gamma_dot;    % (flight path angle rate)
syms x_w2_mu;      % (w2 bank state)
syms x_w2_gamma;   % (w2 gamma state)
syms intg_et;      % (integral of track error)
syms intg_e_Gamma; % (integral of GR flight path angle error)
syms intg_e_chi;   % (integral of course error)

states  = [n,e,d,mu,gamma,xi,mu_dot,gamma_dot,x_w2_mu,x_w2_gamma,intg_et,intg_e_Gamma,intg_e_chi];
n_X     = length(states);

assume(states,'real');
% assumeAlso(gamma > -pi/2);
% assumeAlso(gamma < pi/2);
% assumeAlso(mu > -pi);
% assumeAlso(mu < pi);

% CONTROLS ////////////////////////////////////////////////////////////////
syms mu_r;        % (commanded bank angle)        [rad]
syms gamma_r;     % (commanded fpa)               [rad]

ctrls   = [mu_r,gamma_r];
n_U     = length(ctrls);

assume(ctrls,'real');

% ONLINE DATA /////////////////////////////////////////////////////////////
syms V;
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
syms wn;
syms we;
syms wd;
syms omega_n_mu;
syms zeta_mu;
syms omega_n_gamma;
syms zeta_gamma;
syms aw2;
syms bw2;
syms cw2;
syms dw2;

onlinedata  = [V,...
    pparam1,pparam2,pparam3,pparam4,pparam5,pparam6,pparam7,pparam8,pparam9,...
    pparam1_next,pparam2_next,pparam3_next,pparam4_next,pparam5_next,pparam6_next,pparam7_next,pparam8_next,pparam9_next,...
    wn,we,wd,...
    omega_n_mu,zeta_mu,omega_n_gamma,zeta_gamma,...
    aw2,bw2,cw2,dw2];
n_OD        = length(onlinedata);

assume(onlinedata,'real');

% /////////////////////////////////////////////////////////////////////////
% STATE DIFFERENTIALS /////////////////////////////////////////////////////
n_dot           = V * cos(gamma) * cos(xi) + wn;
e_dot           = V * cos(gamma) * sin(xi) + we;
d_dot           = -V * sin(gamma) + wd;
% mu_dot
% gamma_dot
xi_dot          = 9.81 * tan(mu) / V;
mu_dot_dot      = omega_n_mu * (omega_n_mu * (mu_r - mu) - 2 * zeta_mu * mu_dot);
gamma_dot_dot   = omega_n_gamma * (omega_n_gamma * (gamma_r - gamma) - 2 * zeta_gamma * gamma_dot);
x_w2_mu_dot     = aw2 * x_w2_mu + bw2 * mu_r;
x_w2_gamma_dot  = aw2 * x_w2_gamma + bw2 * gamma_r;
% intg_et_dot
% intg_e_Gamma_dot
% intg_e_chi_dot
% /////////////////////////////////////////////////////////////////////////
% AUGMENTED GUIDANCE LOGIC ////////////////////////////////////////////////

z_w2_mu = cw2 * x_w2_mu + dw2 * mu_r;
z_w2_gamma = cw2 * x_w2_gamma + dw2 * gamma_r;

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
Gamma_d = sym('Gamma_d','real');
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% track position error
pd_n = d_n - n;
pd_e = d_e - e;
pd_d = d_d - d;
cx = Td_e * pd_d - pd_e * Td_d;
cy = -(Td_n * pd_d - pd_n * Td_d);
cz = Td_n * pd_e - pd_n * Td_e;
et = sqrt( cx^2 + cy^2 + cz^2 );

% ground speed
V_g = sqrt(n_dot^2 + e_dot^2 + d_dot^2);

sin_d_dot_V_g_expr = d_dot/V_g;
% double V_g = sqrt(...);
% if (V_g < 0.01) V_g = 0.01;
% 
% double sin_d_dot_V_g = t14*1.0/V_g;
% if (sin_d_dot_V_g > 1.0) sin_d_dot_V_g = 1.0;
% if (sin_d_dot_V_g < -1.0) sin_d_dot_V_g = -1.0;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
sin_d_dot_V_g = sym('sin_d_dot_V_g','real');
tracked_expr(idx_tracked_expr, :) = [sin_d_dot_V_g, sin_d_dot_V_g_expr];     

Gamma = -asin(sin_d_dot_V_g);

% Gamma error
e_Gamma = Gamma_d - Gamma;

% chi error
e_chi_expr = atan2(Td_e,Td_n) - atan2(e_dot,n_dot);
% if (e_chi>3.14159265359) e_chi = e_chi - 6.28318530718;
% if (e_chi<-3.14159265359) e_chi = e_chi + 6.28318530718;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
FaR_expr = e_chi_expr;
findandreplace_atan2s
for i = idx_atan2s_last:idx_atan2s-1
    idx_tracked_expr = idx_tracked_expr + 1;
    tracked_expr(idx_tracked_expr,:) = [atan2s(i),atan2_args(i,1)];
    idx_tracked_expr = idx_tracked_expr + 1;
    tracked_expr(idx_tracked_expr,:) = [0,atan2_args(i,2)];
end
idx_atan2s_last = idx_atan2s;                    

idx_tracked_expr = idx_tracked_expr + 1;
syms e_chi;
tracked_expr(idx_tracked_expr, :) = [e_chi, FaR_expr];

% more state diffs
intg_et_dot = et;
intg_e_Gamma_dot = e_Gamma;
intg_e_chi_dot = e_chi;

% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% OPTIMAL CONTROL PROBLEM /////////////////////////////////////////////////

% ode - right-hand side
for i = 1:n_X
    eval(['frhs(i,1) = ',char(states(i)),'_dot;']);
end

% state output
y   = [ mu; gamma; mu_dot; gamma_dot; et; e_Gamma; e_chi; intg_et; intg_e_Gamma; intg_e_chi; mu_r; gamma_r; z_w2_mu; z_w2_gamma ];
n_Y = length(y);

% ctrl output
z   = [];
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
tracked_expr = subs( tracked_expr, [states ctrls onlinedata], ins );
frhs = subs( frhs, [states ctrls onlinedata], ins );
objectives = subs( objectives, [states ctrls onlinedata], ins );
y = subs( y, [states ctrls onlinedata], ins );

% generate optimized c code
ccode([tracked_expr(:,2); frhs],'file','ext_rhs_.c');
ccode([tracked_expr(:,2); objectives],'file','ext_lsq_obj_.c')
ccode([tracked_expr(:,2);  y],'file','ext_lsq_obj_N_.c');

