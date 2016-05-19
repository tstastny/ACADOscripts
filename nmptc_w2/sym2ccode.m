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
syms n;             %   (north local position)      [m]
syms e;             %   (east local position)       [m]
syms d;             %   (down local position)       [m]
syms xi;            %   (yaw angle)                 [rad]
syms intg_et;       % integral of error
syms x_w2_mu;
syms x_w2_gamma;

states  = [n,e,d,xi,intg_et,x_w2_mu,x_w2_gamma];
n_X     = length(states);

assume(states,'real');
% assumeAlso(gamma > -pi/2);
% assumeAlso(gamma < pi/2);
% assumeAlso(mu > -pi);
% assumeAlso(mu < pi);

% CONTROLS ////////////////////////////////////////////////////////////////
syms mu;        % (commanded bank angle)        [rad]
syms gamma;     % (commanded fpa)               [rad]

ctrls   = [mu,gamma];
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
syms aw2;
syms bw2;
syms cw2;
syms dw2;

onlinedata  = [V,...
    pparam1,pparam2,pparam3,pparam4,pparam5,pparam6,pparam7,pparam8,pparam9,...
    pparam1_next,pparam2_next,pparam3_next,pparam4_next,pparam5_next,pparam6_next,pparam7_next,pparam8_next,pparam9_next,...
    wn,we,wd,...
    aw2,bw2,cw2,dw2];
n_OD        = length(onlinedata);

assume(onlinedata,'real');

% /////////////////////////////////////////////////////////////////////////
% STATE DIFFERENTIALS /////////////////////////////////////////////////////
n_dot           = V * cos(gamma) * cos(xi) + wn;
e_dot           = V * cos(gamma) * sin(xi) + we;
d_dot           = -V * sin(gamma) + wd;
xi_dot          = 9.81 * tan(mu) / V;
%intg_et_dot -->see below
x_w2_mu_dot     = aw2 * x_w2_mu + bw2 * mu;
x_w2_gamma_dot     = aw2 * x_w2_gamma + bw2 * gamma;

% /////////////////////////////////////////////////////////////////////////
% AUGMENTED GUIDANCE LOGIC ////////////////////////////////////////////////

z_w2_mu = cw2 * x_w2_mu + dw2 * mu;
z_w2_gamma = cw2 * x_w2_gamma + dw2 * gamma;

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

% integral of track error
intg_et_dot = et;

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

% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% OPTIMAL CONTROL PROBLEM /////////////////////////////////////////////////

% ode - right-hand side
for i = 1:n_X
    eval(['frhs(i,1) = ',char(states(i)),'_dot;']);
end

% state output
y   = [ et; intg_et ];
n_Y = length(y);

% ctrl output
z   = [ e_Gamma; e_chi; z_w2_mu; z_w2_gamma ];
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

