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
syms mu;            %   (bank angle)                [rad] 
syms gamma;         %   (flight path angle)         [rad]
syms xi;            %   (yaw angle)                 [rad]
syms mu_dot;        %   (bank angle rate)           [rad/s]
syms gamma_dot;     %   (flight path angle rate)    [rad/s]

states  = [n,e,d,mu,gamma,xi,mu_dot,gamma_dot];
n_X     = length(states);

assume(states,'real');
assumeAlso(gamma > -pi/2);
assumeAlso(gamma < pi/2);
assumeAlso(mu > -pi);
assumeAlso(mu < pi);

% CONTROLS ////////////////////////////////////////////////////////////////
syms mu_cmd;        % (commanded bank angle)        [rad]
syms gamma_cmd;     % (commanded fpa)               [rad]

ctrls   = [mu_cmd,gamma_cmd];
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
syms wn;
syms we;
syms wd;
syms k_mu;
syms k_gamma;
syms k_mu_dot;
syms k_gamma_dot;

onlinedata  = [V,...
    pparam1,pparam2,pparam3,pparam4,pparam5,pparam6,pparam7,pparam8,pparam9,...
    wn,we,wd,...
    k_mu,k_gamma,k_mu_dot,k_gamma_dot];
n_OD        = length(onlinedata);

assume(onlinedata,'real');

% /////////////////////////////////////////////////////////////////////////
% STATE DIFFERENTIALS /////////////////////////////////////////////////////
n_dot           = V * cos(gamma) * cos(xi) + wn;
e_dot           = V * cos(gamma) * sin(xi) + we;
d_dot           = -V * sin(gamma) + wd;
mu_dot_dot      = ( (mu_cmd - mu) * k_mu - mu_dot ) * k_mu_dot;
gamma_dot_dot   = ( (gamma_cmd - gamma) * k_gamma - gamma_dot ) * k_gamma_dot;
xi_dot          = 9.81 * tan(mu) / V;

% /////////////////////////////////////////////////////////////////////////
% AUGMENTED GUIDANCE LOGIC ////////////////////////////////////////////////

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

% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% OPTIMAL CONTROL PROBLEM /////////////////////////////////////////////////

% ode - right-hand side
for i = 1:n_X
    eval(['frhs(i,1) = ',char(states(i)),'_dot;']);
end

% state output
y   = [ et; mu_dot; gamma_dot ];
n_Y = length(y);

% ctrl output
z   = [ e_Gamma; e_chi; mu_cmd; gamma_cmd ];
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

