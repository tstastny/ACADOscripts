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
syms mu_dot_cmd;        % (commanded bank angle rate)        [rad]
syms gamma_dot_cmd;     % (commanded fpa rate)               [rad]

ctrls   = [mu_dot_cmd,gamma_dot_cmd];
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
syms tau_mu_dot;
syms tau_gamma_dot;

onlinedata  = [V,...
    pparam1,pparam2,pparam3,pparam4,pparam5,pparam6,pparam7,pparam8,pparam9,...
    wn,we,wd,...
    tau_mu_dot,tau_gamma_dot];
n_OD        = length(onlinedata);

assume(onlinedata,'real');

% /////////////////////////////////////////////////////////////////////////
% STATE DIFFERENTIALS /////////////////////////////////////////////////////
n_dot           = V * cos(gamma) * cos(xi) + wn;
e_dot           = V * cos(gamma) * sin(xi) + we;
d_dot           = -V * sin(gamma) + wd;
mu_dot_dot      = (mu_dot_cmd - mu_dot) / tau_mu_dot;
gamma_dot_dot   = (gamma_dot_cmd - gamma_dot) / tau_gamma_dot;
xi_dot          = 9.81 * tan(mu) / V;

% /////////////////////////////////////////////////////////////////////////
% AUGMENTED GUIDANCE LOGIC ////////////////////////////////////////////////

% calculate closest point on loiter circle
cp_n = n - pparam2;
cp_e = e - pparam3;
norm_cp = sqrt( cp_n^2 + cp_e^2 );
cp_n_unit_expr = cp_n / (norm_cp + 0.01);
cp_e_unit_expr = cp_e / (norm_cp + 0.01);

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
cp_n_unit	= sym('cp_n_unit','real');
tracked_expr(idx_tracked_expr, :) = [cp_n_unit, cp_n_unit_expr];

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
cp_e_unit	= sym('cp_e_unit','real');
tracked_expr(idx_tracked_expr, :) = [cp_e_unit, cp_e_unit_expr];

cd_n = pparam5 * cp_n_unit;
cd_e = pparam5 * cp_e_unit;
d_n = n + cd_n - cp_n;
d_e = e + cd_e - cp_e;

% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% % // variable definitions
% % const double pparam_cc_d = XX;
% % const double pparam_R = XX;
% % const double pparam_ldir = XX;
% % const double pparam_gam_sp = XX;
% % const double pparam_xi0 = XX;
% % const double pparam_dxi = XX;
% % 
% % // spiral angular position: [0,2*pi)
% % const double xi_sp = atan2(cp_e_unit, cp_n_unit);
% % double delta_xi = xi_sp-pparam_xi0;
% % 
% % if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {
% %     
% %     delta_xi = delta_xi + 6.28318530718;
% % 
% % } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {
% %     
% %     delta_xi = delta_xi - 6.28318530718;
% % 
% % }
% % 
% % // closest point on nearest spiral leg and tangent down component
% % double d_d;
% % double Td_d;
% % double Gamma_d;
% % 
% % if (fabs(pparam_gam_sp) < 0.001) {
% % 
% %     d_d = pparam_cc_d;
% %     Td_d = 0.0;
% %     Gamma_d = 0.0;
% % 
% % } else {
% % 
% %     const double Rtangam = pparam_R * tan(pparam_gam_sp);
% % 
% %     // spiral height delta for current angle
% %     const double delta_d_xi = -delta_xi * pparam_ldir * Rtangam;
% % 
% %     // end spiral altitude change
% %     const double delta_d_sp_end = -pparam_dxi * Rtangam;
% % 
% %     // nearest spiral leg
% %     double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
% %     const double delta_d_end_k  = round( (delta_d_sp_end - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
% % 
% %     // check
% %     if (delta_d_k * pparam_gam_sp > 0.0) { //NOTE: gam is actually being used for its sign, but writing a sign operator doesnt make a difference here
% % 
% %         delta_d_k = 0.0;
% %     
% %     } else if (fabs(delta_d_k) > fabs(delta_d_end_k) ) {
% %     
% %         delta_d_k = (delta_d_k < 0.0) ? -fabs(delta_d_end_k) : fabs(delta_d_end_k);
% %     
% %     }
% % 
% %     // closest point on nearest spiral leg
% %     const double delta_d_sp = delta_d_k + delta_d_xi;
% %     d_d = pparam_cc_d + delta_d_sp;
% %     Td_d = -asin(pparam_gam_sp);
% %     Gamma_d = pparam_gam_sp;
% % }

% end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% manual output !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
d_d = sym('d_d','real');
Td_d = sym('Td_d','real');
Gamma_d = sym('Gamma_d','real');
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% calculate tangent
Td_n_expr = pparam6*-cp_e_unit;
Td_e_expr = pparam6*cp_n_unit;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
Td_n = sym('Td_n','real');
tracked_expr(idx_tracked_expr, :) = [Td_n, Td_n_expr];

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
Td_e = sym('Td_e','real');
tracked_expr(idx_tracked_expr, :) = [Td_e, Td_e_expr];     

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
% double V_g = sqrt(t10*t10+t12*t12+t14*t14);
% if (V_g < 0.01) V_g = 0.01;
% 
% double sin_d_dot_V_g = t14*1.0/V_g;
% if (sin_d_dot_V_g > 1.0) {
%     sin_d_dot_V_g = 1.0;
% } else if (sin_d_dot_V_g < -1.0) {
%     sin_d_dot_V_g = -1.0;
% }

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
y   = [ et ];
n_Y = length(y);

% ctrl output
z   = [ e_Gamma; e_chi; mu_dot; gamma_dot; mu_dot_cmd; gamma_dot_cmd ];
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

