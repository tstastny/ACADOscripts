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
syms mu;            %   (roll angle)                [rad] 
syms gamma;         %   (pitch angle)               [rad]  
syms xi;            %   (yaw angle)                 [rad]

states  = [n,e,d,mu,gamma,xi];
n_X     = length(states);

assume(states,'real');
assumeAlso(gamma > -pi/2);
assumeAlso(gamma < pi/2);
assumeAlso(mu > -pi);
assumeAlso(mu < pi);

% CONTROLS ////////////////////////////////////////////////////////////////
syms mu_dot;        % (commanded bank angle)        [rad]
syms gamma_dot;     % (commanded fpa)               [rad]

ctrls   = [mu_dot,gamma_dot];
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

onlinedata  = [V,...
    pparam1,pparam2,pparam3,pparam4,pparam5,pparam6,pparam7,pparam8,pparam9,...
    wn,we,wd];
n_OD        = length(onlinedata);

assume(onlinedata,'real');

% /////////////////////////////////////////////////////////////////////////
% STATE DIFFERENTIALS /////////////////////////////////////////////////////
n_dot       = V * cos(gamma) * cos(xi) + wn;
e_dot       = V * cos(gamma) * sin(xi) + we;
d_dot       = -V * sin(gamma) + wd;
xi_dot      = 9.81 * tan(mu) / V;

% /////////////////////////////////////////////////////////////////////////
% AUGMENTED GUIDANpparam3 LOGIC ////////////////////////////////////////////////

% calculate closest point on loiter circle
cpn = n - pparam2;
cpe = e - pparam3;
norm_cp = sqrt( cpn^2 + cpe^2 );
cpn_u = cpn / (norm_cp + 0.01);
cpe_u = cpe / (norm_cp + 0.01);
cdn = pparam5 * cpn_u;
cde = pparam5 * cpe_u;
dn = n + cdn - cpn;
de = e + cde - cpe;
dd = pparam4;

% calculate tangent
vdn_expr = pparam6*-cpe_u;
vde_expr = pparam6*cpn_u;

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
vdn	= sym('vdn','real');
tracked_expr(idx_tracked_expr, :) = [vdn, vdn_expr];

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
vde	= sym('vde','real');
tracked_expr(idx_tracked_expr, :) = [vde, vde_expr];     

vdd = 0;

% track position error
pdn = dn - n;
pde = de - e;
pdd = dd - d;
cx = vde*pdd - pde*vdd;
cy = -(vdn*pdd - pdn*vdd);
cz = vdn*pde - pdn*vde;
et = sqrt( cx^2 + cy^2 + cz^2 );

% xi/gamma desired
Gamma_d = -asin(vdd);
chi_d_expr = atan2(vde,vdn);

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
FaR_expr = chi_d_expr;
findandreplace_atan2s
for i = idx_atan2s_last:idx_atan2s-1
    idx_tracked_expr = idx_tracked_expr + 1;
    tracked_expr(idx_tracked_expr,:) = [atan2s(i),atan2_args(i,1)];
    idx_tracked_expr = idx_tracked_expr + 1;
    tracked_expr(idx_tracked_expr,:) = [0,atan2_args(i,2)];
end
idx_atan2s_last = idx_atan2s;                    

idx_tracked_expr = idx_tracked_expr + 1;
syms chi_d;
tracked_expr(idx_tracked_expr, :) = [chi_d, FaR_expr];


% ground referenpparam3d angles
V_g = sqrt(n_dot^2 + e_dot^2 + d_dot^2);

% IF ( d_dot/V_g > 1, 1...elseif < -1, -1...elseif V_g == 0, 0 )
Gamma = -asin(d_dot/V_g);
chi_expr = atan2(e_dot,n_dot);

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
FaR_expr = chi_expr;
findandreplace_atan2s
for i = idx_atan2s_last:idx_atan2s-1
    idx_tracked_expr = idx_tracked_expr + 1;
    tracked_expr(idx_tracked_expr,:) = [atan2s(i),atan2_args(i,1)];
    idx_tracked_expr = idx_tracked_expr + 1;
    tracked_expr(idx_tracked_expr,:) = [0,atan2_args(i,2)];
end
idx_atan2s_last = idx_atan2s;                    

idx_tracked_expr = idx_tracked_expr + 1;
syms chi;
tracked_expr(idx_tracked_expr, :) = [chi, FaR_expr];

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
z   = [ Gamma_d-Gamma; chi_d-chi; mu_dot; gamma_dot ];
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

