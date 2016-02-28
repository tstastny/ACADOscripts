% /////////////////////////////////////////////////////////////////////////
% Techpod Model Setup /////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////

tracked_expr = sym('tracked_expr',0);
idx_tracked_expr = 0;

atan2_args = sym('atan2_args',0);
atan2s = sym('atan2s',0);
idx_atan2s = 1;
idx_atan2s_last = idx_atan2s;

% STATES //////////////////////////////////////////////////////////////////
syms u;             %   (x-body velocity)           [m/s]
syms v;             %   (y-body velocity)           [rad]
syms w;             %   (z-body velocity)           [rad]
syms p;             %   (roll rate)                 [rad/s]
syms q;             %   (pitch rate)                [rad/s]   
syms r;             %   (yaw rate)                  [rad/s]   
syms phi;           %   (roll angle)                [rad] 
syms theta;         %   (pitch angle)               [rad]  
syms psi;           %   (yaw angle)                 [rad]
syms n;             %   (north local position)      [m]
syms e;             %   (east local position)       [m]
syms d;             %   (down local position)       [m]             

syms intg_lon;      %   (longitudinal integrator)   [rad*s]
syms intg_lat;      %   (lateral integrator)        [rad*s]
syms intg_V;        %   (airspeed integrator)       [m]            

syms x_w2_uT;       %   (throtte w2)                [~]
syms x_w2_uE;       %   (elevator w2)               [~]
syms x_w2_uA;       %   (aileron w2)                [~]
syms x_w2_uR;       %   (rudder w2)                 [~]
syms x_w2_sv;       %   (slack w2)                  [~]

states  = [u,v,w,p,q,r,phi,theta,psi,n,e,d,intg_lon,intg_lat,intg_V,x_w2_uT,x_w2_uE,x_w2_uA,x_w2_uR,x_w2_sv];
n_X     = length(states);

assume(states,'real');
assumeAlso(theta > -pi/2);
assumeAlso(theta < pi/2);
assumeAlso(phi > -pi);
assumeAlso(phi < pi);

% CONTROLS ////////////////////////////////////////////////////////////////
syms uT;     %   (throttle setting)     [0,1]
syms uE;     %   (elevator deflection)	[rad]
syms uA;     %   (aileron deflection)	[rad]
syms uR;     %   (rudder deflection)	[rad]
syms sv;     %   (slack variable)       [~]

ctrls   = [uT,uE,uA,uR,sv];
n_U     = length(ctrls);

assume(ctrls,'real');

% ONLINE DATA /////////////////////////////////////////////////////////////
syms L1p_lon;
syms L1d_lon;
syms L1p_lat;
syms L1d_lat;

syms kilon;
syms kilat;
syms kiV;

syms V_cmd;

syms pparam1;   %   type    type
syms pparam2;   %   aa_n    cc_n
syms pparam3;   %   aa_e    cc_e
syms pparam4;   %   aa_d    cc_d
syms pparam5;   %   bb_n    R
syms pparam6;   %   bb_e    dir
syms pparam7;   %   bb_d    gam
syms pparam8;   %   --      xi0
syms pparam9;   %   --      dxi

syms Aw2;
syms Bw2;
syms Cw2;
syms Dw2;

syms wn;
syms we;
syms wd;

onlinedata  = [L1p_lon,L1d_lon,L1p_lat,L1d_lat,...
    kilon,kilat,kiV,...
    V_cmd,...
    pparam1,pparam2,pparam3,pparam4,pparam5,pparam6,pparam7,pparam8,pparam9,...
    Aw2,Bw2,Cw2,Dw2,...
    wn,we,wd];
n_OD        = length(onlinedata);

assume(onlinedata,'real');

% /////////////////////////////////////////////////////////////////////////
% MODEL CONSTRUCTION //////////////////////////////////////////////////////

% intermediate states
V       = sqrt( u^2 + v^2 + w^2 );
beta    = asin( v / V );
alpha   = atan( w / u );

% geometry
S_wing      = 0.47;     % wing surface              [m^2]
b_wing      = 2.59;     % wingspan                  [m]
c_chord     = 0.180;    % mean chord length         [m]
i_thrust    = 0*pi/180;	% thrust incidence angle    [rad] -->NOTE: i is defined from body x in positive pitch (up)

% mass / inertia
mass    = 2.65;         % total mass of airplane    [kg]
Ixx     = 0.1512*1.1;	% inertias                  [kg m^2]
Iyy     = 0.2785*1.4;	% inertias                  [kg m^2]
Izz     = 0.3745*1.4;	% inertias                  [kg m^2]
Ixz     = 0.0755;       % inertias                  [kg m^2]

% environment
g       = 9.81;     % gravitational acceleration    [m/s^2]
rho_air = 1.18;     % air density

% wind to body transform
ca      = cos(alpha);
sa      = sin(alpha);

% non-dimensionalization
p_hat	= p*b_wing /(2*V);
q_hat	= q*c_chord /(2*V);
r_hat	= r*b_wing /(2*V);
q_bar_S = 1/2*rho_air*V^2*S_wing;

% load aircraft parameters
load parameters_2015.09.10_1956_6DoF.mat
for i = 1:length(parameters)
    eval([parameters(i).Name,' = ',num2str(parameters(i).Value),';'])
end

% stability axis aerodynamic force coefficients
cD      = cD0 + cDa*alpha + cDa2*alpha^2;
cY_s    = cYb*beta;
cL      = cL0 + cLa*alpha + cLa2*alpha^2 + cLa3*alpha^3;

% body frame aerodynamic forces
X       = q_bar_S * (-cD * ca + cL * sa);
Y       = q_bar_S * cY_s;
Z       = q_bar_S * (-cD * sa - cL * ca);

% stability axis aerodynamic moment coefficients
cl_s    = clb*beta + clp*p_hat + clr*r_hat + clda*uA;
cm_s    = cm0 + cma*alpha + cmq*q_hat + cmde*uE;
cn_s    = cnb*beta + cnp*p_hat + cnr*r_hat + cndr*uR;

% body frame aerodynamic moments
Lm      = q_bar_S * b_wing * (cl_s * ca - cn_s * sa);
Mm      = q_bar_S * c_chord * cm_s;
Nm      = q_bar_S * b_wing * (cl_s * sa + cn_s * sa);
     
% thrust force
T   = cT1*uT;                                                              % neglecting 2nd order term

% intermediate state differentials
u_dot       = r*v - q*w - g*sin(theta) + (X+T*cos(i_thrust))/mass;
v_dot       = p*w - r*u + g*sin(phi)*cos(theta) + Y/mass;
w_dot       = q*u - p*v + g*cos(phi)*cos(theta) + (Z+T*sin(i_thrust))/mass;

I1          = Ixz*(Iyy-Ixx-Izz);
I2          = (Ixx*Izz-Ixz^2);
p_dot       = (Izz*Lm+Ixz*Nm-(I1*p+(Ixz^2+Izz*(Izz-Iyy))*r)*q)/I2;
q_dot       = (Mm-(Ixx-Izz)*p*r-Ixz*(p^2-r^2))/Iyy;                        % neglecting thrusting moments
r_dot       = (Ixz*Lm+Ixx*Nm+(I1*r+(Ixz^2+Ixx*(Ixx-Iyy))*p)*q)/I2;

phi_dot     = p + (q*sin(phi) + r*cos(phi))*tan(theta);
theta_dot   = q*cos(phi) - r*sin(phi);
psi_dot     = (q*sin(phi) + r*cos(phi))/cos(theta);

n_dot       = u*cos(theta)*cos(psi)+v*(sin(phi)*sin(theta)*cos(psi)-...
    cos(phi)*sin(psi))+w*(sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi)) + wn;
e_dot       = u*cos(theta)*sin(psi)+v*(cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi))+...
    w*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)) + we;
d_dot       = -u*sin(theta)+v*sin(phi)*cos(theta)+w*cos(phi)*cos(theta) + wd;

% /////////////////////////////////////////////////////////////////////////
% AUGMENTED GUIDANCE LOGIC ////////////////////////////////////////////////

% velocities
V_lat_expr = sqrt( n_dot^2 + e_dot^2 );
V_lon_expr = sqrt( n_dot^2 + e_dot^2 + d_dot^2 );

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
V_lat	= sym('V_lat','real');
tracked_expr(idx_tracked_expr, :) = [V_lat, V_lat_expr];
% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
idx_tracked_expr = idx_tracked_expr + 1;
V_lon	= sym('V_lon','real');
tracked_expr(idx_tracked_expr, :) = [V_lon, V_lon_expr];                   


% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% % type = param1;
% % 
% % if param1 == 0 % line
% %     
% %     aa_n    = pparam2;
% %     aa_e    = pparam3;
% %     aa_d    = pparam4;
% %     bb_n    = pparam5;
% %     bb_e    = pparam6;
% %     bb_d    = pparam7;
% % 
% %     % calculate vector from waypoint a to b
% %     aa_bb_n     = bb_n - aa_n;
% %     aa_bb_e     = bb_e - aa_e;
% %     aa_bb_d     = bb_d - aa_d;
% % 
% %     normaa_bb       = sqrt( aa_bb_n^2 + aa_bb_e^2 + aa_bb_d^2 );
% %     aa_bb_unit_n    = aa_bb_n / normaa_bb;
% %     aa_bb_unit_e    = aa_bb_e / normaa_bb;
% %     aa_bb_unit_d    = aa_bb_d / normaa_bb;
% % 
% %     % calculate closest point on line a->b
% %     aa_pp_n     = n - aa_n;
% %     aa_pp_e     = e - aa_e;
% %     aa_pp_d     = d - aa_d;
% % 
% %     pp_proj     = aa_bb_unit_n*aa_pp_n + aa_bb_unit_e*aa_pp_e + aa_bb_unit_d*aa_pp_d;
% % 
% %     dd_n        = aa_n + pp_proj * aa_bb_unit_n;
% %     dd_e        = aa_e + pp_proj * aa_bb_unit_e;
% %     dd_d        = aa_d + pp_proj * aa_bb_unit_d;
% % 
% %     % longitudinal / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% % 
% %     % calculate longitudinal track error
% %     aa_bb_lon_unit_ne   = sqrt( aa_bb_unit_n^2 + aa_bb_unit_e^2 );
% %     aa_bb_lon_unit_d    = aa_bb_unit_d;
% % 
% %     pp_dd_d             = dd_d - d;
% %     xtrackerr_lon       = pp_dd_d / aa_bb_lon_unit_ne;
% % 
% %     % calculate the L1 length required for the desired period
% %     L1R_lon         = L1p_lon * L1d_lon / pi;
% %     L1_lon          = L1R_lon * V_lon;
% % 
% %     % check that L1 vector does not exceed reasonable bounds
% %     L1min_lon       = abs(xtrackerr_lon);
% %     if L1_lon < L1min_lon, L1_lon = L1min_lon; end;
% % 
% %     % calculate L1 vector
% %     normdd_rr_lon   = sqrt(L1_lon^2 - xtrackerr_lon^2);
% %     rr_lon_ne       = aa_bb_lon_unit_ne * normdd_rr_lon;
% %     rr_lon_d        = aa_bb_lon_unit_d * normdd_rr_lon + pp_dd_d;
% %     LL_lon_ne       = rr_lon_ne - pp_dd_d * aa_bb_lon_unit_d / aa_bb_lon_unit_ne;
% %     LL_lon_d        = rr_lon_d;
% % 
% %     % lateral / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% % 
% %     normaa_bb_lat       = sqrt( aa_bb_n^2 + aa_bb_e^2 );
% %     aa_bb_lat_unit_n    = aa_bb_n / normaa_bb_lat;
% %     aa_bb_lat_unit_e    = aa_bb_e / normaa_bb_lat;
% % 
% %     % calculate closest point on line a->b
% %     aa_pp_n     = n - aa_n;
% %     aa_pp_e     = e - aa_e;
% % 
% %     pp_proj_lat     = aa_bb_lat_unit_n*aa_pp_n + aa_bb_lat_unit_e*aa_pp_e;
% % 
% %     dd_lat_n        = aa_n + pp_proj_lat * aa_bb_lat_unit_n;
% %     dd_lat_e        = aa_e + pp_proj_lat * aa_bb_lat_unit_e;
% % 
% %     % calculate lateral track error
% %     normdd_pp_lat   = sqrt( (n - dd_lat_n)^2 + (e - dd_lat_e)^2 );
% % 
% %     % calculate the L1 length required for the desired period
% %     L1R_lat         = L1p_lat * L1d_lat / pi;
% %     L1_lat          = L1R_lat * V_lat;
% % 
% %     % check that L1 vector does not exceed reasonable bounds
% %     L1min_lat       = normdd_pp_lat;
% %     if L1_lat < L1min_lat, L1_lat = L1min_lat; end;
% % 
% %     % calculate L1 vector
% %     normdd_rr_lat	= sqrt(L1_lat^2 - normdd_pp_lat^2);
% %     rr_lat_n        = aa_bb_lat_unit_n * normdd_rr_lat + dd_lat_n;
% %     rr_lat_e        = aa_bb_lat_unit_e * normdd_rr_lat + dd_lat_e;
% %     LL_lat_n        = rr_lat_n - n;
% %     LL_lat_e        = rr_lat_e - e;
% % 
% % elseif param1 == 1 % spiral
% %     
% %     cc_n    = pparam2;
% %     cc_e    = pparam3;
% %     cc_d    = pparam4;
% %     R       = pparam5;
% %     dir     = pparam6;
% %     gam     = pparam7;
% %     xi0     = pparam8;
% %     dxi     = pparam9;
% % 
% % 	% lateral / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% %     
% %     % calculate the distance from the aircraft to the circle
% %     cc_pp_n     = n - cc_n;
% %     cc_pp_e     = e - cc_e;
% %     
% %     if abs(cc_pp_n) < 0.1 && abs(cc_pp_e) < 0.1 % cannot be in center of circle
% %         cc_pp_n = 0.1;
% %         cc_pp_e = 0.1;
% %     end
% % 
% %     normcc_pp2  = cc_pp_n^2 + cc_pp_e^2;
% %     normcc_pp   = sqrt(normcc_pp2);
% % 
% %     pp_dd       = normcc_pp - R;
% % 
% %     % check circle tracking feasibility, calculate L1 ratio
% %     R08_V       = R / V_lat * 0.5;
% %     L1R_lat     = min(L1d_lat * L1p_lat / pi, R08_V);
% % 
% %     % calculate the L1 length required for the desired period
% %     L1_lat      = L1R_lat * V_lat;
% % 
% %     % check that L1 vector does not exceed reasonable bounds
% %     L1min_lat   = abs(pp_dd);
% %     L1max_lat   = 2*R + pp_dd;
% %     if L1_lat > L1max_lat
% %         L1_lat  = L1max_lat;
% %     elseif L1_lat < L1min_lat
% %         L1_lat  = L1min_lat;
% %     end
% % 
% %     % calculate lateral components of L1 vector
% %     cosgamL1    = (L1_lat^2 + normcc_pp2 - R^2) / 2 / L1_lat / normcc_pp;
% %     if cosgamL1 > 1, cosgamL1 = 1; end;
% %     if cosgamL1 < -1, cosgamL1 = -1; end;
% %     gamL1   = acos(cosgamL1);
% % 
% %     b_pp    = atan2(-cc_pp_e,-cc_pp_n);
% % 
% %     b_L1    = b_pp - dir * gamL1;
% %     if b_L1 > pi, b_L1 = b_L1 - 2*pi; end;
% %     if b_L1 < -pi, b_L1 = b_L1 + 2*pi; end;
% % 
% %     LL_lat_n    = L1_lat * cos(b_L1);
% %     LL_lat_e    = L1_lat * sin(b_L1);
% % 
% %     % longitudinal / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% % 
% %     % spiral angular position: [0,2*pi)
% %     xi = atan2(cc_pp_e,cc_pp_n);
% %     delta_xi = xi-xi0;
% %     if dir>0 && xi0>xi
% %         delta_xi = delta_xi + 2*pi;
% %     elseif dir<0 && xi>xi0
% %         delta_xi = delta_xi - 2*pi;
% %     end
% % 
% %     % closest point on nearest spiral leg
% %     if gam < 0.001
% %         dd_d_sp = cc_d;
% %     else
% %         % spiral height delta for current angle
% %         delta_d_xi = -delta_xi*dir*R*tan(gam);
% % 
% %         % end spiral altitude change
% %         delta_d_sp_end  = -dxi*R*tan(gam);
% % 
% %         % nearest spiral leg
% %         delta_d_k = round( (d - (cc_d + delta_d_xi)) / (2*pi*R*tan(gam)) ) * 2*pi*R*tan(gam);
% %         delta_d_end_k = round( (delta_d_sp_end - (cc_d + delta_d_xi)) / (2*pi*R*tan(gam)) ) * 2*pi*R*tan(gam);
% % 
% %         % check
% %         if delta_d_k*sign(gam) > 0
% %             delta_d_k = 0;
% %         elseif abs(delta_d_k)-abs(delta_d_end_k) > 0
% %             delta_d_k = sign(delta_d_k) * min(abs(delta_d_k), abs(delta_d_end_k));
% %         end
% % 
% %         % closest point on nearest spiral leg
% %         delta_d_sp = delta_d_k + delta_d_xi;
% %         dd_d_sp = cc_d + delta_d_sp;
% %     end
% % 
% %     % calculate longitudinal track error
% %     aa_bb_lon_unit_ne   = cos(gam);
% %     aa_bb_lon_unit_d    = -sin(gam);
% %     pp_dd_d             = d0 + cc_d - d;
% %     xtrackerr_lon       = pp_dd_d/aa_bb_lon_unit_ne;
% % 
% %     % calculate the L1 length required for the desired period
% %     L1R_lon         = L1p_lon * L1d_lon / pi;
% %     L1_lon          = L1R_lon * V_lon;
% % 
% %     % check that L1 vector does not exceed reasonable bounds
% %     L1min_lon       = abs(xtrackerr_lon);
% %     if L1_lon < L1min_lon, L1_lon = L1min_lon; end;
% % 
% %     % calculate L1 vector
% %     normdd_rr_lon   = sqrt(L1_lon^2 - xtrackerr_lon^2);
% %     rr_lon_ne       = aa_bb_lon_unit_ne * normdd_rr_lon;
% %     rr_lon_d        = aa_bb_lon_unit_d * normdd_rr_lon + pp_dd_d;
% %     LL_lon_ne       = rr_lon_ne - pp_dd_d * aa_bb_lon_unit_d/aa_bb_lon_unit_ne;
% %     LL_lon_d        = rr_lon_d;
% %     
% % else
% %     LL_lon_ne = 1;
% %     LL_lon_d = 0;
% %     LL_lat_n = 1;
% %     LL_lat_e = 0;
% % end

% end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% manual outputs
LL_lon_ne	= sym('LL_lon_ne','real');
LL_lon_d    = sym('LL_lon_d','real');
LL_lat_n    = sym('LL_lat_n','real');
LL_lat_e    = sym('LL_lat_e','real');
L1R_lon     = sym('L1R_lon','real');
L1R_lat     = sym('L1R_lat','real');

% error angle
etalon_expr     = atan2(d_dot, V_lat) - atan2(LL_lon_d, LL_lon_ne);
% if etalon>pi, etalon=etalon-2*pi; end;
% if etalon<-pi, etalon=etalon+2*pi; end;
etalat_expr     = atan2(LL_lat_e, LL_lat_n) - atan2(e_dot, n_dot);
% if etalat>pi, etalat=etalat-2*pi; end;
% if etalat<-pi, etalat=etalat+2*pi; end; % does not change jacobian

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
FaR_expr = etalon_expr;
findandreplace_atan2s
for i = idx_atan2s_last:idx_atan2s-1
    idx_tracked_expr = idx_tracked_expr + 1;
    tracked_expr(idx_tracked_expr,:) = [atan2s(i),atan2_args(i,1)];
    idx_tracked_expr = idx_tracked_expr + 1;
    tracked_expr(idx_tracked_expr,:) = [0,atan2_args(i,2)];
end
idx_atan2s_last = idx_atan2s;                    

idx_tracked_expr = idx_tracked_expr + 1;
syms etalon;
tracked_expr(idx_tracked_expr, :) = [etalon, FaR_expr];

% | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | TRACK VARIABLE 
FaR_expr = etalat_expr;
findandreplace_atan2s
for i = idx_atan2s_last:idx_atan2s-1
    idx_tracked_expr = idx_tracked_expr + 1;
    tracked_expr(idx_tracked_expr,:) = [atan2s(i),atan2_args(i,1)];
    idx_tracked_expr = idx_tracked_expr + 1;
    tracked_expr(idx_tracked_expr,:) = [0,atan2_args(i,2)];
end
idx_atan2s_last = idx_atan2s;                    

idx_tracked_expr = idx_tracked_expr + 1;
syms etalat;
tracked_expr(idx_tracked_expr, :) = [etalat, FaR_expr];

% calculate the L1 gain (following [2]) */
KL1_lon     = 4 * L1d_lon * L1d_lon;
KL1_lat     = 4 * L1d_lat * L1d_lat;

% error angle PI control
etalonPI    = etalon * KL1_lon + intg_lon;
etalatPI    = etalat*KL1_lat + intg_lat;

% guidance commands accel = K_L1 * ground_speed / L1_ratio * sin(eta);
theta_cmd   = atan(V_lon*etalonPI/(L1R_lon*g));
phi_cmd     = atan(V_lat*etalatPI/(L1R_lat*g));

% error output
e_V     = V_cmd - V;
e_theta = theta_cmd - theta;
e_phi   = phi_cmd - phi;

% /////////////////////////////////////////////////////////////////////////
% MIXED SENSITIVITY LOOP SHAPING //////////////////////////////////////////

z_w2_uT = Cw2*x_w2_uT + Dw2*uT;
z_w2_uE = Cw2*x_w2_uE + Dw2*uE;
z_w2_uA = Cw2*x_w2_uA + Dw2*uA;
z_w2_uR = Cw2*x_w2_uR + Dw2*uR;
z_w2_sv = Cw2*x_w2_sv + Dw2*sv;

% /////////////////////////////////////////////////////////////////////////
% REMAINING DIFFERENTIALS /////////////////////////////////////////////////

intg_lon_dot    = etalon * kilon;
intg_lat_dot    = etalat * kilat;
intg_V_dot      = e_V * kiV;

x_w2_uT_dot     = Aw2*x_w2_uT + Bw2*uT;
x_w2_uE_dot     = Aw2*x_w2_uE + Bw2*uE;
x_w2_uA_dot     = Aw2*x_w2_uA + Bw2*uA;
x_w2_uR_dot     = Aw2*x_w2_uR + Bw2*uR;
x_w2_sv_dot     = Aw2*x_w2_sv + Bw2*sv;

% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% OPTIMAL CONTROL PROBLEM /////////////////////////////////////////////////

% ode - right-hand side
for i = 1:n_X
    eval(['frhs(i,1) = ',char(states(i)),'_dot;']);
end

% state output
y   = [ e_V + intg_V; e_theta; e_phi; beta; p; q; r ];
n_Y = length(y);

% ctrl output
z   = [ z_w2_uT; z_w2_uE; z_w2_uA; z_w2_uR; z_w2_sv ];
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
