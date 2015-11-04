function [phi_cmd,etalat,p_travel] = L1guide_spiral_lat(pp,V,cc,R,ldir,xi0,gamsp,...
    L1p_lat,L1d_lat,intg_lat,...
    phi_cmd_min,phi_cmd_max)

% velocities
V_lat = sqrt(sum(V(1:2).^2));

% gravity
g = 9.81;

% --- lateral plane ---

% calculate the distance from the aircraft to the circle
cc_pp_n     = pp(1) - cc(1);
cc_pp_e     = pp(2) - cc(2);

normcc_pp2  = cc_pp_n^2 + cc_pp_e^2;
normcc_pp   = sqrt(normcc_pp2);

pp_dd       = normcc_pp - R;

% check circle tracking feasibility, calculate L1 ratio
R08_V       = R / V_lat * 0.5;
L1R_lat     = min(L1d_lat * L1p_lat / pi, R08_V);
    
% calculate the L1 length required for the desired period
L1_lat      = L1R_lat * V_lat;

% check that L1 vector does not exceed reasonable bounds
L1min_lat   = abs(pp_dd);
L1max_lat   = 2*R + pp_dd;
if L1_lat > L1max_lat
    L1_lat  = L1max_lat;
elseif L1_lat < L1min_lat
    L1_lat  = L1min_lat;
end

% calculate lateral components of L1 vector
cosgamL1    = (L1_lat^2 + normcc_pp2 - R^2) / 2 / L1_lat / normcc_pp;
if cosgamL1 > 1, cosgamL1 = 1; end;
if cosgamL1 < -1, cosgamL1 = -1; end;
gamL1   = acos(cosgamL1);

b_pp    = atan2(-cc_pp_e,-cc_pp_n);

b_L1    = b_pp - ldir * gamL1;
if b_L1 > pi, b_L1 = b_L1 - 2*pi; end;
if b_L1 < -pi, b_L1 = b_L1 + 2*pi; end;

LL_lat(1)   = L1_lat * cos(b_L1);
LL_lat(2)   = L1_lat * sin(b_L1);

% lateral error angle
etalat = atan2(LL_lat(2), LL_lat(1)) - atan2(V(2),V(1));
if etalat>pi, etalat=etalat-2*pi; end;
if etalat<-pi, etalat=etalat+2*pi; end;

% spiral angular position
xi = atan2(cc_pp_e,cc_pp_n);
delta_xi = xi-xi0;
if ldir>0 && xi0>xi
    delta_xi = delta_xi + 2*pi;
elseif ldir<0 && xi>xi0
    delta_xi = delta_xi - 2*pi;
end

% calculate the L1 gain (following [2]) */
KL1_lat     = 4 * L1d_lat * L1d_lat;

% error angle PI control
etalatPI    = etalat*KL1_lat + intg_lat;

% guidance commands lateral_accel = K_L1 * ground_speed / L1_ratio * sin(eta);
phi_cmd     = atan2(V_lat*etalatPI,L1R_lat*g);

% saturation
if phi_cmd > phi_cmd_max
    phi_cmd = phi_cmd_max;
elseif phi_cmd < phi_cmd_min
    phi_cmd = phi_cmd_min;
end

% calculate p travel
p_travel    = delta_xi;

