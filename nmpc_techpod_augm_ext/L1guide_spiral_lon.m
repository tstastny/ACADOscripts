function [gamma_cmd,etalon,p_travel,rr_travel] = L1guide_spiral_lon(pp,V,cc,R,ldir,xi0,gamsp,...
    L1p_lon,L1d_lon,intg_lon,...
    gamma_cmd_min,gamma_cmd_max)

% velocities
V_lat = sqrt(sum(V(1:2).^2));
V_lon = sqrt(sum(V.^2));

% gravity
g = 9.81;

% calculate the distance from the aircraft to the circle
cc_pp_n     = pp(1) - cc(1);
cc_pp_e     = pp(2) - cc(2);

% --- longitudinal plane ---

% spiral angular position
xi = atan2(cc_pp_e,cc_pp_n);
delta_xi = xi-xi0;
if ldir>0 && xi0>xi
    delta_xi = delta_xi + 2*pi;
elseif ldir<0 && xi>xi0
    delta_xi = delta_xi - 2*pi;
end

% root spiral height for current angle
dsxi0 = -delta_xi*ldir*R*tan(gamsp);

% round to nearest spiral leg
if gamsp==0
    d0 = dsxi0;
else
    d0 = round(max((dsxi0-pp(3))/2/pi/R/tan(gamsp),0))*-2*pi*R*tan(gamsp) + dsxi0;
end

% calculate longitudinal track error
aa_bb_lon_unit  = [cos(gamsp), -sin(gamsp)];
pp_dd_d         = d0 + cc(3) - pp(3);
xtrackerr_lon   = pp_dd_d/aa_bb_lon_unit(1);

% calculate the L1 length required for the desired period
L1R_lon         = L1p_lon * L1d_lon / pi;
L1_lon          = L1R_lon * V_lon;
% L1_lat          = L1_lon;

% check that L1 vector does not exceed reasonable bounds
L1min_lon       = abs(xtrackerr_lon);
if L1_lon < L1min_lon, L1_lon = L1min_lon; end;

% calculate L1 vector
normdd_rr_lon   = sqrt(L1_lon^2 - xtrackerr_lon^2);
rr_lon          = aa_bb_lon_unit * normdd_rr_lon + [0, pp_dd_d];
LL_lon          = rr_lon - [pp_dd_d * aa_bb_lon_unit(2)/aa_bb_lon_unit(1), 0];

% longitudinal error angle
etalon = atan2(V(3),V_lat) - atan2(LL_lon(2),LL_lon(1));
if etalon>pi, etalon=etalon-2*pi; end;
if etalon<-pi, etalon=etalon+2*pi; end;
etalon = atan(etalon);

% calculate the L1 gain (following [2]) */
KL1_lon     = 4 * L1d_lon * L1d_lon;

% error angle PI control
etalonPI    = etalon*KL1_lon + intg_lon;

% guidance commands lateral_accel = K_L1 * ground_speed / L1_ratio * sin(eta);
gamma_cmd   = atan2(V_lon*etalonPI,L1R_lon*g);

% saturation
if gamma_cmd > gamma_cmd_max
    gamma_cmd = gamma_cmd_max;
elseif gamma_cmd < gamma_cmd_min
    gamma_cmd = gamma_cmd_min;
end

% -------------------------------------
% normcc_pp2  = cc_pp_n^2 + cc_pp_e^2;
% normcc_pp   = sqrt(normcc_pp2);
% 
% pp_dd       = normcc_pp - R;

% % check that L1 vector does not exceed reasonable bounds
% L1min_lat   = abs(pp_dd);
% L1max_lat   = 2*R + pp_dd;
% if L1_lat > L1max_lat
%     L1_lat  = L1max_lat;
% elseif L1_lat < L1min_lat
%     L1_lat  = L1min_lat;
% end

% % calculate lateral components of L1 vector
% cosgamL1    = (L1_lat^2 + normcc_pp2 - R^2) / 2 / L1_lat / normcc_pp;
% if cosgamL1 > 1, cosgamL1 = 1; end;
% if cosgamL1 < -1, cosgamL1 = -1; end;
% gamL1   = acos(cosgamL1);
% 
% b_pp    = atan2(-cc_pp_e,-cc_pp_n);
% 
% b_L1    = b_pp - ldir * gamL1;
% if b_L1 > pi, b_L1 = b_L1 - 2*pi; end;
% if b_L1 < -pi, b_L1 = b_L1 + 2*pi; end;
% 
% delta_b_L1 = b_L1-xi;
% if ldir>0 && xi>b_L1
%     delta_b_L1 = delta_b_L1 + 2*pi;
% elseif ldir<0 && b_L1>xi
%     delta_b_L1 = delta_b_L1 - 2*pi;
% end

% calculate p travel
p_travel    = delta_xi;
% p_travel    = delta_xi + delta_b_L1;


