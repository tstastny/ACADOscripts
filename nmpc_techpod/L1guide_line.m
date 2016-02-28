function [gamma_cmd,phi_cmd,etalon,etalat,p_travel] = L1guide_line(pp,V,aa,bb,...
    L1p_lon,L1d_lon,intg_lon,L1p_lat,L1d_lat,intg_lat,...
    gamma_cmd_min,gamma_cmd_max,phi_cmd_min,phi_cmd_max)

% velocities
V_lat = sqrt(sum(V(1:2).^2));
V_lon = sqrt(sum(V.^2));

% gravity
g = 9.81;

% calculate vector from waypoint a to b
aa_bb       = bb - aa;
normaa_bb   = norm(aa_bb);
if normaa_bb == 0
    aa_bb_unit  = aa_bb;
else
    aa_bb_unit  = aa_bb / normaa_bb;
end

% calculate closest point on line a->b
aa_pp       = pp - aa;

p_travel    = dot(aa_bb_unit, aa_pp);

dd          = aa + p_travel * aa_bb_unit;

% --- lateral plane ---

% calculate lateral track error
normdd_pp_lat   = norm(pp(1:2) - dd(1:2));

% calculate the L1 length required for the desired period
L1R_lat         = L1p_lat * L1d_lat / pi;
L1_lat          = L1R_lat * V_lat;

% check that L1 vector does not exceed reasonable bounds
L1min_lat       = normdd_pp_lat;
if L1_lat < L1min_lat, L1_lat = L1min_lat; end;

% calculate L1 vector
normaa_bb_lat       = norm(aa_bb(1:2));
if normaa_bb_lat == 0
    aa_bb_lat_unit	= aa_bb(1:2);
else
    aa_bb_lat_unit  = aa_bb(1:2) / normaa_bb_lat;
end
normdd_rr_lat	= sqrt(L1_lat^2 - normdd_pp_lat^2);
rr_lat          = aa_bb_lat_unit * normdd_rr_lat + dd(1:2);
LL_lat          = rr_lat - pp(1:2);

% lateral error angle
etalat = atan2(LL_lat(2), LL_lat(1)) - atan2(V(2),V(1));
if etalat>pi, etalat=etalat-2*pi; end;
if etalat<-pi, etalat=etalat+2*pi; end;

% --- longitudinal plane ---

% calculate longitudinal track error
aa_bb_lon_unit  = [sqrt(sum(aa_bb_unit(1:2).^2)), aa_bb_unit(3)];
pp_dd_d         = dd(3)-pp(3);
xtrackerr_lon   = pp_dd_d/aa_bb_lon_unit(1);

% calculate the L1 length required for the desired period
L1R_lon         = L1p_lon * L1d_lon / pi;
L1_lon          = L1R_lon * V_lon;

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
% etalon = atan(etalon);

% calculate the L1 gain (following [2]) */
KL1_lon     = 4 * L1d_lon * L1d_lon;
KL1_lat     = 4 * L1d_lat * L1d_lat;

% error angle PI control
etalonPI    = etalon*KL1_lon + intg_lon;
etalatPI    = etalat*KL1_lat + intg_lat;

% guidance commands lateral_accel = K_L1 * ground_speed / L1_ratio * sin(eta);
gamma_cmd   = atan(V_lon*etalonPI/L1R_lon/g);
phi_cmd     = atan(V_lat*etalatPI/L1R_lat/g);
% gamma_cmd   = gamma_cmd_max * 2/pi * atan(V_lon*etalonPI/L1R_lon/g / (gamma_cmd_max * 2/pi));
% phi_cmd     = phi_cmd_max * 2/pi * atan(V_lat*etalatPI/L1R_lat/g / (phi_cmd_max * 2/pi));

% saturation
if gamma_cmd > gamma_cmd_max
    gamma_cmd = gamma_cmd_max;
elseif gamma_cmd < gamma_cmd_min
    gamma_cmd = gamma_cmd_min;
end
if phi_cmd > phi_cmd_max
    phi_cmd = phi_cmd_max;
elseif phi_cmd < phi_cmd_min
    phi_cmd = phi_cmd_min;
end

