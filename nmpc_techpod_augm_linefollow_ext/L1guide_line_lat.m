function [phi_cmd,etalat,p_travel] = L1guide_line_lat(pp,V,aa,bb,...
    L1p_lat,L1d_lat,intg_lat,...
    phi_cmd_min,phi_cmd_max)

% velocities
V_lat = sqrt(sum(V(1:2).^2));

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

