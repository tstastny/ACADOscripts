function [out,aux] = eval_obj(in)

% DEFINE INPUTS -- this is just simply easier to read.. */ 
 
% states */ 
r_n = in(1); 
r_e = in(2); 
r_d = in(3); 
v = in(4); 
gamma = in(5); 
xi = in(6); 
phi = in(7); 
theta = in(8); 
n_p = in(9); 
 
% controls */ 
u_T = in(10); 
phi_ref = in(11); 
theta_ref = in(12); 
 
% online data */ 
 
% disturbances 
w_n = in(13); 
w_e = in(14); 
w_d = in(15); 
 
% path reference 
b_n = in(16); 
b_e = in(17); 
b_d = in(18); 
Gamma_p = in(19); 
chi_p = in(20); 

% guidance
T_b_lat = in(21);
T_b_lon = in(22);
 
%control augmented attitude time constants and gains 
%tau_phi = in(23); 
%tau_theta = in(24); 
%k_phi = in(25); 
%k_theta = in(26); 
 
% soft angle of attack constraints 
delta_aoa = in(27); 
aoa_m = in(28); 
aoa_p = in(29); 
 
% terrain 
delta_h = in(30); 
terr_local_origin_n = in(31); 
terr_local_origin_e = in(32); 
terr_dis = in(33);
%terrain_data = in[33]; 
IDX_TERR_DATA = 33+1; 
 
% INTERMEDIATE CALCULATIONS */ 
 
% ground speed 
v_cos_gamma = v*cos(gamma); 
vG_n = v_cos_gamma*cos(xi) + w_n; 
vG_e = v_cos_gamma*sin(xi) + w_e; 
vG_d = -v*sin(gamma) + w_d;
vG_norm = sqrt(vG_n*vG_n + vG_e*vG_e + vG_d*vG_d);
 
% PATH FOLLOWING */ 
 
% path tangent unit vector  
tP_n_bar = cos(chi_p); 
tP_e_bar = sin(chi_p); 
 
% "closest" poon track 
tp_dot_br = tP_n_bar*(r_n-b_n) + tP_e_bar*(r_e-b_e); 
tp_dot_br_n = tp_dot_br*tP_n_bar; 
tp_dot_br_e = tp_dot_br*tP_e_bar; 
p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar; 
p_d = b_d - p_lat*tan(Gamma_p); 
 
% position error 
e_lat = (r_n-b_n)*tP_e_bar - (r_e-b_e)*tP_n_bar; 
e_lon = p_d - r_d; 
 
% lateral-directional track error boundary
e_b_lat = T_b_lat * sqrt(vG_n*vG_n + vG_e*vG_e);

% course approach angle
chi_app = -atan(pi/2*e_lat/e_b_lat);

% longitudinal track error boundary
if (abs(vG_d) < 1.0)
    e_b_lon = T_b_lon * 0.5 * (1.0 + vG_d*vG_d);
else
    e_b_lon = T_b_lon * abs(vG_d);
end

% flight path approach angle
Gamma_app = 0.3/(pi/2) * atan(pi/2*e_lon/e_b_lon); % XXX: MAGIC NUMBER

% ground velocity setpoint
v_cos_gamma = vG_norm*cos(Gamma_p + Gamma_app);
vP_n = v_cos_gamma*cos(chi_p + chi_app);
vP_e = v_cos_gamma*sin(chi_p + chi_app);
vP_d = -vG_norm*sin(Gamma_p + Gamma_app);

% velocity error
e_v_n = vP_n - vG_n; 
e_v_e = vP_e - vG_e; 
e_v_d = vP_d - vG_d;
 
% SOFT CONSTRAINTS */ 
 
% angle of attack 
aoa = theta - gamma;
sig_aoa = 0.0;
if (aoa > aoa_p - delta_aoa) 
    sig_aoa = abs(aoa - aoa_p + delta_aoa) / delta_aoa;
    sig_aoa = sig_aoa * sig_aoa * sig_aoa;
end
if (aoa < aoa_m + delta_aoa)
    sig_aoa = abs(aoa - aoa_m - delta_aoa) / delta_aoa;
    sig_aoa = sig_aoa * sig_aoa * sig_aoa;
end
 
% TERRAIN */ 
 
% lookup 2.5d grid  
[idx_q, dh] = lookup_terrain_idx(r_n, r_e, terr_local_origin_n, terr_local_origin_e, terr_dis); 
 
% bi-linear interpolation 
h12 = (1-dh(1))*in(IDX_TERR_DATA+idx_q(1)) + dh(1)*in(IDX_TERR_DATA+idx_q(2)); 
h34 = (1-dh(1))*in(IDX_TERR_DATA+idx_q(3)) + dh(1)*in(IDX_TERR_DATA+idx_q(4)); 
h_terr = (1-dh(2))*h12 + dh(2)*h34; 

% soft constraint 
sig_h = 0.0;
if (-r_d < h_terr + delta_h)
    sig_h = abs(-r_d - h_terr - delta_h) / delta_h;
    sig_h = sig_h * sig_h * sig_h;
end

% prioritization
prio_aoa = 1;%1 - min(sig_aoa, 1.0);
prio_h = 1 - min(sig_h, 1.0);
prio_aoa_h = prio_aoa * prio_h;

% state output 
out(1) = e_lat * prio_aoa_h; 
out(2) = e_lon * prio_aoa_h;
out(3) = e_v_n * prio_aoa_h;
out(4) = e_v_e * prio_aoa_h;
out(5) = e_v_d * prio_aoa_h;
out(6) = v; 
out(7) = sig_aoa; 
out(8) = sig_h * prio_aoa; 

% control output
out(9) = u_T;
out(10) = phi_ref;
out(11) = theta_ref;

aux = [h_terr, e_lat, e_lon, e_v_n, e_v_e, e_v_d, sig_aoa, sig_h, prio_aoa, prio_h, prio_aoa_h, Gamma_app, chi_app, Gamma_p, chi_p];

end