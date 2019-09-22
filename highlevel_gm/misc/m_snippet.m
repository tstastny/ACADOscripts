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
%n_p = in(9); 
 
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
 
%control augmented attitude time constants and gains 
%tau_phi = in(21); 
%tau_theta = in(22); 
%k_phi = in(23); 
%k_theta = in(24); 
 
% soft angle of attack constraints 
k_aoa = in(25); 
aoa_m = in(26); 
aoa_p = in(27); 
 
% terrain 
k_h = in(28); 
delta_h = in(29); 
terr_local_origin_n = in(30); 
terr_local_origin_e = in(31); 
%terrain_data = in[31]; 
IDX_TERR_DATA = 31; 
 
% INTERMEDIATE CALCULATIONS */ 
 
% ground speed 
v_cos_gamma = v*cos(gamma); 
vG_n = v_cos_gamma*cos(xi) + w_n; 
vG_e = v_cos_gamma*sin(xi) + w_e; 
vG_d = -v*sin(gamma) + w_d; 
 
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
e_lat = ((r_n-b_n)*tP_e_bar - (r_e-b_e)*tP_n_bar); 
e_lon = p_d - r_d; 
 
% velocity error 
v_cos_gamma = v*cos(Gamma_p); 
vP_n = v_cos_gamma*cos(chi_p); 
vP_e = v_cos_gamma*sin(chi_p); 
vP_d = -v*sin(Gamma_p); 
e_v_n = vP_n - vG_n; 
e_v_e = vP_e - vG_e; 
e_v_d = vP_d - vG_d; 
 
% SOFT CONSTRAINTS */ 
 
% angle of attack 
aoa_mid = (aoa_p - aoa_m) / 2.0; 
aoa = theta - gamma; 
if (aoa > aoa_mid) aoa = 2.0*aoa_mid - aoa; 
sig_aoa = 1.0/(1.0 + exp(k_aoa*(aoa - aoa_m))); 
 
% TERRAIN */ 
 
% lookup 2.5d grid 
idx_q[4]; 
dh[2]; 
lookup_terrain_idx(r_n, r_e, terr_local_origin_n, terr_local_origin_e, idx_q, dh); 
 
% bi-linear interpolation 
h12 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[0]] + dh[0]*in[IDX_TERR_DATA+idx_q[1]]; 
h34 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[2]] + dh[0]*in[IDX_TERR_DATA+idx_q[3]]; 
h_terr = (1-dh[1])*h12 + dh[1]*h34; 
 
% soft constraint 
sig_h = 1.0 / (1 + exp(k_h*((-r_d - h_terr) - delta_h))); 
 
% state output 
out(1) = e_lat; 
out(2) = e_lon; 
out(3) = e_v_n; 
out(4) = e_v_e; 
out(5) = e_v_d; 
out(6) = v; 
out(7) = sig_aoa; 
out(8) = sig_h; 
 
% control output 
out(9) = u_T; 
out(10) = phi_ref; 
out(11) = theta_ref; 
