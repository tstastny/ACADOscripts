/* PATH CALCULATIONS */ 
 
% path tangent unit vector  
tP_n_bar = cos(in(17)); 
tP_e_bar = sin(in(17)); 
 
% "closest" point on track 
tp_dot_br = tP_n_bar*(in(1)-in(13)) + tP_e_bar*(in(2)-in(14)); 
tp_dot_br_n = tp_dot_br*tP_n_bar; 
tp_dot_br_e = tp_dot_br*tP_e_bar; 
p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar; 
p_d = in(15) - p_lat*tan(in(16)); 
 
/* DIRECTIONAL GUIDANCE */ 
 
% lateral-directional error 
e_lat = ((in(1)-in(13))*tP_e_bar - (in(2)-in(14))*tP_n_bar); 
 
% ground speed 
v_c_gamma = in(9)*cos(in(4)); 
vG_n = v_c_gamma*cos(in(5)) + in(10); 
vG_e = v_c_gamma*sin(in(5)) + in(11); 
vG_d = -in(9)*sin(in(4)) + in(12); 
norm_vg_lat2 = vG_n*vG_n + vG_e*vG_e; 
norm_vg_lat = sqrt(norm_vg_lat2); 
 
% lateral-directional track-error boundary 
e_b_lat; 
if (norm_vg_lat > 1.0) { 
e_b_lat = norm_vg_lat*in(18);                                
} else { 
e_b_lat = 0.5*in(18)*(norm_vg_lat2 + 1.0); 
} 
 
% lateral-directional setpoint 
normalized_e_lat = e_lat/e_b_lat; 
chi_sp = in(17) + atan(M_2_PI * normalized_e_lat); 
 
% lateral-directional error 
chi_err = chi_sp - atan2(vG_e,vG_n); 
if (chi_err > M_PI) chi_err = chi_err - M_2_PI; 
if (chi_err < -M_PI) chi_err = chi_err + M_2_PI; 
 
/* LONGITUDINAL GUIDANCE */ 
 
% normalized track-error 
normalized_e_lat = fabs(normalized_e_lat); 
if (normalized_e_lat > 1.0) normalized_e_lat = 1.0; 
 
% smooth track proximity factor 
track_prox = cos(M_PI_2 * normalized_e_lat); 
track_prox = track_prox * track_prox; 
 
% path down velocity setpoint 
vP_d = in(16) * sqrt(norm_vg_lat2 + vg_d*vg_d) * track_prox  - in(12); 
 
% longitudinal velocity increment 
delta_vd; 
if (in(3) < 0.0) { 
delta_vd = VD_SINK + VD_EPS - vP_d; 
} 
else { 
delta_vd = VD_CLMB - VD_EPS - vP_d; 
} 
 
% longitudinal track-error boundary 
e_b_lon = in(19) * delta_vd; 
nomralized_e_lon = fabs(in(3)/e_b_lon); 
 
% longitudinal approach velocity 
vsp_d_app = TWO_OVER_PI * atan(M_2_PI * nomralized_e_lon) * delta_vd + vP_d; 
 
% down velocity setpoint (air-mass relative) 
vsp_d = vP_d + vsp_d_app; 
 
% flight path angle setpoint 
vsp_d_over_v = vsp_d/in(9); 
if (vsp_d_over_v > 1.0) vsp_d_over_v = 1.0; 
if (vsp_d_over_v < -1.0) vsp_d_over_v = -1.0; 
gamma_sp = asin(vsp_d_over_v); 
 
/* TERRAIN */ 
 
% lookup 2.5d grid 
int idx_q[4]; 
dh[2]; 
lookup_terrain_idx(in(1), in(2), in(21), in(22), idx_q, dh); 
 
% bi-linear interpolation 
h12 = (1-dh[0])*in[22+idx_q[0]] + dh[0]*in[22+idx_q[1]]; 
h34 = (1-dh[0])*in[22+idx_q[2]] + dh[0]*in[22+idx_q[3]]; 
h_terr = (1-dh[1])*h12 + dh[1]*h34; 
 
% soft constraint formulation 
one_minus_h_normalized = 1.0 + (in(3) + h_terr)/in(20); 
if (one_minus_h_normalized <= 0.0) one_minus_h_normalized = 0.0; 
 
% constraint priority 
sig_h = (one_minus_h_normalized > 1.0) ? 1.0 : cos(M_PI*one_minus_h_normalized)*0.5+0.5; 
 
% state output 
out(1) = sig_h*chi_err; 
out(2) = one_minus_h_normalized*one_minus_h_normalized; 
 
% control output 
out(3) = sig_h*gamma_sp - in(7);    % gamma ref 
out(4) = in(8);                     % phi ref 
out(5) = (in(7) - in(4))/1;         % gamma dot 
out(6) = (in(8) - in(6))/0.5;       % phi dot 
