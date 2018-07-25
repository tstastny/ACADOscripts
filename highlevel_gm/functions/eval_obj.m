function [out,aux] = eval_obj(in)

M_PI_2 = 1.570796326794897;
M_PI = 3.141592653589793;
M_2_PI = 6.283185307179586;
TWO_OVER_PI = 0.636619772367581;

% longitudinal guidance constants
VD_SINK = 1.5;
VD_CLMB = -3.5;
VD_EPS = 0.01;

% PATH CALCULATIONS */ 
 
% path tangent unit vector  
tP_n_bar = cos(in(17)); 
tP_e_bar = sin(in(17)); 
 
% "closest" point on track 
tp_dot_br = tP_n_bar*(in(1)-in(13)) + tP_e_bar*(in(2)-in(14)); 
tp_dot_br_n = tp_dot_br*tP_n_bar; 
tp_dot_br_e = tp_dot_br*tP_e_bar; 
p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar; 
p_d = in(15) - p_lat*tan(in(16)); 
 
% DIRECTIONAL GUIDANCE */ 
 
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
if (norm_vg_lat > 1.0)
    e_b_lat = norm_vg_lat*in(18);                                
else
    e_b_lat = 0.5*in(18)*(norm_vg_lat2 + 1.0); 
end
 
% lateral-directional setpoint 
normalized_e_lat = e_lat/e_b_lat; 
chi_sp = in(17) + atan(M_2_PI * normalized_e_lat); 
 
% lateral-directional error 
chi_err = chi_sp - atan2(vG_e,vG_n); 
if (chi_err > M_PI)
    chi_err = chi_err - M_2_PI; 
end
if (chi_err < -M_PI)
    chi_err = chi_err + M_2_PI; 
end

% LONGITUDINAL GUIDANCE */ 

% longitudinal track-error
e_lon = p_d-in(3);
    
% normalized lateral-directional track-error 
normalized_e_lat = abs(normalized_e_lat); 
if (normalized_e_lat > 1.0)
    normalized_e_lat = 1.0; 
end
 
% smooth track proximity factor 
track_prox = cos(M_PI_2 * normalized_e_lat); 
track_prox = track_prox * track_prox; 
 
% path down velocity setpoint 
vP_d = in(16) * sqrt(norm_vg_lat2 + vG_d*vG_d) * track_prox  - in(12); 
 
% longitudinal velocity increment 
if (in(3) < 0.0) 
    delta_vd = VD_SINK + VD_EPS - vP_d;  
else
    delta_vd = VD_CLMB - VD_EPS - vP_d; 
end 
 
% longitudinal track-error boundary 
e_b_lon = in(19) * delta_vd; 
nomralized_e_lon = abs(e_lon/e_b_lon); 
 
% longitudinal approach velocity 
vsp_d_app = TWO_OVER_PI * atan(M_2_PI * nomralized_e_lon) * delta_vd + vP_d; 
 
% down velocity setpoint (air-mass relative) 
vsp_d = vP_d + vsp_d_app; 
 
% flight path angle setpoint 
vsp_d_over_v = vsp_d/in(9); 
if (vsp_d_over_v > 1.0)
    vsp_d_over_v = 1.0; 
end
if (vsp_d_over_v < -1.0)
    vsp_d_over_v = -1.0; 
end
gamma_sp = -asin(vsp_d_over_v); 
 
% TERRAIN */ 
 
% lookup 2.5d grid  
[idx_q, dh] = lookup_terrain_idx(in(1), in(2), in(21), in(22)); 
 
% bi-linear interpolation 
h12 = (1-dh(1))*in(23+idx_q(1)) + dh(1)*in(23+idx_q(2)); 
h34 = (1-dh(1))*in(23+idx_q(3)) + dh(1)*in(23+idx_q(4)); 
h_terr = (1-dh(2))*h12 + dh(2)*h34; 
 
% soft constraint formulation 
one_minus_h_normalized = 1.0 + (in(3) + h_terr)/in(20); 
if (one_minus_h_normalized <= 0.0)
    one_minus_h_normalized = 0.0; 
end
 
% constraint priority 
if (one_minus_h_normalized > 1.0)
    sig_h = 1.0;
else
    sig_h =  cos(M_PI*one_minus_h_normalized)*0.5+0.5;
end

% state output 
out(1) = sig_h*chi_err; 
out(2) = sig_h*(gamma_sp - in(4));
out(3) = one_minus_h_normalized*one_minus_h_normalized; 
 
% control output 
out(4) = in(7) - sig_h*gamma_sp;    % gamma ref 
out(5) = in(8);                     % phi ref 
out(6) = (in(7) - in(4))/1;         % gamma dot 
out(7) = (in(8) - in(6))/0.5;       % phi dot 

aux = [e_lat, e_lon, h_terr, gamma_sp];

    function [idx_q, dh] = lookup_terrain_idx( pos_n, pos_e, pos_n_origin, pos_e_origin)
        
        LEN_IDX_N = 61;
        LEN_IDX_E = 61;
        LEN_IDX_N_1 = 60;
        LEN_IDX_E_1 = 60;
        ONE_DIS = 0.1;

        % relative position / indices
        rel_n = pos_n - pos_n_origin;
        rel_n_bar = rel_n * ONE_DIS;
        idx_n = floor(rel_n_bar);
        if (idx_n < 0)
            idx_n = 0;
        elseif (idx_n > LEN_IDX_N_1)
            idx_n = LEN_IDX_N_1;
        end
        rel_e = pos_e - pos_e_origin;
        rel_e_bar = rel_e * ONE_DIS;
        idx_e = floor(rel_e_bar);
        if (idx_e < 0)
            idx_e = 0;
        elseif (idx_e > LEN_IDX_E_1)
            idx_e = LEN_IDX_E_1;
        end

        % neighbor orientation / interpolation weights
        delta_n = rel_n_bar-idx_n;
        if (delta_n<0.5) 
            down = -1;
            dh_n = 0.5 + delta_n;
        else
            down = 0;
            dh_n = delta_n - 0.5;
        end
        delta_e = rel_e_bar-idx_e;
        if (delta_e<0.5) 
            left = -1;
            dh_e = 0.5 + delta_e;
        else 
            left = 0;
            dh_e = delta_e - 0.5;
        end

        % neighbor origin (down,left)
        q1_n = idx_n + down;
        q1_e = idx_e + left;

        % neighbors (north)
        if (q1_n >= LEN_IDX_N_1) 
            q_n(1) = LEN_IDX_N_1;
            q_n(2) = LEN_IDX_N_1;
            q_n(3) = LEN_IDX_N_1;
            q_n(4) = LEN_IDX_N_1;
        else 
            q_n(1) = q1_n;
            q_n(2) = q1_n + 1;
            q_n(3) = q1_n;
            q_n(4) = q1_n + 1;
        end
        % neighbors (east)
        if (q1_e >= LEN_IDX_N_1)
            q_e(1) = LEN_IDX_E_1;
            q_e(2) = LEN_IDX_E_1;
            q_e(3) = LEN_IDX_E_1;
            q_e(4) = LEN_IDX_E_1;
        else
            q_e(1) = q1_e;
            q_e(2) = q1_e;
            q_e(3) = q1_e + 1;
            q_e(4) = q1_e + 1;
        end

        % neighbors row-major indices
        idx_q(1) = q_n(1)*LEN_IDX_E + q_e(1);
        idx_q(2) = q_n(2)*LEN_IDX_E + q_e(2);
        idx_q(3) = q_n(3)*LEN_IDX_E + q_e(3);
        idx_q(4) = q_n(4)*LEN_IDX_E + q_e(4);

        % interpolation weights
        dh(1) = dh_n;
        dh(2) = dh_e;

    end
end

