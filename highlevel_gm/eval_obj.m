function [out,aux] = eval_obj(in)

% path tangent unit vector  
tP_n_bar = cos(in(17)); 
tP_e_bar = sin(in(17)); 
 
% "closest" point on track 
tp_dot_br = tP_n_bar*(in(1)-in(13)) + tP_e_bar*(in(2)-in(14)); 
tp_dot_br_n = tp_dot_br*tP_n_bar; 
tp_dot_br_e = tp_dot_br*tP_e_bar; 
p_n = in(13) + tp_dot_br_n; 
p_e = in(14) + tp_dot_br_e; 
p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar; 
p_d = in(15) - p_lat*tan(in(16)); 
 
% directional error 
v_c_gamma = in(9)*cos(in(4)); 
v_n = v_c_gamma*cos(in(5)); 
v_e = v_c_gamma*sin(in(5)); 
tp_dot_vg = tP_n_bar*(v_n+in(10)) + tP_e_bar*(v_e+in(11)); 
 
% terrain 
h_terr = lookup_terrain(in(1), in(2), in(19:end)); 
one_minus_h_normalized = 1.0 + (in(3) + h_terr)/in(18);
if (one_minus_h_normalized <= 0.0)
    one_minus_h_normalized = 0.0;
end

% state output 
out(1) = (in(2)-p_e)*cos(in(17)) - (in(1)-p_n)*sin(in(17)); 
out(2) = p_d - in(3); 
out(3) = tp_dot_vg*0.5+0.5; 
out(4) = one_minus_h_normalized*one_minus_h_normalized; 
 
% control output 
out(5) = in(7); % gamma ref 
out(6) = in(8); % mu ref 
out(7) = (in(7) - in(4))/1; % gamma dot 
out(8) = (in(8) - in(6))/0.7; % mu dot 

aux = h_terr;

function h_terr = lookup_terrain(pos_n, pos_e, terrain_data)

    IDX_CENTER_N = 31;
    IDX_CENTER_E = 31;
    LEN_IDX_N = 61;
    LEN_IDX_E = 61;
    DIS = 10.0;
    ONE_OVER_DIS = 0.1;
    DIS_OVER_2 = 5.0;

    rel_n = pos_n - terrain_data(IDX_CENTER_N * LEN_IDX_E + IDX_CENTER_E + 1);
    if (rel_n<0.0)
        rel_n = rel_n - DIS_OVER_2;
    else
        rel_n = rel_n + DIS_OVER_2;
    end
    delta_idx_n = rel_n * ONE_OVER_DIS;
    if delta_idx_n < 0
        delta_idx_n = ceil(delta_idx_n);
    else
        delta_idx_n = floor(delta_idx_n);
    end
    idx_n = IDX_CENTER_N + delta_idx_n; 
    if (idx_n >= LEN_IDX_N)
        idx_n = LEN_IDX_N-1; 
    elseif (idx_n < 0) 
        idx_n = 0; 
    end

    rel_e = pos_e - terrain_data(IDX_CENTER_N * LEN_IDX_E + IDX_CENTER_E + 1);
    if (rel_e<0.0)
        rel_e = rel_e - DIS_OVER_2;
    else
        rel_e = rel_e + DIS_OVER_2;
    end
    delta_idx_e = rel_e * ONE_OVER_DIS;
    if delta_idx_e < 0
        delta_idx_e = ceil(delta_idx_e);
    else
        delta_idx_e = floor(delta_idx_e);
    end
    idx_e = IDX_CENTER_E + delta_idx_e; 
    if (idx_e >= LEN_IDX_E)
        idx_e = LEN_IDX_E-1; 
    elseif (idx_e < 0) 
        idx_e = 0; 
    end

    h_terr = terrain_data(idx_n * LEN_IDX_E + idx_e + 1);
