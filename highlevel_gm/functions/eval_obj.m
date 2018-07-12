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
idx_terr = lookup_terrain_idx(in(1), in(2), in(19), in(20));
h_terr = in(21+idx_terr);
one_minus_h_normalized = 1.0 + (in(3) + h_terr)/in(18);
if (one_minus_h_normalized <= 0.0), one_minus_h_normalized = 0.0; end;  

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

function idx_terr = lookup_terrain_idx(pos_n, pos_e, pos_n_origin, pos_e_origin)

    ONE_DIS = 0.1;
    LEN_IDX_N = 61;
    LEN_IDX_E = 61;

    rel_n = pos_n - pos_n_origin;

    idx_n = floor(rel_n * ONE_DIS);
    if (idx_n < 0)
        idx_n = 0;
    elseif (idx_n >= LEN_IDX_N) 
        idx_n = LEN_IDX_N - 1;
    end

    rel_e = pos_e - pos_e_origin;

    idx_e = floor(rel_e * ONE_DIS);
    if (idx_e < 0) 
        idx_e = 0;
    elseif (idx_e >= LEN_IDX_E) 
        idx_e = LEN_IDX_E - 1;
    end
    
    idx_terr = idx_n*LEN_IDX_E + idx_e;