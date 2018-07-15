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

% lookup 2.5d grid
[idx_q, dh] = lookup_terrain_idx(in(1), in(2), in(19), in(20));
% bi-linear interpolation
h12 = (1-dh(1))*in(21+idx_q(1)) + dh(1)*in(21+idx_q(2));
h34 = (1-dh(1))*in(21+idx_q(3)) + dh(1)*in(21+idx_q(4));
h_terr = (1-dh(2))*h12 + dh(2)*h34;
% soft constraint formulation
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

