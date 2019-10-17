function [idx_q, dn, de] = lookup_terrain_idx( pos_n, pos_e, pos_n_origin, pos_e_origin, terr_dis)
        
%     LEN_IDX_N = 141;
%     LEN_IDX_E = 141;
%     LEN_IDX_N_1 = 140;
%     LEN_IDX_E_1 = 140;

%     LEN_IDX_N = 29;
%     LEN_IDX_E = 29;
%     LEN_IDX_N_1 = 28;
%     LEN_IDX_E_1 = 28;

    LEN_IDX_N = 57;
    LEN_IDX_E = 57;
    LEN_IDX_N_1 = 56;
    LEN_IDX_E_1 = 56;
    
    % relative position / indices
    rel_n = pos_n - pos_n_origin;
    rel_n_bar = rel_n / terr_dis;
    idx_n = floor(rel_n_bar);
    rel_e = pos_e - pos_e_origin;
    rel_e_bar = rel_e / terr_dis;
    idx_e = floor(rel_e_bar);
    
    % interpolation weights
    dn = rel_n_bar-idx_n;
    de = rel_e_bar-idx_e;
    
    % cap ends
    if (idx_n < 0)
        idx_n = 0;
    elseif (idx_n > LEN_IDX_N_1)
        idx_n = LEN_IDX_N_1;
    end
    if (idx_e < 0)
        idx_e = 0;
    elseif (idx_e > LEN_IDX_E_1)
        idx_e = LEN_IDX_E_1;
    end

    % neighbors (north)
    if (idx_n >= LEN_IDX_N_1) 
        q_n(1) = LEN_IDX_N_1;
        q_n(2) = LEN_IDX_N_1;
        q_n(3) = LEN_IDX_N_1;
        q_n(4) = LEN_IDX_N_1;
    else 
        q_n(1) = idx_n;
        q_n(2) = idx_n + 1;
        q_n(3) = idx_n;
        q_n(4) = idx_n + 1;
    end
    % neighbors (east)
    if (idx_e >= LEN_IDX_N_1)
        q_e(1) = LEN_IDX_E_1;
        q_e(2) = LEN_IDX_E_1;
        q_e(3) = LEN_IDX_E_1;
        q_e(4) = LEN_IDX_E_1;
    else
        q_e(1) = idx_e;
        q_e(2) = idx_e;
        q_e(3) = idx_e + 1;
        q_e(4) = idx_e + 1;
    end

    % neighbors row-major indices
    idx_q(1) = q_n(1)*LEN_IDX_E + q_e(1);
    idx_q(2) = q_n(2)*LEN_IDX_E + q_e(2);
    idx_q(3) = q_n(3)*LEN_IDX_E + q_e(3);
    idx_q(4) = q_n(4)*LEN_IDX_E + q_e(4);

end