function [idx_q, dh] = lookup_terrain_idx_test(pos_n, pos_e, pos_n_origin, pos_e_origin, len_idx_n, len_idx_e, terr_dis)
        
    len_idx_n_1 = len_idx_n-1;
    len_idx_e_1 = len_idx_e-1;
    
    % relative position / indices
    rel_n = pos_n - pos_n_origin;
    rel_n_bar = rel_n / terr_dis;
    idx_n = floor(rel_n_bar);
    if (idx_n < 0)
        idx_n = 0;
    elseif (idx_n > len_idx_n_1)
        idx_n = len_idx_n_1;
    end
    rel_e = pos_e - pos_e_origin;
    rel_e_bar = rel_e / terr_dis;
    idx_e = floor(rel_e_bar);
    if (idx_e < 0)
        idx_e = 0;
    elseif (idx_e > len_idx_e_1)
        idx_e = len_idx_e_1;
    end

    % neighbor orientation / interpolation weights
    delta_n = rel_n_bar-idx_n;
%     if (delta_n<0.5) 
%         down = -1;
%         dh_n = 0.5 + delta_n;
%     else
%         down = 0;
%         dh_n = delta_n - 0.5;
%     end
    delta_e = rel_e_bar-idx_e;
%     if (delta_e<0.5) 
%         left = -1;
%         dh_e = 0.5 + delta_e;
%     else 
%         left = 0;
%         dh_e = delta_e - 0.5;
%     end

    % neighbor origin (down,left)
%     q1_n = max(idx_n + down, 0);
%     q1_e = max(idx_e + left, 0);
    q1_n = max(idx_n, 0);
    q1_e = max(idx_e, 0);

    % neighbors (north)
    if (q1_n >= len_idx_n_1) 
        q_n(1) = len_idx_n_1;
        q_n(2) = len_idx_n_1;
        q_n(3) = len_idx_n_1;
        q_n(4) = len_idx_n_1;
    else 
        q_n(1) = q1_n;
        q_n(2) = q1_n + 1;
        q_n(3) = q1_n;
        q_n(4) = q1_n + 1;
    end
    % neighbors (east)
    if (q1_e >= len_idx_n_1)
        q_e(1) = len_idx_e_1;
        q_e(2) = len_idx_e_1;
        q_e(3) = len_idx_e_1;
        q_e(4) = len_idx_e_1;
    else
        q_e(1) = q1_e;
        q_e(2) = q1_e;
        q_e(3) = q1_e + 1;
        q_e(4) = q1_e + 1;
    end

    % neighbors row-major indices
    idx_q(1) = q_n(1)*len_idx_e + q_e(1);
    idx_q(2) = q_n(2)*len_idx_e + q_e(2);
    idx_q(3) = q_n(3)*len_idx_e + q_e(3);
    idx_q(4) = q_n(4)*len_idx_e + q_e(4);

    % interpolation weights
%     dh(1) = dh_n;
%     dh(2) = dh_e;
    dh(1) = delta_n;
    dh(2) = delta_e;
end