function [idx_q, dn, de] = lookup_terrain_idx( pos_n, pos_e, pos_n_origin, pos_e_origin, map_height, map_width, map_resolution)
        
    map_height_1 = map_height-1;
    map_width_1 = map_width-1;
    
    % relative position / indices
    rel_n = pos_n - pos_n_origin;
    rel_n_bar = rel_n / map_resolution;
    idx_n = floor(rel_n_bar);
    rel_e = pos_e - pos_e_origin;
    rel_e_bar = rel_e / map_resolution;
    idx_e = floor(rel_e_bar);
    
    % interpolation weights
    dn = rel_n_bar-idx_n;
    de = rel_e_bar-idx_e;
    
    % cap ends
    if (idx_n < 0)
        idx_n = 0;
    elseif (idx_n > map_height_1)
        idx_n = map_height_1;
    end
    if (idx_e < 0)
        idx_e = 0;
    elseif (idx_e > map_width_1)
        idx_e = map_width_1;
    end

    % neighbors (north)
    if (idx_n >= map_height_1) 
        q_n(1) = map_height_1;
        q_n(2) = map_height_1;
        q_n(3) = map_height_1;
        q_n(4) = map_height_1;
    else 
        q_n(1) = idx_n;
        q_n(2) = idx_n + 1;
        q_n(3) = idx_n;
        q_n(4) = idx_n + 1;
    end
    % neighbors (east)
    if (idx_e >= map_height_1)
        q_e(1) = map_width_1;
        q_e(2) = map_width_1;
        q_e(3) = map_width_1;
        q_e(4) = map_width_1;
    else
        q_e(1) = idx_e;
        q_e(2) = idx_e;
        q_e(3) = idx_e + 1;
        q_e(4) = idx_e + 1;
    end

    % neighbors row-major indices
    idx_q(1) = q_n(1)*map_width + q_e(1);
    idx_q(2) = q_n(2)*map_width + q_e(2);
    idx_q(3) = q_n(3)*map_width + q_e(3);
    idx_q(4) = q_n(4)*map_width + q_e(4);

end