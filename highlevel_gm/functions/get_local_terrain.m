% get local terrain

rel_n = posk(1) - terr_origin_n;
if (rel_n<0.0)
    rel_n = rel_n - dis_2;
else
    rel_n = rel_n + dis_2;
end
normalized_rel_n = rel_n * one_over_dis;
if normalized_rel_n < 0
    delta_idx_n = ceil(rel_n * one_over_dis);
else
    delta_idx_n = floor(rel_n * one_over_dis);
end
idx_n = idx_center + delta_idx_n;
if (idx_n >= len_global_idx_n)
    idx_n = len_global_idx_n-1;
elseif (idx_n < 0)
    idx_n = 0;
end

rel_e = posk(1) - terr_origin_n;
if (rel_e<0.0)
    rel_e = rel_e - dis_2;
else
    rel_e = rel_e + dis_2;
end
normalized_rel_e = rel_e * one_over_dis;
if normalized_rel_e < 0
    delta_idx_e = ceil(rel_e * one_over_dis);
else
    delta_idx_e = floor(rel_e * one_over_dis);
end
idx_e = idx_center + delta_idx_e;
if (idx_e >= len_global_idx_e)
    idx_e = len_global_idx_e-1;
elseif (idx_e < 0)
    idx_e = 0;
end

terrain_data_matrix = terrain_data0(idx_n+1-(len_local_idx_n-1)/2:idx_n+1+(len_local_idx_n-1)/2,...
    idx_e+1-(len_local_idx_e-1)/2:idx_e+1+(len_local_idx_e-1)/2);
terrain_data = reshape(terrain_data_matrix.',1,len_local_idx_n*len_local_idx_e);
nnk = nn(idx_n+1-(len_local_idx_n-1)/2:idx_n+1+(len_local_idx_n-1)/2);
eek = ee(idx_e+1-(len_local_idx_e-1)/2:idx_e+1+(len_local_idx_e-1)/2);