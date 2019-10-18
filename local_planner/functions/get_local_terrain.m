% get local terrain

rel_n = posk(1) - terr_origin_n;

normalized_rel_n = max(rel_n * one_over_dis, 0);
idx_n = floor(normalized_rel_n);
if (idx_n >= len_global_idx_n)
    idx_n = len_global_idx_n-1;
elseif (idx_n < 0)
    idx_n = 0;
end

rel_e = posk(2) - terr_origin_e;

normalized_rel_e = max(rel_e * one_over_dis, 0);
idx_e = floor(normalized_rel_e);
if (idx_e >= len_global_idx_e)
    idx_e = len_global_idx_e-1;
elseif (idx_e < 0)
    idx_e = 0;
end

len_2_n = (len_local_idx_n-1)/2;
len_2_e = (len_local_idx_e-1)/2;

if (add_terrain_noise_to_local_map)
    terr_noise_matrix = (rand(len_local_idx_n,len_local_idx_e)*2-1)*terr_noise;
else
    terr_noise_matrix = 0;
end

terrain_data_matrix = terrain_data0(idx_n-len_2_n+1:idx_n+len_2_n+1,...
    idx_e-len_2_e+1:idx_e+len_2_e+1) + terr_noise_matrix;

terrain_data = reshape(terrain_data_matrix.',1,len_local_idx_n*len_local_idx_e);

nnk = nn(idx_n-len_2_n+1:idx_n+len_2_n+1);
eek = ee(idx_e-len_2_e+1:idx_e+len_2_e+1);

terr_local_origin_n = nnk(1);
terr_local_origin_e = eek(1);


