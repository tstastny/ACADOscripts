% check bi-linear interp

% lookup 2.5d grid  
[idx_q, dh] = lookup_terrain_idx(300, 0, terr_local_origin_n, terr_local_origin_e, terr_dis); 
 
% bi-linear interpolation 
h12 = (1-dh(1))*terrain_data(idx_q(1)+1) + dh(1)*terrain_data(idx_q(2)+1); 
h34 = (1-dh(1))*terrain_data(idx_q(3)+1) + dh(1)*terrain_data(idx_q(4)+1); 
h_terr = (1-dh(2))*h12 + dh(2)*h34;

clc;
disp(['c1(',int2str(idx_q(1)),'): ',num2str(terrain_data(idx_q(1)+1))]);
disp(['c2(',int2str(idx_q(2)),'): ',num2str(terrain_data(idx_q(2)+1))]);
disp(['c3(',int2str(idx_q(3)),'): ',num2str(terrain_data(idx_q(3)+1))]);
disp(['c4(',int2str(idx_q(4)),'): ',num2str(terrain_data(idx_q(4)+1))]);
disp(['h_terr: ',num2str(h_terr)]);