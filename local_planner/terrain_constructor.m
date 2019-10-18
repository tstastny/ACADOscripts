% TERRAIN CONSTRUCTOR

% bottom left
terr_origin_n = -5000;
terr_origin_e = -5000;

one_over_dis = 1/terr_dis;
dis_2 = terr_dis/2;

% northing / easting
nn = terr_origin_n:terr_dis:5000;
ee = terr_origin_e:terr_dis:5000;

len_global_idx_n = length(nn);
len_global_idx_e = length(ee);

% noise
if (terrain_noise_random_seed > 0)
    rng(terrain_noise_random_seed);
else
    rng('shuffle');
end
if (add_terrain_noise_to_global_map)
    terr_noise_matrix = (rand(len_global_idx_n, len_global_idx_e)*2-1)*terr_noise;
else
    terr_noise_matrix = 0;
end

% terrain
% hh_sine = 100*sin(pi*((nn')/2000)).^2 + 0*ee;
% hh_hill = 180*exp(-((ee - 100)/300).^2-((nn - 750)'/300).^2);

hh_wall = zeros(len_global_idx_n, len_global_idx_e);
hh_wall(nn > 300, :) = 200;

% terrain_data0 = max(hh_sine,hh_hill)+terr_noise_matrix;
terrain_data0 = hh_wall+terr_noise_matrix;

% for plotting
center_terr_idx = [(len_global_idx_n-1)/2; (len_global_idx_e-1)/2]+1;
off_e_terr_idx = round([-200; 500]*one_over_dis);
off_n_terr_idx = round([-200; 2000]*one_over_dis);
terrain_data_plot = ...
    terrain_data0( ...
    center_terr_idx(1)+off_n_terr_idx(1):center_terr_idx(1)+off_n_terr_idx(2), ...
    center_terr_idx(2)+off_e_terr_idx(1):center_terr_idx(2)+off_e_terr_idx(2) ...
    );
nn_plot = nn(center_terr_idx(1)+off_n_terr_idx(1):center_terr_idx(1)+off_n_terr_idx(2));
ee_plot = ee(center_terr_idx(2)+off_e_terr_idx(1):center_terr_idx(2)+off_e_terr_idx(2));

if false
    %% THIS ISNT CORRECT...
    
%     nn_voxel = zeros(length(nn_plot)*3-2,1);
%     ee_voxel = zeros(length(ee_plot)*3-2,1);
%     h_voxel = zeros(length(nn_voxel),length(ee_voxel));
%     ii_last = 0;
%     for ii=1:length(nn_plot)
%         
%         if ii == length(nn_plot)
%             iii = ii;
%             nn_voxel(iii) = nn_plot(ii);
%         else
%             iii = (ii-1)*3+1:(ii-1)*3+3;
%             nn_voxel(iii) = nn_plot(ii)*[1; 1+dis/10; 1+dis*9/10];
%         end
%                 
%         for jj=1:length(ee_plot)
%             
%             if jj == length(ee_plot)
%                 jjj = jj;
%                 if ii_last ~= ii
%                     ee_voxel(jjj) = ee_plot(jj);
%                     ii_last = ii;
%                 end
%             else
%                 jjj = (jj-1)*3+1:(jj-1)*3+3;
%                 if ii_last ~= ii
%                     ee_voxel(jjj) = ee_plot(jj)*[1; 1+dis/10; 1+dis*9/10];
%                     ii_last = ii;
%                 end
%             end
% 
%             h_voxel(iii, jjj) = terrain_data_plot(ii,jj);
%         end
%     end
%     
%     figure('color','w');
%     hold on; grid on; box on;
%     
%     surf(ee_voxel, nn_voxel, h_voxel,'edgecolor','none');
%     
%     xlabel('East [m]');
%     ylabel('North [m]');
%     zlabel('Height [m]');

%     figure('color','w');
%     hold on; grid on; box on;
%     
%     surf(ee, nn, terrain_data0,'edgecolor','none');
%     
%     xlabel('East [m]');
%     ylabel('North [m]');
%     zlabel('Height [m]');

    figure('color','w');
    hold on; grid on; box on;
    
    surf(ee_plot, nn_plot, terrain_data_plot,'edgecolor','none');
    
    xlabel('East [m]');
    ylabel('North [m]');
    zlabel('Height [m]');

    
end
    
    
    