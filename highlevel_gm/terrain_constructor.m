% TERRAIN CONSTRUCTOR

% bottom left
terr_origin_n = -5000;
terr_origin_e = -5000;

dis = 10; % discretization
one_over_dis = 1/dis;
dis_2 = dis/2;

% northing / easting
nn = terr_origin_n:dis:5000;
ee = terr_origin_e:dis:5000;

len_global_idx_n = length(nn);
len_global_idx_e = length(ee);

% terrain
hh_sine = 10*sin(pi*((nn')/2000)).^2 + 0*ee;
hh_hill = 100*exp(-((ee - 100)/300).^2-((nn - 750)'/300).^2);

terrain_data0 = max(hh_sine,hh_hill);

% for plotting
terrain_data_plot = terrain_data0(501-100:501+300,501-100:501+100);
nn_plot = nn(501-100:501+300);
ee_plot = ee(501-100:501+100);