% TERRAIN CONSTRUCTOR

terr_origin_n = 0;
terr_origin_e = 0;
idx_center = 501;
one_over_dis = 0.1;
dis = 10;
dis_2 = 5;

len_local_idx_n = 61;
len_local_idx_e = 61;

nn = -5000:10:5000;
ee = -5000:10:5000;

len_global_idx_n = length(nn);
len_global_idx_e = length(ee);

hh_sine = 10*sin(pi*((nn')/2000)).^2 + 0*ee;
hh_hill = 25*exp(-((ee - 250)/500).^2-((nn - 750)'/500).^2);

terrain_data0 = max(hh_sine,hh_hill);

terrain_data_plot = terrain_data0(400:600,400:600);
nn_plot = nn(400:600);
ee_plot = ee(400:600);