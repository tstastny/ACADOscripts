% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% radial ray casting (use floats, intersect triangles)
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

clear; clc; close all;

% aircraft position
r_n = 2.5;
r_e = 5.1;
r_d = -8.9;

% aircraft velocity axis
v = 15;
xi = deg2rad(35);
gamma = deg2rad(10);

% wind axis 
w_n = 0;
w_e = 0;
w_d = 0;

% terrain discretization
terr_dis = 5;

% radial avoidance cost parameters
delta_r0 = 10;
g = 9.81;
phi_max = deg2rad(35);
k_r = 1;

% ground speed vector
vG = [ ...
    v*cos(gamma)*cos(xi) + w_n;
    v*cos(gamma)*sin(xi) + w_e;
    -v*sin(gamma) + w_d;
    ];
norm_vG = norm(vG);

% construct terrain - - - - - - - - - - - - - - - - - - - - - - - - - - - -

terr_mat = [...
    0 0 0 0 0 0 1 2 5 9 15;
    0 0 0 0 0 0 1 3 4 10 15;
    0 0 0 0 0 1 1 4 2 8 12;
    0 0 0 0 0 1 2 5 6 10 15;
    0 0 0 0 0 2 5 6 8 14 18;
    0 0 0 0 3 5 10 10 12 15 20;
    0 0 0 0 2 8 8 10 14 18 20;
    0 0 0 0 3 8 10 12 16 20 22;
    0 0 0 0 5 7 12 16 18 22 22;
    0 0 0 6 8 10 14 16 19 22 25;
    0 0 0 6 12 14 16 20 25 28 30;
    ] * 1.4;
[len_nn, len_ee] = size(terr_mat);
nn = (0:terr_dis:(len_nn-1)*terr_dis)';
ee = (0:terr_dis:(len_ee-1)*terr_dis)';
terr_array = reshape(terr_mat',1,len_nn*len_ee);

terr_map_interpolated = false;

%% ray casting  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

% ray vector
delta_r = norm_vG^2 / g / tan(phi_max) * k_r + delta_r0; % radial buffer zone
mag_ray = delta_r + terr_dis;
v_ray = vG/norm_vG * mag_ray;

% start/end pos of ray
r0 = [r_n; r_e; r_d];
r1 = r0 + v_ray;

output_everything = true;
% cast the ray
[x_occ, y_occ, h_occ, occ_detected, tri, d_occ, p_occ, p1, p2, p3] = ...
    castray_float(r0, r1, vG/norm_vG, terr_dis, terr_mat, len_ee, len_nn, output_everything);

%% radial cost

% calculate radial cost
if (d_occ < delta_r)
    sig_r = (delta_r - d_occ)^3;
else
    sig_r = 0;
end
disp(['Radial cost = ',num2str(sig_r)]);

% calculate radial cost jacobians
if occ_detected
    if tri == 0
        Del_sig_r = jac_sig_r_br(r_n,r_e,r_d,v,gamma,xi,w_e,w_n,w_d,terr_dis,p1(2),p1(1),p1(3),p2(2),p2(1),p2(3),p3(2),p3(1),p3(3),phi_max,delta_r0,g,k_r);
    elseif tri == 1
        Del_sig_r = jac_sig_r_tl(r_n,r_e,r_d,v,gamma,xi,w_e,w_n,w_d,terr_dis,p1(2),p1(1),p1(3),p2(2),p2(1),p2(3),p3(2),p3(1),p3(3),phi_max,delta_r0,g,k_r);
    end
end
disp(['d(sig_r)/d(r_n) = ',num2str(Del_sig_r(1))]);
disp(['d(sig_r)/d(r_e) = ',num2str(Del_sig_r(2))]);
disp(['d(sig_r)/d(r_d) = ',num2str(Del_sig_r(3))]);
disp(['d(sig_r)/d(v) = ',num2str(Del_sig_r(4))]);
disp(['d(sig_r)/d(gamma) = ',num2str(Del_sig_r(5))]);
disp(['d(sig_r)/d(xi) = ',num2str(Del_sig_r(6))]);

%% interpolate terrain map  - - - - - - - - - - - - - - - - - - - - - - - -

% len_plot_bl_interp = (len_nn-1)*10+1;
% nn_plot = linspace(nn(1),nn(end),len_plot_bl_interp)';
% ee_plot = linspace(ee(1),ee(end),len_plot_bl_interp)';
% terr_mat_bl_interp = zeros(len_plot_bl_interp,len_plot_bl_interp);
% for i=1:len_plot_bl_interp
%     for j=1:len_plot_bl_interp
%     
%         [idx_q, dh] = lookup_terrain_idx_test(nn_plot(i), ee_plot(j), 0, 0, len_nn, len_ee, terr_dis);
% 
%         % bi-linear interpolation
%         h12 = (1-dh(1))*terr_array(idx_q(1)+1) + dh(1)*terr_array(idx_q(2)+1); 
%         h34 = (1-dh(1))*terr_array(idx_q(3)+1) + dh(1)*terr_array(idx_q(4)+1); 
%         terr_mat_bl_interp(i,j) = (1-dh(2))*h12 + dh(2)*h34;
%     end
% end
% 
% terr_map_interpolated = true;

%% plot - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

figure('color','w');
hold on; grid on; box on;

% terrain
s0 = surf(ee,nn,terr_mat);
alpha(s0, 0.6);
% if terr_map_interpolated
%     s1 = surf(ee_plot,nn_plot,terr_mat_bl_interp, 'edgecolor', [0.3 0.3 0.3]);
%     alpha(s1, 0.6);
% end

% cells ray passes through
for i = 1:length(x_occ)
    if i==length(x_occ)
        if occ_detected
            fill_c = [1 0.8 0.8];
            edge_c = [1 0 0];
            lw = 1;
        else
            fill_c = [0.8 0.8 0.8];
            edge_c = [0 0 0];
            lw = 1;
        end
    else
        fill_c = [1 0.8 0.8];
        edge_c = [0 0 0];
        lw = 1;
    end
    hf = fill3(([0 1 1 0]+double(x_occ(i)))*terr_dis, ...
        ([0 0 1 1]+double(y_occ(i)))*terr_dis, ...
        h_occ(i)*ones(1,4), ...
        fill_c, ...
        'EdgeColor', edge_c, 'linewidth', lw);
    set(hf,'facealpha',.8)
    plot3(([0 1]+double(x_occ(i)))*terr_dis, ...
        ([0 1]+double(y_occ(i)))*terr_dis, ...
        h_occ(i)*ones(1,2), '-k');
end

% aircraft position
plot3(r_e, r_n, -r_d, '^c', 'markersize', 5);

% ray truncation border
chi = atan2(vG(2),vG(1)); % course
Gamma = atan2(-vG(3), norm_vG); % inertial fpa
Phi = linspace(0-pi/16,0+pi/16,361);
circ_x0 = (mag_ray-terr_dis)*cos(Phi); % x in ray direction
circ_y0 = (mag_ray-terr_dis)*sin(Phi);
circ_z0 = zeros(1,length(Phi));
rot_Gamma = [cos(-Gamma) 0 sin(-Gamma); 0 1 0; -sin(-Gamma) 0 cos(-Gamma)]; % rotation about ray dir "y" axis
circ_x1 = rot_Gamma(1,:) * [circ_x0; circ_y0; circ_z0];
circ_y1 = rot_Gamma(2,:) * [circ_x0; circ_y0; circ_z0];
circ_z1 = rot_Gamma(3,:) * [circ_x0; circ_y0; circ_z0];
rot_chi = [cos(chi-pi/2) sin(chi-pi/2) 0; -sin(chi-pi/2) cos(chi-pi/2) 0; 0 0 1]; % rotation about ray dir "z" axis
circ_x2 = rot_chi(1,:) * [circ_x1; circ_y1; circ_z1];
circ_y2 = rot_chi(2,:) * [circ_x1; circ_y1; circ_z1];
circ_z2 = rot_chi(3,:) * [circ_x1; circ_y1; circ_z1];
plot3(circ_x2 + r_e, circ_y2 + r_n, -r_d + circ_z2, '--g', 'linewidth', 1);

% ray-triangle intersection
if occ_detected
    
    % violated triangle
    tri_off = 0;
    to_vG = tri_off/norm_vG;
    ht = fill3([p1(1) p2(1) p3(1) p1(1)]-vG(2)*to_vG, ...
        [p1(2) p2(2) p3(2) p1(2)]-vG(1)*to_vG, ...
        [p1(3) p2(3) p3(3) p1(3)]+vG(3)*to_vG, ...
        [1 0 0], 'linewidth', 2, 'edgecolor', [1 0 0]);
    set(ht,'facealpha',.6)
    
    % interception point
    plot3([r0(2) p_occ(1)], [r0(1) p_occ(2)], [-r0(3) p_occ(3)], '-r', 'linewidth', 2);
    plot3(p_occ(1), p_occ(2), p_occ(3), 'ro', 'markersize', 10);
    
    % negative position jacobian
    sz = 2/norm(Del_sig_r(1:3));
    quiver3(r0(2), r0(1), -r0(3), ...
        -Del_sig_r(2)*sz, -Del_sig_r(1)*sz, Del_sig_r(3)*sz, ...
        'm');
else
    % ray
    plot3([r0(2) r1(2)], [r0(1) r1(1)], -[r0(3) r1(3)], '-g', 'linewidth', 1);
end

xlabel('East [m]');
ylabel('North [m]');
zlabel('Height [m]');

