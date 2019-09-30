% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% radial ray casting with end point triangularization
%
% DEPRECATED: this was abandoned due to corner cases which may only be
% resolved by allowing a back propagation of triangle checking ..
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

clear; clc; close all;

% aircraft position
r_n = 0;
r_e = 0;
r_d = -8.5;

% aircraft velocity axis
v = 15;
xi = deg2rad(25);
gamma = deg2rad(5);

% wind axis 
w_n = -1;
w_e = 3;
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
vG_lat_2 = vG(1)^2 + vG(2)^2;
vG_lat = sqrt(vG_lat_2);

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
r_ray = vG(1:2)/vG_lat * (vG_lat^2 / g / tan(phi_max) * k_r + delta_r0);

% normalized ray indices
i_ray_ed = ([r_n; r_e]+r_ray)/terr_dis;
i_ray_st = [r_n; r_e]/terr_dis;

% make sure inputs are integers
x0 = int32(i_ray_st(2));
y0 = int32(i_ray_st(1));
x1 = int32(i_ray_ed(2));
y1 = int32(i_ray_ed(1));
output_everything = true;
% cast the ray
[x_occ_int,y_occ_int,occ_detected] = castray(x0, y0, x1, y1, -r_d, terr_mat, output_everything);
x_occ = double(x_occ_int);
y_occ = double(y_occ_int);

%% organize nearst neighbors  - - - - - - - - - - - - - - - - - - - - - - -

if occ_detected
    % an occlusion was detected
    
    % calculate the vector between the current position and the occlusion cell
    v_occ = [y_occ(end)*terr_dis; x_occ(end)*terr_dis] - [r_n, r_e];
    
    % calculate the occlusion approach angle
    chi_occ = atan2(v_occ(2),v_occ(1));
    
    % choose nearest triangularization
    if (chi_occ > pi/2)
        % REGION 1: bottom-right right triangle
        rt = 0;
        p1_e = (x_occ(end)-1)*terr_dis;
        p1_n = (y_occ(end)+0)*terr_dis;
        p1_h = terr_mat(1+ y_occ(end)+0, 1+ x_occ(end)-1);
        p2_h = terr_mat(1+ y_occ(end)+0, 1+ x_occ(end)+0);
        p3_h = terr_mat(1+ y_occ(end)+1, 1+ x_occ(end)+0);
        % for plotting
        p1 = [p1_n; p1_e; p1_h];
        p2 = [[y_occ(end)+0; x_occ(end)+0]*terr_dis; p2_h];
        p3 = [[y_occ(end)+1; x_occ(end)+0]*terr_dis; p3_h];
    elseif (chi_occ > pi/4)
        % REGION 2: top-left right triangle
        rt = 1;
        p1_e = (x_occ(end)-1)*terr_dis;
        p1_n = (y_occ(end)-1)*terr_dis;
        p1_h = terr_mat(1+ y_occ(end)-1, 1+ x_occ(end)-1);
        p2_h = terr_mat(1+ y_occ(end)+0, 1+ x_occ(end)-1);
        p3_h = terr_mat(1+ y_occ(end)+0, 1+ x_occ(end)+0);
        % for plotting
        p1 = [p1_n; p1_e; p1_h];
        p2 = [[y_occ(end)+0; x_occ(end)-1]*terr_dis; p2_h];
        p3 = [[y_occ(end)+0; x_occ(end)+0]*terr_dis; p3_h];
    elseif (chi_occ > 0)
        % REGION 3: bottom-right right triangle
        rt = 0;
        p1_e = (x_occ(end)-1)*terr_dis;
        p1_n = (y_occ(end)-1)*terr_dis;
        p1_h = terr_mat(1+ y_occ(end)-1, 1+ x_occ(end)-1);
        p2_h = terr_mat(1+ y_occ(end)-1, 1+ x_occ(end)+0);
        p3_h = terr_mat(1+ y_occ(end)+0, 1+ x_occ(end)+0);
        % for plotting
        p1 = [p1_n; p1_e; p1_h];
        p2 = [[y_occ(end)-1; x_occ(end)+0]*terr_dis; p2_h];
        p3 = [[y_occ(end)+0; x_occ(end)+0]*terr_dis; p3_h];
    elseif (chi_occ > -pi/2)
        % REGION 4: top-left right triangle
        rt = 1;
        p1_e = (x_occ(end)+0)*terr_dis;
        p1_n = (y_occ(end)-1)*terr_dis;
        p1_h = terr_mat(1+ y_occ(end)-1, 1+ x_occ(end)+0);
        p2_h = terr_mat(1+ y_occ(end)+0, 1+ x_occ(end)+0);
        p3_h = terr_mat(1+ y_occ(end)+0, 1+ x_occ(end)+1);
        % for plotting
        p1 = [p1_n; p1_e; p1_h];
        p2 = [[y_occ(end)+0; x_occ(end)+0]*terr_dis; p2_h];
        p3 = [[y_occ(end)+0; x_occ(end)+1]*terr_dis; p3_h];
    elseif (chi_occ > -3*pi/4)
        % REGION 5: bottom-right right triangle
        rt = 0;
        p1_e = (x_occ(end)+0)*terr_dis;
        p1_n = (y_occ(end)+0)*terr_dis;
        p1_h = terr_mat(1+ y_occ(end)+0, 1+ x_occ(end)+0);
        p2_h = terr_mat(1+ y_occ(end)+0, 1+ x_occ(end)+0);
        p3_h = terr_mat(1+ y_occ(end)+0, 1+ x_occ(end)+0);
        % for plotting
        p1 = [p1_n; p1_e; p1_h];
        p2 = [[y_occ(end)+0; x_occ(end)+0]*terr_dis; p2_h];
        p3 = [[y_occ(end)+0; x_occ(end)+0]*terr_dis; p3_h];
    else
        % REGION 6: top-left right triangle
        rt = 1;
        p1_e = (x_occ(end)+0)*terr_dis;
        p1_n = (y_occ(end)+0)*terr_dis;
        p1_h = terr_mat(1+ y_occ(end)+0, 1+ x_occ(end)+0);
        p2_h = terr_mat(1+ y_occ(end)+0, 1+ x_occ(end)+0);
        p3_h = terr_mat(1+ y_occ(end)+0, 1+ x_occ(end)+0);
        % for plotting
        p1 = [p1_n; p1_e; p1_h];
        p2 = [[y_occ(end)+0; x_occ(end)+0]*terr_dis; p2_h];
        p3 = [[y_occ(end)+0; x_occ(end)+0]*terr_dis; p3_h];
    end
    
    if (rt == 0)
        % bottom-right right triangle
        
        % triangulated plane coefficients
        A_br = terr_dis * (p3_h - p2_h);
        B_br = terr_dis * (p2_h - p1_h);
        C_br = -terr_dis^2;
        
        % horizontal distance to triangulated plane
        r_ne = -(A_br*(r_e - p1_e) + B_br*(r_n - p1_n) + C_br*(-r_d - p1_h)) * vG_lat / (A_br*vG(2) + B_br*vG(1) + C_br*0);
        
        % radial buffer zone
        delta_r = delta_r0 + vG_lat_2/g/tan(phi_max)*k_r;
        
        % jacobian of radial cost
        if (r_ne < delta_r)
            jac_sig_r = jac_sig_r_br(r_n,r_e,r_d,v,gamma,xi,w_e,w_n,terr_dis,p1_e,p1_h,p2_h,p3_h,p1_n,phi_max,delta_r0,g,k_r);
        else
            jac_sig_r = 0;
        end
        
    elseif (rt == 1)
        
        % top-left right triangle
        
        % triangulated plane coefficients    
        A_tl = -terr_dis * (p2_h - p1_h);
        B_tl = -terr_dis * (p3_h - p2_h);
        C_tl = terr_dis^2;
        
        % horizontal distance to triangulated plane
        r_ne = -(A_tl*(r_e - p1_e) + B_tl*(r_n - p1_n) + C_tl*(-r_d - p1_h)) * vG_lat / (A_tl*vG(2) + B_tl*vG(1) + C_tl*0);
        
        % radial buffer zone
        delta_r = delta_r0 + vG_lat_2/g/tan(phi_max)*k_r;
        
        % jacobian of radial cost
        if (r_ne < delta_r)
            jac_sig_r = jac_sig_r_tl(r_n,r_e,r_d,v,gamma,xi,w_e,w_n,terr_dis,p1_e,p1_h,p2_h,p3_h,p1_n,phi_max,delta_r0,g,k_r);
        else
            jac_sig_r = 0;
        end
        
    end
    
    % radial cost
    if (r_ne < delta_r)
        sig_r = (r_ne - delta_r)^3;
    else
        sig_r = 0;
    end
    
else
    % no occlusion detected
    
    % radial cost
    sig_r = 0;
    
    % jacobian of radial cost
    jac_sig_r = zeros(6,1);
   
end

%% interpolate terrain map  - - - - - - - - - - - - - - - - - - - - - - - -

len_plot_bl_interp = (len_nn-1)*10+1;
nn_plot = linspace(nn(1),nn(end),len_plot_bl_interp)';
ee_plot = linspace(ee(1),ee(end),len_plot_bl_interp)';
terr_mat_bl_interp = zeros(len_plot_bl_interp,len_plot_bl_interp);
for i=1:len_plot_bl_interp
    for j=1:len_plot_bl_interp
    
        [idx_q, dh] = lookup_terrain_idx_test(nn_plot(i), ee_plot(j), 0, 0, len_nn, len_ee, terr_dis);

        % bi-linear interpolation
        h12 = (1-dh(1))*terr_array(idx_q(1)+1) + dh(1)*terr_array(idx_q(2)+1); 
        h34 = (1-dh(1))*terr_array(idx_q(3)+1) + dh(1)*terr_array(idx_q(4)+1); 
        terr_mat_bl_interp(i,j) = (1-dh(2))*h12 + dh(2)*h34;
    end
end

terr_map_interpolated = true;

%% plot - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

figure('color','w');
hold on; grid on; box on;

% terrain
s0 = surf(ee,nn,terr_mat);
if terr_map_interpolated
    s1 = surf(ee_plot,nn_plot,terr_mat_bl_interp);
    alpha(s1, 0.2);
end


% ray cast cells
for i = 1:length(x_occ)
    if i==length(x_occ)
        fill_c = [1 0 0];
    else
        fill_c = [1 0.8 0.8];
    end
    fill3(([0 1 1 0]-0.5+double(x_occ(i)))*terr_dis, ...
        ([0 0 1 1]-0.5 + double(y_occ(i)))*terr_dis, ...
        ones(1,4)*-r_d, ...
        fill_c);
end

plot3(r_e, r_n, -r_d, '^g');
plot3(r_e+[0 r_ray(2)], r_n+[0 r_ray(1)], [-r_d -r_d], '--r');

if occ_detected
    
    % triangularization
    fill3([p1(2) p2(2) p3(2) p1(2)], ...
        [p1(1) p2(1) p3(1) p1(1)], ...
        [p1(3) p2(3) p3(3) p1(3)], ...
        [0.8 1 0.8]);
    
    % interesction point
    r_occ = [r_n; r_e] + vG(1:2)/vG_lat * r_ne;
    plot3([r_e; r_occ(2)], [r_n; r_occ(1)], -[r_d; r_d], 'r', 'linewidth', 2);
    plot3(r_occ(2), r_occ(1), -r_d, 'rs');
    
    % jacobian...
else
    plot3(r_e+r_ray(2), r_n+r_ray(1), -r_d, 'rs');
end

xlabel('East [m]');
ylabel('North [m]');
zlabel('Height [m]');

