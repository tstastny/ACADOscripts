% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% radial ray casting (2 rays)
%
% DEPRECATED: why? integer rounding errors are a problem.
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

clear; clc; %close all;

% aircraft position
r_n = 2;
r_e = 4;
r_d = -8.9;

% aircraft velocity axis
v = 15;
xi = deg2rad(25);
gamma = deg2rad(5);

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
mag_ray = (vG_lat^2 / g / tan(phi_max) * k_r + delta_r0) + 2*terr_dis;
v_ray = vG(1:2)/vG_lat * mag_ray;

% normalized ray indices
% i_ray_ed = ([r_n; r_e]+v_ray)/terr_dis;
i_ray_st = [r_n; r_e]/terr_dis;

for i = 1:2
    
    if i==1
        % first ray
        
        x0 = int32(round(i_ray_st(2)));
        y0 = int32(round(i_ray_st(1)));
%         x1 = int32(round(i_ray_ed(2)));
%         y1 = int32(round(i_ray_ed(1)));
        i_ray_ed = (double([x0; y0])*terr_dis + v_ray)/terr_dis;
        x1 = int32(round(i_ray_ed(2)));
        y1 = int32(round(i_ray_ed(1)));
        
    elseif i==2
        % second ray
        
        % determine which direction to offset the next ray
        
        % remainder of rounded cell element
        rm_x = double(x0) - i_ray_st(2);
        rm_y = double(y0) - i_ray_st(1);
        
        % cross product of diff with ground velocity
        cp_rm = vG(2) * rm_y  - vG(1) * rm_x;
        
        % "perpendicular" step
        if (cp_rm > 0)
            % right rotation
            dx = y_occ_int(2)-y_occ_int(1);
            dy = -(x_occ_int(2)-x_occ_int(1));
        else 
            % left rotation
            dx = -(y_occ_int(2)-y_occ_int(1));
            dy = x_occ_int(2)-x_occ_int(1);
        end
        
        % update start / end points of ray
        x0 = x0 + dx;
        y0 = y0 + dy;
        x1 = x1 + dx;
        y1 = y1 + dy;
%         i_ray_ed = (double([x0; y0])*terr_dis + v_ray)/terr_dis;
%         x1 = int32(round(i_ray_ed(2)));
%         y1 = int32(round(i_ray_ed(1)));
    end
    
    output_everything = true;
    % cast the ray
    [x_occ_int,y_occ_int,occ_detected] = castray(x0, y0, x1, y1, -r_d, terr_mat, output_everything);
        t_raycast(k_timing+(i-1)*110) = toc(t_st);
    if i==1
        x_occ1 = double(x_occ_int);
        y_occ1 = double(y_occ_int);
        occ_detected1 = occ_detected;
    elseif i==2
        x_occ2 = double(x_occ_int);
        y_occ2 = double(y_occ_int);
        occ_detected2 = occ_detected;
    end
end

%% interpolate horizontal obstacle  - - - - - - - - - - - - - - - - - - - -

if occ_detected1
    r_occ1 = [y_occ1(end); x_occ1(end)]*terr_dis;
else
    r_occ1 = [y_occ1(1); x_occ1(1)]*terr_dis + v_ray*(1.01);
end
if occ_detected2
    r_occ2 = [y_occ2(end); x_occ2(end)]*terr_dis;
else
    r_occ2 = [y_occ2(1); x_occ2(1)]*terr_dis + v_ray*(1.01);
end

% horizontal distance to occlusion line
r_i = intersect2d([r_n; r_e], v_ray, r_occ1, r_occ2);
r_ne = sqrt((r_i(1)-r_n)^2 + (r_i(2)-r_e)^2);

% radial buffer zone
delta_r = delta_r0 + vG_lat_2/g/tan(phi_max)*k_r;
            
% radial cost
if (r_ne < delta_r)
    sig_r = (r_ne - delta_r)^3;
else
    sig_r = 0;
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
alpha(s0, 0.8);
% if terr_map_interpolated
%     s1 = surf(ee_plot,nn_plot,terr_mat_bl_interp, 'edgecolor', [0.3 0.3 0.3]);
%     alpha(s1, 0.6);
% end


% cells ray passes through
for i = 1:length(x_occ1)
    if i==length(x_occ1)
        if occ_detected1
            fill_c = [1 0 0];
        else
            fill_c = [0.8 0.8 0.8];
        end
    else
        fill_c = [1 0.8 0.8];
    end
    hf = fill3(([0 1 1 0]-0.5+double(x_occ1(i)))*terr_dis, ...
        ([0 0 1 1]-0.5 + double(y_occ1(i)))*terr_dis, ...
        ones(1,4)*-r_d, ...
        fill_c);
    set(hf,'facealpha',.8)
end
for i = 1:length(x_occ2)
    if i==length(x_occ2)
        if occ_detected2
            fill_c = [1 1 0];
        else
            fill_c = [1 1 0.8];
        end
    else
        fill_c = [1 1 0.8];
    end
    hf = fill3(([0 1 1 0]-0.5+double(x_occ2(i)))*terr_dis, ...
        ([0 0 1 1]-0.5 + double(y_occ2(i)))*terr_dis, ...
        ones(1,4)*-r_d-0.1, ...
        fill_c);
    set(hf,'facealpha',.8)
end

% aircraft position
plot3(r_e, r_n, -r_d, '^c', 'markersize', 5);

% ground velocity
chi = atan2(vG(2),vG(1));
% ray truncation border
cc = linspace(chi-pi/8,chi+pi/8,361);
plot3((mag_ray-2*terr_dis)*sin(cc), (mag_ray-2*terr_dis)*cos(cc), -r_d*ones(length(cc),1),'c');

yend1 = min(max(y_occ1(end)+1, 0), len_nn);
xend1 = min(max(x_occ1(end)+1, 0), len_ee);
yend2 = min(max(y_occ2(end)+1, 0), len_nn);
xend2 = min(max(x_occ2(end)+1, 0), len_ee);

% ray 1
plot3(x_occ1(1)*terr_dis, y_occ1(1)*terr_dis, -r_d, 'rs', 'markersize', 5);
plot3(x_occ1(end)*terr_dis, y_occ1(end)*terr_dis, terr_mat(yend1, xend1), 'gs', 'markersize', 5);

% ray 2
plot3(x_occ2(1)*terr_dis, y_occ2(1)*terr_dis, -r_d, 'rs', 'markersize', 5);
plot3(x_occ2(end)*terr_dis, y_occ2(end)*terr_dis, terr_mat(yend2, xend2), 'gs', 'markersize', 5);

% terrain line
plot3([x_occ1(end), x_occ2(end)]*terr_dis, [y_occ1(end), y_occ2(end)]*terr_dis, ...
    [terr_mat(yend1, xend1), terr_mat(yend2, xend2)], 'g', 'linewidth', 2);

% ground velocity collision point
plot3([r_e, r_i(2)], [r_n, r_i(1)], -r_d*ones(1,2), '-g', 'linewidth', 2);
plot3(r_i(2), r_i(1), -r_d, 'g^', 'markersize', 5);

xlabel('East [m]');
ylabel('North [m]');
zlabel('Height [m]');

