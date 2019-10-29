% test raytrace
close all;
clear;
clc;

r_n = 69;
r_e = 64;
r_d = -10;

yaw = deg2rad(50);
r_end = [r_n r_e] + 150*[cos(yaw); sin(yaw)];

terr_dis = 5;

terr_mat = zeros(41,41);
terr_mat(31:41,31:41) = 20;
len_terr = length(terr_mat);
xx = linspace(0,(len_terr-1)*terr_dis,len_terr)';
yy = xx;

i_ray_ed = (r_end)/terr_dis;
i_ray_st = ([r_n, r_e])/terr_dis;

% make sure inputs are integers
x0 = int32(i_ray_st(2));
y0 = int32(i_ray_st(1));
x1 = int32(i_ray_ed(2));
y1 = int32(i_ray_ed(1));
output_everything = true;
% cast the ray
[x_occ,y_occ,occ_found] = castray(x0, y0, x1, y1, -r_d, terr_mat, output_everything);
% [x_occ,y_occ,occ_found] = bresenham(i_ray_st(2), i_ray_st(1), i_ray_ed(2), i_ray_ed(1), -r_d, terr_mat, output_everything);

%%

figure('color','w'); hold on; grid on; box on;

ss = surf(xx,yy,terr_mat);
alpha(ss, 0.2);

for i = 1:length(x_occ)
    fill3(([0 1 1 0]-0.5+double(x_occ(i)))*terr_dis, ...
        ([0 0 1 1]-0.5 + double(y_occ(i)))*terr_dis, ...
        ones(1,4)*-r_d, ...
        [1 0.8 0.8]);
end

plot3([i_ray_st(2), i_ray_ed(2)]*terr_dis, ...
    [i_ray_st(1), i_ray_ed(1)]*terr_dis, ...
    ones(1,2)*-r_d, ...
    'r', 'linewidth', 2);

xlabel('x');
ylabel('y');
zlabel('h');

zlim([0 max([terr_mat(:); -r_d])]);


