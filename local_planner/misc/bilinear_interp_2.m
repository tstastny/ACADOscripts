clear; clc; close all;

len = 301;
x = linspace(0,3,len);
y = linspace(0,3,len)';

fxy = (0.5 * cos(y*pi/2)) * sin(x*pi/2);

% % syms x y;
% % fxy = (0.5 * cos(y*pi/2)) * sin(x*pi/2);
% % fxy_x = jacobian(fxy,x)
% % fxy_y = jacobian(fxy,y)

dis = 0.2;

xdis = x(1):dis:x(end);
ydis = y(1):dis:y(end);

len_i = length(xdis);

fxy_bl = zeros(len,len);
fxy_bl_x = zeros(len,len);
fxy_bl_y = zeros(len,len);
fxy_x = zeros(len,len);
fxy_y = zeros(len,len);
fxy_bl_x_fd = zeros(len,len);
fxy_bl_y_fd = zeros(len,len);
for i = 1:len
    
    ix1_uncstr = floor(x(i) / dis);
    if ix1_uncstr + 1 > len_i
        ix1 = len_i;
        ix2 = len_i;
    else
        ix1 = ix1_uncstr + 1;
        ix2 = ix1_uncstr + 1 + 1;
        if ix2 > len_i
            ix2 = len_i;
        end
    end
    x1 = ix1_uncstr*dis;
    x2 = x1+dis;

    for j = 1:len

        iy1_uncstr = floor(y(j) / dis);
        if iy1_uncstr + 1 > len_i
            iy1 = len_i;
            iy2 = len_i;
        else
            iy1 = iy1_uncstr + 1;
            iy2 = iy1_uncstr + 1 + 1;
            if iy2 > len_i
                iy2 = len_i;
            end
        end
        y1 = iy1_uncstr*dis;
        y2 = y1+dis;
        
        
        f11 = (0.5 * cos(ydis(iy1)*pi/2)) * sin(xdis(ix1)*pi/2);
        f12 = (0.5 * cos(ydis(iy2)*pi/2)) * sin(xdis(ix1)*pi/2);
        f21 = (0.5 * cos(ydis(iy1)*pi/2)) * sin(xdis(ix2)*pi/2);
        f22 = (0.5 * cos(ydis(iy2)*pi/2)) * sin(xdis(ix2)*pi/2);

        fxy_bl(j,i) = 1/(dis*dis) * [x2 - x(i), x(i) - x1] * [f11 f12; f21 f22] * [y2 - y(j); y(j) - y1];
        
        fxy_bl_x(j,i) = (f11/dis^2 - f21/dis^2)*(y(j) - y2) - (f12/dis^2 - f22/dis^2)*(y(j) - y1);
 
        fxy_bl_y(j,i) = (f11*(x(i) - x2))/dis^2 - (f12*(x(i) - x2))/dis^2 - (f21*(x(i) - x1))/dis^2 + (f22*(x(i) - x1))/dis^2;
        
        delta_fd = 0.00001;
        fxy_bl_x_fd(j,i) = ...
            ( ...
            (1/(dis*dis) * [x2 - (x(i)+delta_fd), (x(i)+delta_fd) - x1] * [f11 f12; f21 f22] * [y2 - y(j); y(j) - y1]) - ...
            (1/(dis*dis) * [x2 - (x(i)-delta_fd), (x(i)-delta_fd) - x1] * [f11 f12; f21 f22] * [y2 - y(j); y(j) - y1]) ...
            ) / (2*delta_fd);
        fxy_bl_y_fd(j,i) = ...
            ( ...
            (1/(dis*dis) * [x2 - x(i), x(i) - x1] * [f11 f12; f21 f22] * [y2 - (y(j)+delta_fd); (y(j)+delta_fd) - y1]) - ...
            (1/(dis*dis) * [x2 - x(i), x(i) - x1] * [f11 f12; f21 f22] * [y2 - (y(j)-delta_fd); (y(j)-delta_fd) - y1]) ...
            ) / (2*delta_fd);
        
        fxy_x(j,i) = (pi*cos((pi*x(i))/2)*cos((pi*y(j))/2))/4;
 
        fxy_y(j,i) = -(pi*sin((pi*x(i))/2)*sin((pi*y(j))/2))/4;
        
    end
    
end

%%
figure('color','w'); hold on; grid on; box on;

surf(x, y, fxy, 'edgecolor', 'none');
surf(x, y, fxy_bl + 2, 'edgecolor', 'none');

xlabel('x');
ylabel('y');
zlabel('f');

%%
figure('color','w'); hold on; grid on; box on;

sc = 1.5;

surf(x, y, sc*fxy_x, 'edgecolor', 'none');
surf(x, y, sc*fxy_bl_x + 7, 'edgecolor', 'none');
surf(x, y, sc*fxy_bl_x_fd + 14, 'edgecolor', 'none');

xlabel('x');
ylabel('y');
zlabel('f_x');

%%
figure('color','w'); hold on; grid on; box on;

sc = 1.5;

surf(x, y, sc*fxy_y, 'edgecolor', 'none');
surf(x, y, sc*fxy_bl_y + 7, 'edgecolor', 'none');
surf(x, y, sc*fxy_bl_y_fd + 14, 'edgecolor', 'none');

xlabel('x');
ylabel('y');
zlabel('f_y');
