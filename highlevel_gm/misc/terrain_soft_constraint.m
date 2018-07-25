clear;
clc;

h_terr = 10;
d_set = linspace(0,-50,501);
delta_h = 10;

h_p = max(1+(d_set+h_terr)/delta_h, 0);


figure('color','w'); hold on; grid on;
plot(-d_set,h_p.^2)