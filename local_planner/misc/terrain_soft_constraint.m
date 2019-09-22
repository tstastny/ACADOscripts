clear;
clc;

h_terr = 10;
d_set = linspace(0,-50,501);
delta_h = 10;
k_h = 10/2;
h_thres = 20;

% h_p = max(1+(d_set+h_terr)/delta_h, 0);

% h_rel = -d_set - h_terr;
% sig_h = 1./(1 + exp(k_h*(h_rel - h_thres)));

sig_h = zeros(length(d_set),1);
sig_h(-d_set < h_terr + delta_h) = (abs(-d_set(-d_set < h_terr + delta_h) - h_terr - delta_h) / delta_h).^3;

figure('color','w'); hold on; grid on;
% plot(-d_set,h_p.^2)
plot(-d_set, sig_h)