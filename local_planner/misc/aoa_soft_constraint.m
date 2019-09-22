clear;
clc;
close all;

aoa_set = linspace(-10,13,1001);
k_aoa = 10/2;
aoa_m = -5;
aoa_p = 8;
delta_aoa = 2;

sig_aoa_m = 1./(1 + exp(k_aoa*(aoa_set - aoa_m)));
sig_aoa_p = 1 - 1./(1 + exp(k_aoa*(aoa_set - aoa_p)));
sig_aoa = sig_aoa_m .* (1 - sig_aoa_p) + sig_aoa_p .* (1 - sig_aoa_m);

aoa_mid = (aoa_p + aoa_m)/2;
aoa_set_1 = aoa_set;
% aoa_set_1(aoa_set > aoa_mid) = aoa_mid - (aoa_set(aoa_set > aoa_mid) - aoa_mid);
aoa_set_1(aoa_set > aoa_mid) = 2.0*aoa_mid - aoa_set(aoa_set > aoa_mid);
sig_aoa_one_exp = 1./(1 + exp(k_aoa*(aoa_set_1 - aoa_m)));

sig_aoa_exp_quad = sig_aoa_one_exp .* (aoa_set_1-aoa_m).^2;

sig_aoa_quad = zeros(length(aoa_set),1);
sig_aoa_quad(aoa_set < aoa_m + delta_aoa) = (aoa_set(aoa_set < aoa_m + delta_aoa) - aoa_m - delta_aoa).^2/delta_aoa^2;
sig_aoa_quad(aoa_set > aoa_p - delta_aoa) = (aoa_set(aoa_set > aoa_p - delta_aoa) - aoa_p + delta_aoa).^2/delta_aoa^2;

sig_aoa_cub = zeros(length(aoa_set),1);
sig_aoa_cub(aoa_set < aoa_m + delta_aoa) = abs(aoa_set(aoa_set < aoa_m + delta_aoa) - aoa_m - delta_aoa).^3/delta_aoa^3;
sig_aoa_cub(aoa_set > aoa_p - delta_aoa) = abs(aoa_set(aoa_set > aoa_p - delta_aoa) - aoa_p + delta_aoa).^3/delta_aoa^3;

sig_aoa_exp = exp((aoa_set - aoa_p)*6.82/delta_aoa) + exp((-aoa_set + aoa_m)*6.82/delta_aoa);

delta_aoa2 = 3;
sig_aoa_exp2 = exp((aoa_set - aoa_p)*6.82/delta_aoa2) + exp((-aoa_set + aoa_m)*6.82/delta_aoa2);


figure('color','w'); hold on; grid on;
% plot(aoa_set, sig_aoa_m);
% plot(aoa_set, sig_aoa_p);
% plot(aoa_set, sig_aoa);
% plot(aoa_set, sig_aoa_one_exp);
plot(aoa_set, sig_aoa_quad);
plot(aoa_set, sig_aoa_cub);
plot(aoa_set, sig_aoa_exp);
plot(aoa_set, sig_aoa_exp2);
xlabel('\alpha [deg]');
ylabel('\sigma_\alpha');

xlim([aoa_m-2 aoa_p+2]);