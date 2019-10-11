clear;
clc;
close all;

aoa_set = linspace(-10,13,1001);
aoa_m = -5;
aoa_p = 8;
delta_aoa = 2;
sig_aoa_1 = 0.01; % MUST be > 0
w_sig_aoa = 10;


k_aoa = log(w_sig_aoa/sig_aoa_1);

sig_aoa_p = w_sig_aoa*exp((aoa_set - aoa_p)/delta_aoa*k_aoa);
sig_aoa_m = w_sig_aoa*exp(-(aoa_set - aoa_m)/delta_aoa*k_aoa);
sig_aoa = sig_aoa_p + sig_aoa_m;

figure('color','w'); hold on; grid on;
plot(aoa_set, sig_aoa_m);
plot(aoa_set, sig_aoa_p);
plot(aoa_set, sig_aoa);
xlabel('\alpha [deg]');
ylabel('\sigma_\alpha');

xlim([aoa_m aoa_p]);