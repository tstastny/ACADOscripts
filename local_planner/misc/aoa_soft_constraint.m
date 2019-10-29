clear;
clc;
close all;

aoa_set = linspace(-10,13,1001);
aoa_m = -5;
aoa_p = 8;
delta_aoa = 2;
sig_aoa_1 = 0.01; % MUST be > 0
w_sig_aoa = 5;

% weight dependent scaling
k_aoa = log(w_sig_aoa/sig_aoa_1);

% exponential cost
sig_aoa_p = w_sig_aoa*exp((aoa_set - aoa_p)/delta_aoa*k_aoa);
sig_aoa_m = w_sig_aoa*exp(-(aoa_set - aoa_m)/delta_aoa*k_aoa);

% linearize cost beyond zero point
sig_aoa_p(aoa_set - aoa_p > 0) = w_sig_aoa * (1.0 + k_aoa/delta_aoa * (aoa_set(aoa_set - aoa_p > 0) - aoa_p));
sig_aoa_m(aoa_set - aoa_m < 0) = w_sig_aoa * (1.0 + -k_aoa/delta_aoa * (aoa_set(aoa_set - aoa_m < 0) - aoa_m));

sig_aoa = sig_aoa_p + sig_aoa_m;

% exponential jacobians
t2 = 1.0/delta_aoa; 
t3 = k_aoa*sig_aoa_m*t2; 
jac_gamma = t3-k_aoa*sig_aoa_p.*t2; 
jac_theta = -t3+k_aoa*sig_aoa_p.*t2;

% linearized jacobians
jac_gamma(aoa_set - aoa_p > 0) = w_sig_aoa * -k_aoa/delta_aoa;
jac_gamma(aoa_set - aoa_m < 0) = w_sig_aoa * k_aoa/delta_aoa;
jac_theta(aoa_set - aoa_p > 0) = w_sig_aoa * k_aoa/delta_aoa;
jac_theta(aoa_set - aoa_m < 0) = w_sig_aoa * -k_aoa/delta_aoa;

figure('color','w');

subplot(2,2,[1 3]); hold on; grid on; box on;
plot(aoa_set, sig_aoa_m);
plot(aoa_set, sig_aoa_p);
plot(aoa_set, sig_aoa);
xlabel('\alpha [deg]');
ylabel('\sigma_\alpha');

xlim(aoa_set([1 end]));

subplot(2,2,2); hold on; grid on; box on;
plot(aoa_set, jac_gamma);
xlabel('\alpha [deg]');
ylabel('d(\sigma)/d(\gamma)');

xlim(aoa_set([1 end]));

subplot(2,2,4); hold on; grid on; box on;
plot(aoa_set, jac_theta);
xlabel('\alpha [deg]');
ylabel('d(\sigma)/d(\theta)');

xlim(aoa_set([1 end]));
