
clear; clc;

omega_n_mu = 5;
zeta_mu = 0.8;
omega_n_gamma = 10;
zeta_gamma = 0.8;

dt = 0.01;
time = 0:dt:10;

x = zeros(length(time),4);
mu_r = 1;
gamma_r = 1;

for k = 2:length(time)
    
    mu = x(k-1,1);
    gamma = x(k-1,2);
    mu_dot = x(k-1,3);
    gamma_dot = x(k-1,4);
    mu_dot_dot = omega_n_mu * (omega_n_mu * (mu_r - mu) - 2 * zeta_mu * mu_dot);
    gamma_dot_dot = omega_n_gamma * (omega_n_gamma * (gamma_r - gamma) - 2 * zeta_gamma * gamma_dot);
    
    d_x = [mu_dot,gamma_dot,mu_dot_dot,gamma_dot_dot];
    x(k,:) = x(k-1,:) + d_x * dt;
    
end

figure('color','w');
subplot(2,1,1); hold on; grid on;
plot([time(1),time(end)],[mu_r,mu_r],'--')
plot(time',x(:,1))
% ylabel('\mu')
% subplot(2,1,2); hold on; grid on;
% plot([time(1),time(end)],[gamma_cmd,gamma_cmd],'--')
plot(time',x(:,2))
ylabel('\gamma')

