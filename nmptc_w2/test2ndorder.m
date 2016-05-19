
clear; clc;

k_mu = 2;
k_mu_dot = 5;
k_gamma = 5;
k_gamma_dot = 5;

dt = 0.01;
time = 0:dt:10;

x = zeros(length(time),4);
mu_cmd = 35;
gamma_cmd = 35;

for k = 2:length(time)
    
%     mu_dot=(mu_cmd - x(k-1,1)) * k_mu;
%     gamma_dot=(gamma_cmd - x(k-1,2)) * k_gamma;
%     mu_dot_dot=0;
%     gamma_dot_dot=0;
    mu_dot = x(k-1,3);
    gamma_dot = x(k-1,4);
    mu_dot_dot = ( (mu_cmd - x(k-1,1)) * k_mu - mu_dot ) * k_mu_dot;
    gamma_dot_dot = ( (gamma_cmd - x(k-1,2)) * k_gamma - gamma_dot ) * k_gamma_dot;
    
    d_x = [mu_dot,gamma_dot,mu_dot_dot,gamma_dot_dot];
    x(k,:) = x(k-1,:) + d_x * dt;
    
end

figure('color','w');
subplot(2,1,1); hold on; grid on;
plot([time(1),time(end)],[mu_cmd,mu_cmd],'--')
plot(time',x(:,1))
% ylabel('\mu')
% subplot(2,1,2); hold on; grid on;
% plot([time(1),time(end)],[gamma_cmd,gamma_cmd],'--')
plot(time',x(:,2))
ylabel('\gamma')

