clear; clc;

dt = 0.1;
x_k1 = 50;
u_rec(1) = 0;

for k = 1:1000
    
    x_rec(k) = x_k1;
    
    u_rec(k) = -0.1 * x_rec(k);
    
    xdot_k = u_rec(k);
    
    x_k1 = xdot_k * dt + x_rec(k);
    
end

t = 0:dt:dt*(1000-1);

figure(1);
subplot(2,1,1); hold on; grid on;
plot(t,x_rec);
subplot(2,1,2); hold on; grid on;
plot(t,u_rec);