% plot position gradients for terrain avoidnace cost

figure('color','w');

subplot(3,2,1); hold on; grid on; box on;
plot(time, rec.dJdn(:,8));
ylabel('d(sig_h)/d(r_n)');

subplot(3,2,3); hold on; grid on; box on;
plot(time, rec.dJde(:,8));
ylabel('d(sig_h)/d(r_e)');

subplot(3,2,5); hold on; grid on; box on;
plot(time, rec.dJdd(:,8));
ylabel('d(sig_h)/d(r_d)');

subplot(3,2,2:2:6); hold on; grid on; box on;
plot3(rec.x(:,2), rec.x(:,1), -rec.x(:,3));
di = 30;
ii = 1:di:length(time);
quiver3(rec.x(ii,2), rec.x(ii,1), -rec.x(ii,3), -rec.dJde(ii,8), -rec.dJdn(ii,8), rec.dJdd(ii,8));
ylabel('D(J)/D(r) per position [m]');
xlabel('East [m]');
ylabel('North [m]');
zlabel('Height [m]');