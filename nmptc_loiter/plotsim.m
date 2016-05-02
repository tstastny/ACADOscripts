
r2d = 180/pi;

figure('color','w','name','Position')
subplot(5,1,1:4); hold on; grid on;
plot3(X_rec(:,2),X_rec(:,1),-X_rec(:,3))
xlabel('East [m]')
ylabel('North [m]')
zlabel('Height [m]');
subplot(5,1,5); hold on; grid on;
plot(time,J_rec(:,1));
ylabel('e_T')
xlabel('time [s]')

figure('color','w','name','Attitude')
subplot(3,1,1); hold on; grid on;
plot(time,U_rec(:,1)*r2d,'-k');
plot(time,X_rec(:,4)*r2d);
ylabel('\mu [deg]');
legend('reference','state');
subplot(3,1,2); hold on; grid on;
plot(time,U_rec(:,2)*r2d,'-k');
plot(time,X_rec(:,5)*r2d);
ylabel('\gamma [deg]')
legend('reference','state');
subplot(3,1,3); hold on; grid on;
plot(time,X_rec(:,6)*r2d);
ylabel('\xi [deg]')
xlabel('time [s]')

figure('color','w','name','Auxillary')
stairs(time,tsolve*10^3)
ylabel('t_{solve} [ms]')
xlabel('time [s]')
