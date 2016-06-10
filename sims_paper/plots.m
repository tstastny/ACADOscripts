
clear; clc;
r2d = 180/pi;

figure('color','w','name','windy loiter');
subplot(2,6,[1:3,7:9]); hold on; grid on; axis equal; box on;
% path
ltset=1000;
tset = linspace(0,2*pi,ltset);
plot(sin(tset)*60,cos(tset)*60,'-','color',[0.6 0.6 0.6])
% traj N40
load sim_loiter_wind_N40.mat
h_pos(1) = plot(X_rec(:,2),X_rec(:,1),'-','color',[0 0 0]);
plot(X_rec(1,2),X_rec(1,1),'^','color',[0 0 0])
text(X_rec(1,2)+10,X_rec(1,1),'start')
plot(X_rec(end,2),X_rec(end,1),'s','color',[0 0 0])
text(X_rec(end,2)-10,X_rec(end,1)+10,'end')
clearvars -except r2d h_pos
% traj N80
load sim_loiter_wind_N80.mat
h_pos(2) = plot(X_rec(:,2),X_rec(:,1),':','color',[0 0 0.6]);
plot(X_rec(1,2),X_rec(1,1),'^','color',[0 0 0.6])
% text(X_rec(1,2)+10,X_rec(1,1),'start')
plot(X_rec(end,2),X_rec(end,1),'s','color',[0 0 0.6])
% text(X_rec(end,2)-10,X_rec(end,1)+10,'end')
legend(h_pos,{'N=40','N=80'})
xlabel('Easting [m]')
ylabel('Northing [m]')
title('Position')
xlim([-90,70])

subplot(2,6,4:6); hold on; grid on; box on;
clearvars -except r2d
% traj N40
load sim_loiter_wind_N40.mat
h_att(1) = plot(time,U_rec(:,1)*r2d,'--','color',[0.6 0.6 0.6]);
h_att(2) = plot(time,X_rec(:,3)*r2d,'-','color',[0 0 0]);
ylabel('N = 40');
title('Bank Angle, \mu [deg]');
legend('Reference','Actual');
xlim([time(1),time(end)])
subplot(2,6,10:12); hold on; grid on; box on;
clearvars -except r2d
% traj N80
load sim_loiter_wind_N80.mat
h_att(1) = plot(time,U_rec(:,1)*r2d,'--','color',[0.6 0.6 0.6]);
h_att(2) = plot(time,X_rec(:,3)*r2d,'-','color',[0 0 0]);
ylabel('N = 80');
xlabel('Time [s]')
xlim([time(1),time(end)])


figure('color','w','name','slew');
hold on; grid on; box on;
clearvars -except r2d
% traj w/o
load sim_loiter_wind_N40_noWmuprev.mat
h_att(1) = plot(time,U_rec(:,1)*r2d,'-','color',[0.6 0.6 0.6]);
clearvars -except r2d h_att
% traj w
load sim_loiter_wind_N40.mat
h_att(2) = plot(time,U_rec(:,1)*r2d,'-','color',[0 0 0]);
title('Bank Angle Reference');
ylabel('\mu_r [deg]')
legend(h_att,{'W_{\mu_{k-1}} = 0','W_{\mu_{k-1}} = 100'});
xlim([time(471),time(691)])
xlabel('Time [s]')
