
r2d = 180/pi;

figure('color','w','name','airspeed/airflow angles');

subplot(3,1,1); hold on; grid on;
plot(time',Y_rec(:,1),'--k');
plot(time',X_rec(:,1)); ylabel('V [m/s]');
subplot(3,1,2); hold on; grid on;
plot(time',X_rec(:,2)*r2d); ylabel('\beta [deg]');
subplot(3,1,3); hold on; grid on;
plot(time',X_rec(:,3)*r2d); ylabel('\alpha [deg]');
xlabel('t [s]')

figure('color','w','name','body angular rates');

subplot(3,1,1); hold on; grid on;
plot(time',X_rec(:,4)*r2d); ylabel('p [deg/s]');
subplot(3,1,2); hold on; grid on;
plot(time',X_rec(:,5)*r2d); ylabel('q [deg/s]');
subplot(3,1,3); hold on; grid on;
plot(time',X_rec(:,6)*r2d); ylabel('r [deg/s]');
xlabel('t [s]')

figure('color','w','name','euler angles');

subplot(3,1,1); hold on; grid on;
plot(time',Y_rec(:,3)*r2d,'--k');
plot(time',X_rec(:,7)*r2d); ylabel('\phi [deg]');
subplot(3,1,2); hold on; grid on;
plot(time',Y_rec(:,2)*r2d,'--k');
plot(time',X_rec(:,8)*r2d); ylabel('\theta [deg]');
subplot(3,1,3); hold on; grid on;
plot(time',X_rec(:,9)*r2d); ylabel('\psi [deg]');
xlabel('t [s]')

figure('color','w','name','controls');

minde = min(U_rec(:,2));
maxde = max(U_rec(:,2));
rangede = maxde-minde;
minda = min(U_rec(:,3));
maxda = max(U_rec(:,3));
rangeda = maxda-minda;
mindr = min(U_rec(:,4));
maxdr = max(U_rec(:,4));
rangedr = maxdr-mindr;

subplot(4,1,1); hold on; grid on;
plot(time',U_rec(:,1),'--k');
plot(time',X_rec(:,13)); ylabel('\delta_T [~]'); ylim([0,1]);
subplot(4,1,2); hold on; grid on;
plot(time',U_rec(:,2)*r2d); ylabel('\delta_E [deg]'); ylim([(minde-rangede*0.1)*r2d-eps,(maxde+rangede*0.1)*r2d+eps]);
subplot(4,1,3); hold on; grid on;
plot(time',U_rec(:,3)*r2d); ylabel('\delta_A [deg]'); ylim([(minda-rangeda*0.1)*r2d-eps,(maxda+rangeda*0.1)*r2d+eps]);
subplot(4,1,4); hold on; grid on;
plot(time',U_rec(:,4)*r2d); ylabel('\delta_R [deg]'); ylim([(mindr-rangedr*0.1)*r2d-eps,(maxdr+rangedr*0.1)*r2d+eps]);
xlabel('t [s]')

figure('color','w','name','aux');

subplot(3,1,1); hold on; grid on;
% plot(time',U_rec(:,5)*r2d); ylabel('\alpha_{slack} [deg]'); ylim([0,5])
plot(time',U_rec(:,5)); ylabel('V_{slack} [m/s]'); 
subplot(3,1,2); hold on; grid on;
plot(time',X_rec(:,14)); ylabel('intg_V');
subplot(3,1,3); hold on; grid on;
plot(time',tsolve'*1E3); ylabel('T_{solve} [ms]'); ylim([0,max(tsolve)*1E3*1.1]);
xlabel('t [s]')

figure('color','w','name','aux 2');

hold on; grid on;%p1=subplot(4,1,1); 
plot(time',Y_rec(:,6)*r2d); ylabel('\gamma [deg]');

% figure('color','w','name','aux 2');
% 
% p1=subplot(4,1,1); hold on; grid on;
% plot(time',Y_rec(:,6)*r2d); ylabel('\eta_{LON} [deg]');
% p2=subplot(4,1,2); hold on; grid on;
% plot(time',Y_rec(:,7)*r2d); ylabel('\eta_{LAT} [deg]');
% subplot(4,1,3); hold on; grid on;
% plot(time',Y_rec(:,4)); ylabel('intg_{LON}');
% subplot(4,1,4); hold on; grid on;
% plot(time',Y_rec(:,5)); ylabel('intg_{LAT}');
% xlabel('t [s]')
% linkaxes([p1,p2],'x')

figure('color','w','name','3D position/height');

subplot(5,1,1:4); hold on; grid on;

% ltset=1000;
% for k = 1:length(paths)
% 
%     if paths(k).pparam1 == 0
%         plot3([paths(k).pparam3 paths(k).pparam6], ...
%             [paths(k).pparam2 paths(k).pparam5], ...
%             -[paths(k).pparam4 paths(k).pparam7],'--m','linewidth',2);
%     elseif paths(k).pparam1 == 1
%         tset = linspace(paths(k).pparam8, ...
%             paths(k).pparam8 + paths(k).pparam9 * paths(k).pparam6, ...
%             ltset)';
%         r = repmat([paths(k).pparam2,paths(k).pparam3,paths(k).pparam4],ltset,1) + ...
%             [paths(k).pparam5 * cos(tset), paths(k).pparam5 * sin(tset), ...
%             -paths(k).pparam6 * (tset-tset(1)) * paths(k).pparam5 * tan(paths(k).pparam7)];
%         plot3(r(:,2),r(:,1),-r(:,3),'--m','linewidth',2)
%     end
%     
% end
plot3(X_rec(:,11),X_rec(:,10),-X_rec(:,12));
xlabel('e [m]'); ylabel('n [m]'); zlabel('h [m]');
subplot(5,1,5); hold on; grid on;
plot(time',-X_rec(:,12)); ylabel('h [m]');
xlabel('t [s]')

