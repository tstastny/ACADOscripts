
cmap = lines(7);
color_ref = [0.5 0.5 0.5];
color_hor = [min(ones(1,3), cmap(2,:)*1.6) 0.2];%[0.2 0.2 1 0.6];


%% /////////////////////////////////////////////////////////////////////////
% POSITION

figure('color','w','name','Position')
hold on; grid on; %axis equal;

% path
len_path = 500;
plot3([be, be+len_path*sin(chi_p)*cos(gamma_p)], ...
    [bn, bn+len_path*cos(chi_p)*cos(gamma_p)], ...
    -[bd, bd-len_path*sin(gamma_p)],'color',color_ref,'linewidth',1.5);

% position horizons
hor_int = round(10/Ts);
horO = plot3(rec.x_hor(1,1:hor_int:end,2), rec.x_hor(1,1:hor_int:end,1), ...
    -rec.x_hor(1,1:hor_int:end,3), 'o', 'MarkerSize', 4, 'Color', color_hor(1:3));
% horO.Color = color_hor;
hor1 = plot3(rec.x_hor(:,1:hor_int:end,2), rec.x_hor(:,1:hor_int:end,1), ...
    -rec.x_hor(:,1:hor_int:end,3), 'Color', color_hor(1:3));
% hor1.Color = color_hor;

% position
plot3(rec.x(:,2),rec.x(:,1),-rec.x(:,3), 'linewidth',1.5, 'color', cmap(2,:));

xlabel('East [m]')
ylabel('North [m]')
zlabel('Height [m]');

%% /////////////////////////////////////////////////////////////////////////
% ATTITUDE

figure('color','w','name','Attitude');

% flight path angle
hand_att(1) = subplot(3,1,1); hold on; grid on;
plot(time,rad2deg(rec.u(:,1)),'color',color_ref);
plot(time,rad2deg(rec.x(:,4)));
legend('ref','state')
ylabel('\gamma [deg]')

% heading angle
hand_att(2) = subplot(3,1,2); hold on; grid on;
plot(time,rad2deg(rec.x(:,5)));
ylabel('\xi [deg]')

% bank angle
hand_att(3) = subplot(3,1,3); hold on; grid on;
plot(time,rad2deg(rec.u(:,2)),'color',color_ref);
plot(time,rad2deg(rec.x(:,6)));
legend('ref','state')
ylabel('\mu [deg]')

xlabel('Time [s]')

linkaxes(hand_att,'x')

%% /////////////////////////////////////////////////////////////////////////
% OBJECTIVE COSTS

figure('color','w','name','Objective Costs')

hand_obj(1)=subplot(4,1,1); hold on; grid on;

plot(time,rec.yz(:,1));
plot(time,rec.yz(:,2));

legend('e_{lat}','e_{lon}')
ylabel('e [m]')

hand_obj(2)=subplot(4,1,2:4); hold on; grid on;

plot(time,(yref(1)*ones(length(time),1)-rec.yz(:,1)).^2*Q_output(1)); % e_lat
plot(time,(yref(2)*ones(length(time),1)-rec.yz(:,2)).^2*Q_output(2)); % e_lon

plot(time,(zref(1)*ones(length(time),1)-rec.yz(:,3)).^2*R_controls(1)); % gamma_ref
plot(time,(zref(2)*ones(length(time),1)-rec.yz(:,4)).^2*R_controls(2)); % mu_ref
plot(time,(zref(3)*ones(length(time),1)-rec.yz(:,5)).^2*R_controls(3)); % gamma dot
plot(time,(zref(4)*ones(length(time),1)-rec.yz(:,6)).^2*R_controls(4)); % mu dot


legend('e_{lat}','e_{lon}','\gamma_{ref}','\mu_{ref}','\gamma{dot}','\mu{dot}')
ylabel('J(x,u)')

xlabel('time [s]')

linkaxes(hand_obj,'x');
