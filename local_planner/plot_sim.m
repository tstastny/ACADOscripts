
cmap = lines(7);
color_ref = [0.5 0.5 0.5];
color_hor = [min(ones(1,3), cmap(5,:)*1.6) 0.2];%[0.2 0.2 1 0.6];
color_state = cmap(1,:);

%% ////////////////////////////////////////////////////////////////////////
% POSITION

figure('color','w','name','Position')
hold on; grid on; %axis equal;

% terrain
surf1 = surf(ee_plot,nn_plot,terrain_data_plot,'edgecolor','none');
surf2 = surf(eek,nnk,terrain_data_matrix+1,'edgecolor','none');
alpha(surf1,0.5);
alpha(surf2,0.5);

% path
len_path = 1500;
plot3([b_e, b_e+len_path*sin(chi_p)*cos(Gamma_p)], ...
    [b_n, b_n+len_path*cos(chi_p)*cos(Gamma_p)], ...
    -[b_d, b_d-len_path*sin(Gamma_p)],'color',color_ref,'linewidth',1.5);

% position horizons
hor_int = round(10/Ts);
horO = plot3(rec.x_hor(1,1:hor_int:end,2), rec.x_hor(1,1:hor_int:end,1), ...
    -rec.x_hor(1,1:hor_int:end,3), 'o', 'MarkerSize', 4, 'Color', color_hor(1:3));
% horO.Color = color_hor;
hor1 = plot3(rec.x_hor(:,1:hor_int:end,2), rec.x_hor(:,1:hor_int:end,1), ...
    -rec.x_hor(:,1:hor_int:end,3), 'Color', color_hor(1:3));
% hor1.Color = color_hor;

% position
plot3(rec.x(:,2),rec.x(:,1),-rec.x(:,3), 'linewidth', 1.5, 'color', cmap(2,:));

xlabel('East [m]')
ylabel('North [m]')
zlabel('Height [m]');

%% ////////////////////////////////////////////////////////////////////////
% POSITION 2

figure('color','w','name','Position 2');
hold on; grid on; %axis equal;

% path
len_path = 500;
b_ed_e = b_e+len_path*sin(chi_p)*cos(Gamma_p);
b_ed_n = b_n+len_path*cos(chi_p)*cos(Gamma_p);
b_ed_d = b_d-len_path*sin(Gamma_p);
plot3([b_e, b_ed_e], ...
    [b_n, b_ed_n], ...
    -[b_d, b_ed_d],'color',color_ref,'linewidth',1.5);

% position horizons
hor_int = round(10/Ts);
horO = plot3(rec.x_hor(1,1:hor_int:end,2), rec.x_hor(1,1:hor_int:end,1), ...
    -rec.x_hor(1,1:hor_int:end,3), 'o', 'MarkerSize', 4, 'Color', color_hor(1:3));
% horO.Color = color_hor;
hor1 = plot3(rec.x_hor(:,1:hor_int:end,2), rec.x_hor(:,1:hor_int:end,1), ...
    -rec.x_hor(:,1:hor_int:end,3), 'Color', color_hor(1:3));
% hor1.Color = color_hor;

% position
plot3(rec.x(:,2),rec.x(:,1),-rec.x(:,3), 'linewidth', 1.5, 'color', color_state);

xlabel('East [m]')
ylabel('North [m]')
zlabel('Height [m]');

%% ////////////////////////////////////////////////////////////////////////
% ATTITUDE

figure('color','w','name','Attitude');

% roll angle
hand_att(1) = subplot(3,1,1); hold on; grid on; box on;
plot(time,rad2deg(rec.u(:,2)),'color',color_ref);
plot(time,rad2deg(rec.x(:,7)),'color',color_state);
legend('ref','state')
ylabel('\phi [deg]')

% pitch
hand_att(2) = subplot(3,1,2); hold on; grid on; box on;
plot(time,rad2deg(rec.u(:,3)),'color',color_ref);
plot(time,rad2deg(rec.x(:,8)),'color',color_state);
legend('ref','state')
ylabel('\theta [deg]')

% heading
hand_att(3) = subplot(3,1,3); hold on; grid on; box on;
plot(time,rad2deg(rec.x(:,6)),'color',color_state);
ylabel('\xi [deg]')

xlabel('Time [s]')

linkaxes(hand_att,'x')

%% ////////////////////////////////////////////////////////////////////////
% MOTOR

figure('color','w','name','Motor');

% u_T
hand_mot(1) = subplot(2,1,1); hold on; grid on; box on;
plot(time,rec.u(:,1),'color',color_ref);
ylabel('u_T [~]')

% prop speed
hand_mot(2) = subplot(2,1,2); hold on; grid on; box on;

u_n = calc_prop_speed(rec.u(:,1),rec.x(:,4),rec.x(:,8)-rec.x(:,5),model_params,sysid_config);

plot(time,u_n,'color',color_ref);
plot(time,rec.x(:,9),'color',color_state);
legend('ref','state')
ylabel('n_p [rps]')

xlabel('Time [s]')

linkaxes(hand_att,'x')

%% ////////////////////////////////////////////////////////////////////////
% POSITION ERRORS

figure('color','w','name','Position errors')

hand_pos_err(1)=subplot(2,1,1); hold on; grid on; box on;

plot(time,rec.aux(:,2));
plot(time,rec.aux(:,3));

legend('e_{lat}','e_{lon}')
ylabel('e [m]')

hand_pos_err(2)=subplot(2,1,2); hold on; grid on; box on;

plot(time,rec.aux(:,1),'color',color_ref);
plot(time,rec.aux(:,1)+delta_h,'color',color_ref,'linestyle','--');
plot(time,-rec.x(:,3),'color',color_state);

legend('h_{terr}','h_{terr,buf}','h')
ylabel('Height [m]')

xlabel('time [s]')

linkaxes(hand_pos_err,'x');

%% ////////////////////////////////////////////////////////////////////////
% DIRECTIONAL ERRORS

figure('color','w','name','Directional errors')

hand_dir_err(1)=subplot(2,1,1); hold on; grid on; box on;

plot(time,rec.aux(:,4));
plot(time,rec.aux(:,5));

legend('e_{v_n}','e_{v_e}')
ylabel('e_v [m/s]')

hand_dir_err(2)=subplot(2,1,2); hold on; grid on; box on;

plot(time,rec.aux(:,6));

ylabel('e_{v_d} [m/s]')

%% ////////////////////////////////////////////////////////////////////////
% WIND AXIS

figure('color','w','name','Wind axis');

% airspeed
hand_vab(1) = subplot(2,1,1); hold on; grid on; box on;
plot(time([1 end]),[v_ref v_ref],'color',color_ref);
plot(time,rec.x(:,4),'color',color_state);
legend('ref','state')
ylabel('v [m/s]')

% angle of attack
hand_vab(2) = subplot(2,1,2); hold on; grid on; box on;
plot(time([1 end]),rad2deg([aoa_m aoa_m]),'color',color_ref);
plot(time([1 end]),rad2deg([aoa_p aoa_p]),'color',color_ref);
plot(time,rad2deg(rec.x(:,8) - rec.x(:,5)),'color',color_state);
ylabel('\alpha [deg]')

%% ////////////////////////////////////////////////////////////////////////
% OBJECTIVE COSTS

obj_cost = zeros(len_t, n_Y+n_Z);
for ii = 1:n_Y
    obj_cost(:,ii) = (yref(ii)*ones(length(time),1)-rec.yz(:,ii)).^2*Q_output(ii);
end

for ii = 1:n_Z
    obj_cost(:,n_Y+ii) = (zref(ii)*ones(length(time),1)-rec.yz(:,n_Y+ii)).^2*R_controls(ii);
end

figure('color','w','name','Objective Costs')

hand_obj(1)=subplot(2,1,1); hold on; grid on; box on;

leg_str_ = cell(1,n_Y);
for ii = 1:n_Y
    plot(time,obj_cost(:,ii));
    leg_str_{ii} = ['y',int2str(ii)];
end

legend(hand_obj(1), leg_str_);
ylabel('J(x)');
ylim([0 max(obj_cost(:))]);

hand_obj(2)=subplot(2,1,2); hold on; grid on; box on;

leg_str_ = cell(1,n_Z);
for ii = 1:n_Z
    plot(time,obj_cost(:,n_Y+ii));
    leg_str_{ii} = ['z',int2str(ii)];
end

legend(hand_obj(2), leg_str_);
ylabel('J(u)');
ylim([0 max(obj_cost(:))]);

xlabel('time [s]');

linkaxes(hand_obj,'x');

%% ////////////////////////////////////////////////////////////////////////
% TIMING

% figure('color','w','name','NMPC timing'); hold on; grid on; box on;
% 
% plot(time,tsolve);
% plot(time,tarray);
% 
% legend('nmpc','array allocation');
% ylabel('cpu time [s]');
% xlabel('time [s]');
% 
% %%

figure('color','w','name','Sim. timing'); hold on; grid on; box on;
idx_ = find(nmpc_executed);
area(time(idx_),tsim(idx_)*1000,'FaceColor',cmap(1,:));
area(time(idx_),tsolve(idx_)*1000,'FaceColor',cmap(2,:));
area(time(idx_),tarray(idx_)*1000,'FaceColor',cmap(3,:));
area(time(idx_),trec(idx_)*1000,'FaceColor',cmap(4,:));

legend('total sim.','solve','array allo.','rec.');
xlabel('time [s]');
ylabel('cpu time [ms]');

