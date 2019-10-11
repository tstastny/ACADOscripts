
cmap = lines(7);
color_ref = [0.5 0.5 0.5];
color_hor = [min(ones(1,3), cmap(5,:)*1.6) 0.2];%[0.2 0.2 1 0.6];
color_state = cmap(1,:);

% start/end time
t_st_plot = 0;
t_ed_plot = time(end);
isp = find(time<=t_st_plot, 1, 'last');
iep = find(time<=t_ed_plot, 1, 'last');

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
hor_int = round(1/Ts);
node_int = 10;
hor1 = plot3(rec.x_hor(1:node_int:end,isp:hor_int:iep,2), rec.x_hor(1:node_int:end,isp:hor_int:iep,1), ...
    -rec.x_hor(1:node_int:end,isp:hor_int:iep,3), '-o', 'MarkerSize', 4, 'Color', color_hor(1:3));

% position
plot3(rec.x(isp:iep,2),rec.x(isp:iep,1),-rec.x(isp:iep,3), 'linewidth', 1.5, 'color', cmap(2,:));

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
hor_int = round(3/Ts);
node_int = 10;
hor1 = plot3(rec.x_hor(1:node_int:end,isp:hor_int:iep,2), rec.x_hor(1:node_int:end,isp:hor_int:iep,1), ...
    -rec.x_hor(1:node_int:end,isp:hor_int:iep,3), '-o', 'MarkerSize', 4, 'Color', [0.3 0.3 0.3]);

% position
plot3(rec.x(isp:iep,2),rec.x(isp:iep,1),-rec.x(isp:iep,3), 'linewidth', 1.5, 'color', color_state);

xlabel('East [m]')
ylabel('North [m]')
zlabel('Height [m]');

%% ////////////////////////////////////////////////////////////////////////
% ATTITUDE

figure('color','w','name','Attitude');

% roll angle
hand_att(1) = subplot(3,1,1); hold on; grid on; box on;
plot(time(isp:iep),rad2deg(rec.u(isp:iep,2)),'color',color_ref);
plot(time(isp:iep),rad2deg(rec.x(isp:iep,7)),'color',color_state);
legend('ref','state')
ylabel('\phi [deg]')

% pitch
hand_att(2) = subplot(3,1,2); hold on; grid on; box on;
plot(time(isp:iep),rad2deg(rec.u(isp:iep,3)),'color',color_ref);
plot(time(isp:iep),rad2deg(rec.x(isp:iep,8)),'color',color_state);
legend('ref','state')
ylabel('\theta [deg]')

% heading
hand_att(3) = subplot(3,1,3); hold on; grid on; box on;
plot(time(isp:iep),rad2deg(rec.x(isp:iep,6)),'color',color_state);
ylabel('\xi [deg]')

xlabel('Time [s]')

linkaxes(hand_att,'x')

%% ////////////////////////////////////////////////////////////////////////
% MOTOR

figure('color','w','name','Motor');

% u_T
hand_mot(1) = subplot(2,1,1); hold on; grid on; box on;
plot(time(isp:iep),rec.u(isp:iep,1),'color',color_ref);
ylabel('u_T [~]')

% prop speed
hand_mot(2) = subplot(2,1,2); hold on; grid on; box on;

u_n = calc_prop_speed(rec.u(isp:iep,1),rec.x(isp:iep,4),rec.x(isp:iep,8)-rec.x(isp:iep,5),model_params,sysid_config);

plot(time(isp:iep),u_n,'color',color_ref);
plot(time(isp:iep),rec.x(isp:iep,9),'color',color_state);
legend('ref','state')
ylabel('n_p [rps]')

xlabel('Time [s]')

linkaxes(hand_att,'x')

%% ////////////////////////////////////////////////////////////////////////
% POSITION ERRORS

figure('color','w','name','Position errors')

hand_pos_err(1)=subplot(2,1,1); hold on; grid on; box on;

plot(time(isp:iep),rec.aux(isp:iep,2));
plot(time(isp:iep),rec.aux(isp:iep,3));

legend('e_{lat}','e_{lon}')
ylabel('e [m]')

hand_pos_err(2)=subplot(2,1,2); hold on; grid on; box on;

plot(time(isp:iep),rec.aux(isp:iep,1),'color',color_ref);
plot(time(isp:iep),rec.aux(isp:iep,1)+h_offset,'color',color_ref,'linestyle','--');
plot(time(isp:iep),rec.aux(isp:iep,1)+h_offset+delta_h,'color',color_ref,'linestyle',':');
plot(time(isp:iep),-rec.x(isp:iep,3),'color',color_state);

legend('h_{terr}','h_{terr,off}','h_{terr}+h_{terr,off}+\Delta_h','h')
ylabel('Height [m]')

xlabel('time [s]')

linkaxes(hand_pos_err,'x');

%% ////////////////////////////////////////////////////////////////////////
% DIRECTIONAL ERRORS

figure('color','w','name','Directional errors')

hand_dir_err(1)=subplot(2,1,1); hold on; grid on; box on;

plot(time(isp:iep),rec.aux(isp:iep,4));
plot(time(isp:iep),rec.aux(isp:iep,5));

legend('e_{v_n}','e_{v_e}')
ylabel('e_v [m/s]')

hand_dir_err(2)=subplot(2,1,2); hold on; grid on; box on;

plot(time(isp:iep),rec.aux(isp:iep,6));

ylabel('e_{v_d} [m/s]')

%% ////////////////////////////////////////////////////////////////////////
% VELOCITY TRACKING

figure('color','w','name','Velocity tracking')

v_cos_gamma = rec.x(isp:iep,4).*cos(rec.x(isp:iep,5)); 
vG_n = v_cos_gamma.*cos(rec.x(isp:iep,6)) + w_n; 
vG_e = v_cos_gamma.*sin(rec.x(isp:iep,6)) + w_e; 
vG_d = -rec.x(isp:iep,4).*sin(rec.x(isp:iep,5)) + w_d;

% VGN - - - - 
hand_vel(1)=subplot(7,1,1:2); hold on; grid on; box on;

% vP
plot(time(isp:iep),rec.aux(isp:iep,35),'color',cmap(5,:),'linestyle',':');

% vP app
plot(time(isp:iep),rec.aux(isp:iep,26),'color',cmap(4,:),'linestyle','--');

% v_occ
plot(time(isp:iep),rec.aux(isp:iep,29),'color',cmap(2,:),'linestyle','--');

% vP * sig_r + v_occ (1 - sig_r)
plot(time(isp:iep),rec.aux(isp:iep,32),'color',color_ref);

% v
plot(time(isp:iep),vG_n,'color',color_state);

legend('v_P','v_{P_{app}}','v_{occ}','v_{cmd}','v_G');
ylabel('v_{G_n} [m/s]');

% VGE - - - - 
hand_vel(2)=subplot(7,1,3:4); hold on; grid on; box on;

% vP
plot(time(isp:iep),rec.aux(isp:iep,36),'color',cmap(5,:),'linestyle',':');

% vP app
plot(time(isp:iep),rec.aux(isp:iep,27),'color',cmap(4,:),'linestyle','--');

% v_occ
plot(time(isp:iep),rec.aux(isp:iep,30),'color',cmap(2,:),'linestyle','--');

% vP * sig_r + v_occ (1 - sig_r)
plot(time(isp:iep),rec.aux(isp:iep,33),'color',color_ref);

% v
plot(time(isp:iep),vG_e,'color',color_state);

legend('v_P','v_{P_{app}}','v_{occ}','v_{cmd}','v_G');
ylabel('v_{G_e} [m/s]');

% VGD - - - - 
hand_vel(3)=subplot(7,1,5:6); hold on; grid on; box on;

% vP
plot(time(isp:iep),rec.aux(isp:iep,37),'color',cmap(5,:),'linestyle',':');

% vP app
plot(time(isp:iep),rec.aux(isp:iep,28),'color',cmap(4,:),'linestyle','--');

% v_occ
plot(time(isp:iep),rec.aux(isp:iep,31),'color',cmap(2,:),'linestyle','--');

% vP * sig_r + v_occ (1 - sig_r)
plot(time(isp:iep),rec.aux(isp:iep,34),'color',color_ref);

% v
plot(time(isp:iep),vG_d,'color',color_state);

legend('v_P','v_{P_{app}}','v_{occ}','v_{cmd}','v_G');
ylabel('v_{G_d} [m/s]');

% PRIO - - - - 
hand_vel(4)=subplot(7,1,7); hold on; grid on; box on;

plot(time(isp:iep),rec.aux(isp:iep,25));

ylabel('{prio}_r');

xlabel('Time [s]');

%% ////////////////////////////////////////////////////////////////////////
% WIND AXIS

figure('color','w','name','Wind axis');

% airspeed
hand_vab(1) = subplot(2,1,1); hold on; grid on; box on;
plot(time([isp iep]),[v_ref v_ref],'color',color_ref);
plot(time(isp:iep),rec.x(isp:iep,4),'color',color_state);
legend('ref','state')
ylabel('v [m/s]')

% angle of attack
hand_vab(2) = subplot(2,1,2); hold on; grid on; box on;
plot(time([isp iep]),rad2deg([aoa_m aoa_m]),'color',color_ref);
plot(time([isp iep]),rad2deg([aoa_p aoa_p]),'color',color_ref);
plot(time(isp:iep),rad2deg(rec.x(isp:iep,8) - rec.x(isp:iep,5)),'color',color_state);
ylabel('\alpha [deg]')

%% ////////////////////////////////////////////////////////////////////////
% RADIAL COST

figure('color','w','name','Radial cost');

% r
hand_r(1) = subplot(5,1,1:2); hold on; grid on; box on;
plot(time(isp:iep),rec.aux(isp:iep,24),'color',color_ref);
plot(time(isp:iep),rec.aux(isp:iep,38).*double(rec.aux(isp:iep,16)>0),'color',color_state);
legend('\Delta_r','d_{occ}')
ylabel('r [m]')

% angle of attack
hand_r(2) = subplot(5,1,3:4); hold on; grid on; box on;
plot(time(isp:iep),rec.aux(isp:iep,17),'color',color_ref);
ylabel('\sigma_r')

hand_r(3) = subplot(5,1,5); hold on; grid on; box on;
plot(time(isp:iep),rec.aux(isp:iep,16),'color',color_state);
ylabel('Detect')


%% ////////////////////////////////////////////////////////////////////////
% OBJECTIVE COSTS

obj_cost = zeros(length(isp:iep), n_Y+n_Z);
for ii = 1:n_Y
    obj_cost(:,ii) = (yref(ii)*ones(length(time(isp:iep)),1)-rec.yz(isp:iep,ii)).^2*Q_output(ii);
end

for ii = 1:n_Z
    obj_cost(:,n_Y+ii) = (zref(ii)*ones(length(time(isp:iep)),1)-rec.yz(isp:iep,n_Y+ii)).^2*R_controls(ii);
end

figure('color','w','name','Objective Costs')

hand_obj(1)=subplot(2,1,1); hold on; grid on; box on;

leg_str_ = cell(1,n_Y);
for ii = 1:n_Y
    plot(time(isp:iep),obj_cost(:,ii));
    leg_str_{ii} = ['y',int2str(ii)];
end

legend(hand_obj(1), leg_str_);
ylabel('J(x)');
ylim([0 max(obj_cost(:))]);

hand_obj(2)=subplot(2,1,2); hold on; grid on; box on;

leg_str_ = cell(1,n_Z);
for ii = 1:n_Z
    plot(time(isp:iep),obj_cost(:,n_Y+ii));
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
% plot(time(isp:iep),tsolve);
% plot(time(isp:iep),tarray);
% 
% legend('nmpc','array allocation');
% ylabel('cpu time [s]');
% xlabel('time [s]');
% 
% %%

figure('color','w','name','Sim. timing');

subplot(5,1,1:4); hold on; grid on; box on;

idx_ = find(nmpc_executed);
area(time(idx_),tsim(idx_)*1000,'FaceColor',cmap(1,:));
area(time(idx_),tsolve(idx_)*1000,'FaceColor',cmap(2,:));
area(time(idx_),tarray(idx_)*1000,'FaceColor',cmap(3,:));
area(time(idx_),trec(idx_)*1000,'FaceColor',cmap(4,:));

legend('total sim.','solve','array allo.','rec.');
ylabel('cpu time [ms]');

subplot(5,1,5); hold on; grid on; box on;

qp_iter = [INFO_MPC.QP_iter];
plot(time(idx_), qp_iter(1:length(idx_)));
ylabel('qp iter.');

xlabel('time [s]');
