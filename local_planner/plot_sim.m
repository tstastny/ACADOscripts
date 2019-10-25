
cmap = lines(7);
color_ref = [0.5 0.5 0.5];
color_hor = [min(ones(1,3), cmap(5,:)*1.6) 0.2];%[0.2 0.2 1 0.6];
color_state = cmap(1,:);

%%

if (plot_opt.position)
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

end

if (plot_opt.position2)
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
hor_int = round(2/Ts);
node_int = 10;
hor1 = plot3(rec.x_hor(1:node_int:end,isp:hor_int:iep,2), rec.x_hor(1:node_int:end,isp:hor_int:iep,1), ...
    -rec.x_hor(1:node_int:end,isp:hor_int:iep,3), '-o', 'MarkerSize', 4);

% position
plot3(rec.x(isp:iep,2),rec.x(isp:iep,1),-rec.x(isp:iep,3), 'linewidth', 1.5, 'color', color_state);

xlabel('East [m]')
ylabel('North [m]')
zlabel('Height [m]');

end

if (plot_opt.attitude)
%% ////////////////////////////////////////////////////////////////////////
% ATTITUDE

figure('color','w','name','Attitude');

% roll angle
hand_att(1) = subplot(3,1,1); hold on; grid on; box on;
plot(time(isp:iep),rad2deg(rec.u_ref(isp:iep,2)),'color',color_ref,'linestyle','-.');
plot(time(isp:iep),rad2deg(rec.u(isp:iep,2)),'color',color_ref);
plot(time(isp:iep),rad2deg(rec.x(isp:iep,7)),'color',color_state);
legend('guidance','ref','state')
ylabel('\phi [deg]')

% pitch
hand_att(2) = subplot(3,1,2); hold on; grid on; box on;
plot(time(isp:iep),rad2deg(rec.u_ref(isp:iep,3)),'color',color_ref,'linestyle','-.');
plot(time(isp:iep),rad2deg(rec.u(isp:iep,3)),'color',color_ref);
plot(time(isp:iep),rad2deg(rec.x(isp:iep,8)),'color',color_state);
legend('guidance','ref','state')
ylabel('\theta [deg]')

% heading
hand_att(3) = subplot(3,1,3); hold on; grid on; box on;
plot(time(isp:iep),rad2deg(rec.x(isp:iep,6)),'color',color_state);
ylabel('\xi [deg]')

xlabel('Time [s]')

linkaxes(hand_att,'x')

end

if (plot_opt.roll_horizon)
%% ////////////////////////////////////////////////////////////////////////
% ATTITUDE HORIZON (ROLL)

figure('color','w','name','Attitude | Roll');

% roll
node_int = 1;
k_int = round(Ts_nmpc/Ts*10);
idx_k = isp:k_int:iep;

hand_att_roll(1) = subplot(3,1,1); hold on; grid on; box on;
stairs(time(isp:iep),rad2deg(rec.u_ref(isp:iep,2)),'color',color_ref);
for ii=1:length(idx_k)
    tt = time(idx_k(ii))+(0:Ts_step:(N-1)*Ts_step);
    plot(tt,rad2deg(rec.u_ref_hor(1:node_int:end,idx_k(ii),2)));
end
ylabel('\phi_{guidance} [deg]')

hand_att_roll(2) = subplot(3,1,2); hold on; grid on; box on;
stairs(time(isp:iep),rad2deg(rec.u(isp:iep,2)),'color',color_ref);
for ii=1:length(idx_k)
    tt = time(idx_k(ii))+(0:Ts_step:(N-1)*Ts_step);
    plot(tt,rad2deg(rec.u_hor(1:node_int:end,idx_k(ii),2)));
end
ylabel('\phi_{ref} [deg]')

hand_att_roll(3) = subplot(3,1,3); hold on; grid on; box on;
plot(time(isp:iep),rad2deg(rec.x(isp:iep,7)),'color',color_ref);
for ii=1:length(idx_k)
    tt = time(idx_k(ii))+(0:Ts_step:(N)*Ts_step);
    plot(tt,rad2deg(rec.x_hor(1:node_int:end,idx_k(ii),7)));
end
ylabel('\phi [deg]')

xlabel('Time [s]')

linkaxes(hand_att_roll,'x')

end

if (plot_opt.pitch_horizon)
%% ////////////////////////////////////////////////////////////////////////
% ATTITUDE HORIZON (PITCH)

figure('color','w','name','Attitude | Pitch');

% pitch
node_int = 1;
k_int = round(Ts_nmpc/Ts*10);
idx_k = isp:k_int:iep;

% ^ use nmpc_executed array?

hand_att_pitch(1) = subplot(3,1,1); hold on; grid on; box on;
stairs(time(isp:iep),rad2deg(rec.u_ref(isp:iep,3)),'color',color_ref);
for ii=1:length(idx_k)
    tt = time(idx_k(ii))+(0:Ts_step:(N-1)*Ts_step);
    plot(tt,rad2deg(rec.u_ref_hor(1:node_int:end,idx_k(ii),3)));
end
ylabel('\theta_{guidance} [deg]')

hand_att_pitch(2) = subplot(3,1,2); hold on; grid on; box on;
stairs(time(isp:iep),rad2deg(rec.u(isp:iep,3)),'color',color_ref);
for ii=1:length(idx_k)
    tt = time(idx_k(ii))+(0:Ts_step:(N-1)*Ts_step);
    plot(tt,rad2deg(rec.u_hor(1:node_int:end,idx_k(ii),3)));
end
ylabel('\theta_{ref} [deg]')

hand_att_pitch(3) = subplot(3,1,3); hold on; grid on; box on;
plot(time(isp:iep),rad2deg(rec.x(isp:iep,8)),'color',color_ref);
for ii=1:length(idx_k)
    tt = time(idx_k(ii))+(0:Ts_step:(N)*Ts_step);
    plot(tt,rad2deg(rec.x_hor(1:node_int:end,idx_k(ii),8)));
end
ylabel('\theta [deg]')

xlabel('Time [s]')

linkaxes(hand_att_pitch,'x')

end

if (plot_opt.motor)
%% ////////////////////////////////////////////////////////////////////////
% MOTOR

figure('color','w','name','Motor');

% u_T
hand_mot(1) = subplot(2,1,1); hold on; grid on; box on;
plot(time(isp:iep),rec.u_ref(isp:iep,1),'color',color_ref,'linestyle','-.');
plot(time(isp:iep),rec.u(isp:iep,1),'color',color_ref);
legend('guidance','ref');
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

end

if (plot_opt.position_errors)
%% ////////////////////////////////////////////////////////////////////////
% POSITION ERRORS
figure('color','w','name','Position errors')

hand_pos_err(1)=subplot(2,1,1); hold on; grid on; box on;

plot(rec.time_nmpc(isp_nmpc:iep_nmpc),rec.aux(1,isp_nmpc:iep_nmpc,AUX_E_LAT));
plot(rec.time_nmpc(isp_nmpc:iep_nmpc),rec.aux(1,isp_nmpc:iep_nmpc,AUX_E_LON));

legend('e_{lat}','e_{lon}')
ylabel('e [m]')

hand_pos_err(2)=subplot(2,1,2); hold on; grid on; box on;

plot(rec.time_nmpc(isp_nmpc:iep_nmpc),rec.aux(1,isp_nmpc:iep_nmpc,AUX_H_TERR),'color',color_ref);
plot(rec.time_nmpc(isp_nmpc:iep_nmpc),rec.aux(1,isp_nmpc:iep_nmpc,AUX_H_TERR)+h_offset,'color',color_ref,'linestyle','--');
plot(rec.time_nmpc(isp_nmpc:iep_nmpc),rec.aux(1,isp_nmpc:iep_nmpc,AUX_H_TERR)+h_offset+delta_h,'color',color_ref,'linestyle',':');
plot(time(isp:iep),-rec.x(isp:iep,3),'color',color_state);

legend('h_{terr}','h_{terr,off}','h_{terr}+h_{terr,off}+\Delta_h','h')
ylabel('Height [m]')

xlabel('time [s]')

linkaxes(hand_pos_err,'x');

end

if (plot_opt.velocity_tracking)
%% ////////////////////////////////////////////////////////////////////////
% VELOCITY TRACKING

figure('color','w','name','Unit velocity tracking')

cos_gamma = cos(rec.x(isp:iep,5)); 
vG_n = rec.x(isp:iep,4).*cos_gamma.*cos(rec.x(isp:iep,6)) + w_n; 
vG_e = rec.x(isp:iep,4).*cos_gamma.*sin(rec.x(isp:iep,6)) + w_e; 
vG_d = rec.x(isp:iep,4).*-sin(rec.x(isp:iep,5)) + w_d;
vG_norm = sqrt(vG_n.^2+vG_e.^2+vG_d.^2);
one_over_vG_norm = 1./vG_norm;
one_over_vG_norm(vG_norm<0.01) = 100;
vG_n_unit = vG_n .* one_over_vG_norm;
vG_e_unit = vG_e .* one_over_vG_norm;
vG_d_unit = vG_d .* one_over_vG_norm;

% VGN - - - - 
hand_vel(1)=subplot(7,1,1:2); hold on; grid on; box on;

plot(rec.time_nmpc(isp_nmpc:iep_nmpc),rec.yref(1,isp_nmpc:iep_nmpc,1),'color',color_ref);
plot(time(isp:iep),vG_n_unit,'color',color_state);

ylabel('$\hat{v}_{G_n}$','interpreter','latex');

% VGE - - - - 
hand_vel(2)=subplot(7,1,3:4); hold on; grid on; box on;

plot(rec.time_nmpc(isp_nmpc:iep_nmpc),rec.yref(1,isp_nmpc:iep_nmpc,2),'color',color_ref);
plot(time(isp:iep),vG_e_unit,'color',color_state);

ylabel('$\hat{v}_{G_e}$','interpreter','latex');

% VGD - - - - 
hand_vel(3)=subplot(7,1,5:6); hold on; grid on; box on;

plot(rec.time_nmpc(isp_nmpc:iep_nmpc),rec.yref(1,isp_nmpc:iep_nmpc,3),'color',color_ref);
plot(time(isp:iep),vG_d_unit,'color',color_state);

ylabel('$\hat{v}_{G_d}$','interpreter','latex');

% PRIO - - - - 
hand_vel(4)=subplot(7,1,7); hold on; grid on; box on;

plot(rec.time_nmpc(isp_nmpc:iep_nmpc), rec.priorities(1,isp_nmpc:iep_nmpc,2));
plot(rec.time_nmpc(isp_nmpc:iep_nmpc), rec.priorities(1,isp_nmpc:iep_nmpc,3));
plot(rec.time_nmpc(isp_nmpc:iep_nmpc), rec.priorities(1,isp_nmpc:iep_nmpc,2) .* ...
    rec.priorities(1,isp_nmpc:iep_nmpc,3));

ylabel('prio');
legend('{prio}_h','{prio}_r','{prio}_h*{prio}_r');

xlabel('Time [s]');

end

if (plot_opt.wind_axis)
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

end

if (plot_opt.radial_cost)
%% ////////////////////////////////////////////////////////////////////////
% RADIAL COST

k_int = round(Ts_nmpc/Ts);
idx_k = isp:k_int:iep;

cos_gamma = cos(rec.x(isp:iep,5)); 
vG_n = rec.x(isp:iep,4).*cos_gamma.*cos(rec.x(isp:iep,6)) + w_n; 
vG_e = rec.x(isp:iep,4).*cos_gamma.*sin(rec.x(isp:iep,6)) + w_e; 
vG_d = rec.x(isp:iep,4).*-sin(rec.x(isp:iep,5)) + w_d;
vG_vec = [vG_e(idx_k), vG_n(idx_k), -vG_d(idx_k)];
vG_norm = sqrt(sum(vG_vec.^2,2));
v_ray_fwd = vG_vec./vG_norm;
v_ray_left = [-vG_n(idx_k), vG_e(idx_k), zeros(length(vG_n(idx_k)),1)]./vG_norm;
v_ray_right = [vG_n(idx_k), -vG_e(idx_k), zeros(length(vG_n(idx_k)),1)]./vG_norm;

vrel_fwd = dot(v_ray_fwd, vG_vec, 2);
vrel_fwd(vrel_fwd < 0) = 0;
vrel_left = dot(v_ray_left, vG_vec, 2);
vrel_left(vrel_left < 0) = 0;
vrel_right = dot(v_ray_right, vG_vec, 2);
vrel_right(vrel_right < 0) = 0;

Rmin_fwd_offset = vrel_fwd.^2 .* k_r_offset;
Rmin_fwd_delta = vrel_fwd.^2 .* k_delta_r;
Rmin_left_offset = vrel_left.^2 .* k_r_offset;
Rmin_left_delta = vrel_left.^2 .* k_delta_r;
Rmin_right_offset = vrel_right.^2 .* k_r_offset;
Rmin_right_delta = vrel_right.^2 .* k_delta_r;

figure('color','w','name','Radial cost');

% r fwd
hand_r(1) = subplot(3,1,1); hold on; grid on; box on;

r_occ = rec.aux(1,isp_nmpc:iep_nmpc,AUX_R_OCC_FWD);
r_occ(rec.aux(1,isp_nmpc:iep_nmpc,AUX_OCC_DETECT_FWD)==0) = NaN;

plot(time([isp iep]),[r_offset r_offset],'color',color_ref);
plot(rec.time_nmpc(isp_nmpc:iep_nmpc), Rmin_fwd_offset + r_offset,'color',color_ref,'linestyle','--');
plot(rec.time_nmpc(isp_nmpc:iep_nmpc), Rmin_fwd_offset + r_offset + Rmin_fwd_delta + delta_r0,'color',color_ref,'linestyle',':');
plot(rec.time_nmpc(isp_nmpc:iep_nmpc), r_occ,'color',color_state);
stairs(rec.time_nmpc(isp_nmpc:iep_nmpc),rec.aux(1,isp_nmpc:iep_nmpc,AUX_OCC_DETECT_FWD), ...
    'color',cmap(2,:)*0.3+0.7*ones(1,3));

ylabel('r (fwd) [m]');

legend('r_{{offset}}','r_{{offset}_1}','r_{{offset}_1}+\Delta_r','r_{occ}','DETECT');

% r left
hand_r(2) = subplot(3,1,2); hold on; grid on; box on;

r_occ = rec.aux(1,isp_nmpc:iep_nmpc,AUX_R_OCC_LEFT);
r_occ(rec.aux(1,isp_nmpc:iep_nmpc,AUX_OCC_DETECT_LEFT)==0) = NaN;

plot(time([isp iep]),[r_offset r_offset],'color',color_ref);
plot(rec.time_nmpc(isp_nmpc:iep_nmpc), Rmin_left_offset + r_offset,'color',color_ref,'linestyle','--');
plot(rec.time_nmpc(isp_nmpc:iep_nmpc), Rmin_left_offset + r_offset + Rmin_left_delta + delta_r0,'color',color_ref,'linestyle',':');
plot(rec.time_nmpc(isp_nmpc:iep_nmpc), r_occ,'color',color_state);
stairs(rec.time_nmpc(isp_nmpc:iep_nmpc),rec.aux(1,isp_nmpc:iep_nmpc,AUX_OCC_DETECT_LEFT), ...
    'color',cmap(2,:)*0.3+0.7*ones(1,3));

ylabel('r (left) [m]');

% r right
hand_r(3) = subplot(3,1,3); hold on; grid on; box on;

r_occ = rec.aux(1,isp_nmpc:iep_nmpc,AUX_R_OCC_RIGHT);
r_occ(rec.aux(1,isp_nmpc:iep_nmpc,AUX_OCC_DETECT_RIGHT)==0) = NaN;

plot(time([isp iep]),[r_offset r_offset],'color',color_ref);
plot(rec.time_nmpc(isp_nmpc:iep_nmpc), Rmin_right_offset + r_offset,'color',color_ref,'linestyle','--');
plot(rec.time_nmpc(isp_nmpc:iep_nmpc), Rmin_right_offset + r_offset + Rmin_right_delta + delta_r0,'color',color_ref,'linestyle',':');
plot(rec.time_nmpc(isp_nmpc:iep_nmpc), r_occ,'color',color_state);
stairs(rec.time_nmpc(isp_nmpc:iep_nmpc),rec.aux(1,isp_nmpc:iep_nmpc,AUX_OCC_DETECT_RIGHT), ...
    'color',cmap(2,:)*0.3+0.7*ones(1,3));

ylabel('r (right) [m]');

end

if (plot_opt.priorities)
%% ////////////////////////////////////////////////////////////////////////
% PRIORITIZATION

figure('color','w','name','Terrain height prioritization');

surf(rec.time_nmpc(isp_nmpc:iep_nmpc), (1:N+1)', rec.priorities(:,:,2));

xlabel('Time [s]');
ylabel('Node');
ylim([1 N+1]);
zlabel('{prio}_h');
zlim([0 1]);

%%

figure('color','w','name','Radial terrain prioritization');

surf(rec.time_nmpc(isp_nmpc:iep_nmpc), (1:N+1)', rec.priorities(:,:,3));

xlabel('Time [s]');
ylabel('Node');
ylim([1 N+1]);
zlabel('{prio}_r');
zlim([0 1]);

end

if (plot_opt.objectives)
%% ////////////////////////////////////////////////////////////////////////
% OBJECTIVE COSTS

obj_cost = zeros(length(isp_nmpc:iep_nmpc), n_Y+n_Z);
for ii = 1:n_Y
    obj_cost(:,ii) = rec.W(isp_nmpc:iep_nmpc,ii) .* ...
        (rec.yref(1,isp_nmpc:iep_nmpc,ii)' - rec.y(1,isp_nmpc:iep_nmpc,ii)').^2;
end

for ii = 1:n_Z
    obj_cost(:,n_Y+ii) = rec.W(isp_nmpc:iep_nmpc,n_Y+ii) .* ...
        (rec.yref(1,isp_nmpc:iep_nmpc,n_Y+ii)' - rec.y(1,isp_nmpc:iep_nmpc,n_Y+ii)').^2;
end

leg_str_ = cell(1,n_Y+n_Z);
for ii = 1:n_Y
    leg_str_{ii} = ['y',int2str(ii)];
end
for ii = 1:n_Z
    leg_str_{n_Y+ii} = ['z',int2str(ii)];
end

figure('color','w','name','Objective Costs');

hand_obj(1)=subplot(2,2,1); hold on; grid on; box on;

for ii=1:3
    plot(rec.time_nmpc(isp_nmpc:iep_nmpc),obj_cost(:,ii));
end

legend(hand_obj(1), leg_str_{1:3});
ylabel('J(x)');
ylim([0 max(obj_cost(:))]);

hand_obj(2)=subplot(2,2,2); hold on; grid on; box on;

for ii=4:6
    plot(rec.time_nmpc(isp_nmpc:iep_nmpc),obj_cost(:,ii));
end

legend(hand_obj(2), leg_str_{4:6});
ylabel('J(x)');
ylim([0 max(obj_cost(:))]);

hand_obj(3)=subplot(2,2,3); hold on; grid on; box on;

for ii=7:9
    plot(rec.time_nmpc(isp_nmpc:iep_nmpc),obj_cost(:,ii));
end

legend(hand_obj(3), leg_str_{7:9});
ylabel('J(x)');
ylim([0 max(obj_cost(:))]);

xlabel('time [s]');

hand_obj(4)=subplot(2,2,4); hold on; grid on; box on;

for ii=10:12
    plot(rec.time_nmpc(isp_nmpc:iep_nmpc),obj_cost(:,ii));
end

legend(hand_obj(4), leg_str_{10:12});
ylabel('J(u)');
ylim([0 max(obj_cost(:))]);

xlabel('time [s]');

linkaxes(hand_obj,'x');

end

if (plot_opt.timing)
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

end