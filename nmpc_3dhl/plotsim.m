
horiz_disp = true;
horiz_disp_int = 100; % every Nth record

horiz_time = (repmat(time(1:horiz_disp_int:end),N+1,1)+Ts_step*repmat((0:N)',1,length(time(1:horiz_disp_int:end))));

r2d = 180/pi;

c_horiz = [0.5 0.8 0.8];
c_ref = [0.5 0.5 0.5];
c_state = [0 0 0];

lnwidth_horiz=0.5;
alpha_horiz=0.5;

%% /////////////////////////////////////////////////////////////////////////
% POSITION

figure('color','w','name','Position')
hold on; grid on; %axis equal;
ltset=1000;
hl = gobjects(0);
for k = 1:length(paths)

    if paths(k).pparam1 == 0
        hl(1) = plot3([paths(k).pparam3 paths(k).pparam6], ...
            [paths(k).pparam2 paths(k).pparam5], ...
            -[paths(k).pparam4 paths(k).pparam7], ...
            'color',c_ref,'linewidth',1);
    elseif paths(k).pparam1 == 1
        tset = linspace(paths(k).pparam8, ...
            paths(k).pparam8 + paths(k).pparam9 * paths(k).pparam6, ...
            ltset)';
        r = repmat([paths(k).pparam2,paths(k).pparam3,paths(k).pparam4],ltset,1) + ...
            [paths(k).pparam5 * cos(tset), paths(k).pparam5 * sin(tset), ...
            -paths(k).pparam6 * (tset-tset(1)) * paths(k).pparam5 * tan(paths(k).pparam7)];
        hl(1) = plot3(r(:,2),r(:,1),-r(:,3),'color',c_ref,'linewidth',1);
    end
    
end
% guidance
guide_disp_int=100;
if true
    plot3([J_rec(1:guide_disp_int:end,9) X_rec(1:guide_disp_int:end,2)]',...
        [J_rec(1:guide_disp_int:end,8) X_rec(1:guide_disp_int:end,1)]',...
        [-J_rec(1:guide_disp_int:end,10) -X_rec(1:guide_disp_int:end,3)]','r')
    plot3(J_rec(1:guide_disp_int:end,9),J_rec(1:guide_disp_int:end,8),-J_rec(1:guide_disp_int:end,10),'ro')
    quiver3(J_rec(1:guide_disp_int:end,9),J_rec(1:guide_disp_int:end,8),-J_rec(1:guide_disp_int:end,10),...
        J_rec(1:guide_disp_int:end,12),J_rec(1:guide_disp_int:end,11),-J_rec(1:guide_disp_int:end,13),'c')
end
% position horizons
if horiz_disp
    hl(length(hl)+1) = plot3(horiz_rec(:,1,2), ...
        horiz_rec(:,1,1), ...
        -horiz_rec(:,1,3), ...
        '-', 'color', c_horiz);
    plot3(horiz_rec(:,horiz_disp_int:horiz_disp_int:end,2), ...
        horiz_rec(:,horiz_disp_int:horiz_disp_int:end,1), ...
        -horiz_rec(:,horiz_disp_int:horiz_disp_int:end,3), ...
        '-', 'color', c_horiz);
%     hl(length(hl)+1) = patchline(horiz_rec(:,1,2),horiz_rec(:,1,1),-horiz_rec(:,1,3), ...
%         'edgecolor',c_horiz,'linewidth',lnwidth_horiz,'edgealpha',alpha_horiz);
%     patchline(horiz_rec(:,horiz_disp_int:horiz_disp_int:end,2), ...
%         horiz_rec(:,horiz_disp_int:horiz_disp_int:end,1), ...
%         -horiz_rec(:,horiz_disp_int:horiz_disp_int:end,3), ...
%         'edgecolor',c_horiz,'linewidth',lnwidth_horiz,'edgealpha',alpha_horiz);
end
hl(length(hl)+1) = plot3(X_rec(:,2),X_rec(:,1),-X_rec(:,3),'color',c_state,'linewidth',1.5);
xlabel('East [m]')
ylabel('North [m]')
zlabel('Height [m]');
if horiz_disp
    legend(hl,{'path','horizon','position'});
else
    legend(hl,{'path','position'});
end
% view(10,60)

%% /////////////////////////////////////////////////////////////////////////
% COSTS

figure('color','w','name','Costs')
hc(1)=subplot(4,1,1); hold on; grid on;
plot(time,J_rec(:,1));
plot(time,J_rec(:,2));
% plot(time,J_rec(:,1));
% plot(time,J_rec(:,2));
legend('e_{T_{ne}}','e_{T_{d}}')
ylabel('e [m]')

hc(2)=subplot(4,1,2:4); hold on; grid on;

plot(time,J_rec(:,3).^2*Q_output(1)); %e_t_1_ne
plot(time,J_rec(:,4).^2*Q_output(2)); %e_t_1_d
plot(time,(yref(3)*ones(length(time),1)-X_rec(:,13)).^2*Q_output(3)); % i_e_t

plot(time,J_rec(:,5).^2*Q_output(3)); %e_v1
plot(time,J_rec(:,6).^2*Q_output(4)); %e_v2
plot(time,J_rec(:,7).^2*Q_output(5)); %e_v3

plot(time,(yref(7)*ones(length(time),1)-X_rec(:,4)).^2*Q_output(7)); % e_V
plot(time,(yref(8)*ones(length(time),1)-X_rec(:,9)).^2*Q_output(8)); % e_p
plot(time,(yref(9)*ones(length(time),1)-X_rec(:,10)).^2*Q_output(9)); % e_q
plot(time,(yref(10)*ones(length(time),1)-X_rec(:,11)).^2*Q_output(10)); % e_r

% plot(time,J_rec(:,4).^2*R_controls(4)); %Delta_T
% plot(time,J_rec(:,4).^2*R_controls(5)); %Delta_phi
% plot(time,J_rec(:,4).^2*R_controls(6)); %Delta_theta

legend('e_{T^1_{ne}}','e_{T^1_{d}}','i_{e_{T}}','e_{v_{n}}','e_{v_{e}}','e_{v_{d}}','e_V','e_p','e_q','e_r')
ylabel('J(e)')
xlabel('time [s]')

linkaxes(hc,'x');

%% /////////////////////////////////////////////////////////////////////////
% AIRSPEED + THROTTLE

figure('color','w','name','Airspeed & Throttle')

handle_vdt(1) = subplot(3,1,1); hold on; grid on;
hl = gobjects(0);
hl(length(hl)+1) = plot(time,yref(7)*ones(1,length(time)),'color',c_ref);
hl(length(hl)+1) = plot(time,X_rec(:,4),'color',c_state);
ylabel('V [m/s]');
legend(hl,{'ref','state'});

handle_vdt(2) = subplot(3,1,2); hold on; grid on;
hl = gobjects(0);
hl(length(hl)+1) = plot(time,alpha_p_co*ones(1,length(time))*r2d,'color',c_ref);
plot(time,alpha_m_co*ones(1,length(time))*r2d,'color',c_ref);
hl(length(hl)+1) = plot(time,(X_rec(:,8)-X_rec(:,5))*r2d,'color',c_state);
ylabel('\alpha [deg]');
legend(hl,{'cut-off','state'});

handle_vdt(3) = subplot(3,1,3); hold on; grid on;
hl = gobjects(0);
if horiz_disp
    hl(length(hl)+1) = plot(horiz_time(:,1), ...
        horiz_rec(:,1,12), ...
        '-', 'color', c_horiz);
    plot(horiz_time(:,2:end), ...
        horiz_rec(:,horiz_disp_int:horiz_disp_int:end,12), ...
        '-', 'color', c_horiz);
%     hl(length(hl)+1) = plot(horiz_time(1:end-1,1), ...
%         horiz_rec_U(:,1,1), ...
%         '-', 'color', c_horiz);
%     plot(horiz_time(1:end-1,2:end), ...
%         horiz_rec_U(:,horiz_disp_int:horiz_disp_int:end,1), ...
%         '-', 'color', c_horiz);
end
hl(length(hl)+1) = plot(time,U_rec(:,1),'color',c_ref);
hl(length(hl)+1) = plot(time,X_rec(:,12),'color',c_state);
ylabel('\delta_T [%]');
if horiz_disp
    legend(hl,{'horizon','ref','state'});
else
    legend(hl,{'ref','state'});
end

xlabel('time [s]')
linkaxes(handle_vdt,'x')

%% /////////////////////////////////////////////////////////////////////////
% ATTITUDE

figure('color','w','name','Attitude')

handle_att(1) = subplot(3,1,1); hold on; grid on;
hl = gobjects(0);
if horiz_disp
%     hl(length(hl)+1) = plot(horiz_time(:,1), ...
%         horiz_rec(:,1,7)*r2d, ...
%         '-', 'color', c_horiz);
%     plot(horiz_time(:,2:end), ...
%         horiz_rec(:,horiz_disp_int:horiz_disp_int:end,7)*r2d, ...
%         '-', 'color', c_horiz);
    hl(length(hl)+1) = plot(horiz_time(1:end-1,1), ...
        horiz_rec_U(:,1,2)*r2d, ...
        '-', 'color', c_horiz);
    plot(horiz_time(1:end-1,2:end), ...
        horiz_rec_U(:,horiz_disp_int:horiz_disp_int:end,2)*r2d, ...
        '-', 'color', c_horiz);
end
hl(length(hl)+1) = plot(time,U_rec(:,2)*r2d,'color',c_ref);
hl(length(hl)+1) = plot(time,X_rec(:,7)*r2d,'color',c_state);
ylabel('\phi [deg]');
if horiz_disp
    legend(hl,{'horizon','ref','state'});
else
    legend(hl,{'ref','state'});
end

handle_att(2) = subplot(3,1,2); hold on; grid on;
hl = gobjects(0);
if horiz_disp
%     hl(length(hl)+1) = plot(horiz_time(:,1), ...
%         horiz_rec(:,1,8)*r2d, ...
%         '-', 'color', c_horiz);
%     plot(horiz_time(:,2:end), ...
%         horiz_rec(:,horiz_disp_int:horiz_disp_int:end,8)*r2d, ...
%         '-', 'color', c_horiz);
    hl(length(hl)+1) = plot(horiz_time(1:end-1,1), ...
        horiz_rec_U(:,1,3)*r2d, ...
        '-', 'color', c_horiz);
    plot(horiz_time(1:end-1,2:end), ...
        horiz_rec_U(:,horiz_disp_int:horiz_disp_int:end,3)*r2d, ...
        '-', 'color', c_horiz);
end
hl(length(hl)+1) = plot(time,U_rec(:,3)*r2d,'color',c_ref);
hl(length(hl)+1) = plot(time,X_rec(:,8)*r2d,'color',c_state);
ylabel('\theta [deg]');
if horiz_disp
    legend(hl,{'horizon','ref','state'});
else
    legend(hl,{'ref','state'});
end

handle_att(3) = subplot(3,1,3); hold on; grid on;
plot(time,J_rec(:,5));
plot(time,J_rec(:,6));
plot(time,J_rec(:,7));
ylabel('e_v [~]');
legend('e_{v_n}','e_{v_e}','e_{v_d}');

xlabel('time [s]')
linkaxes(handle_att,'x')

%% /////////////////////////////////////////////////////////////////////////
% RATES

figure('color','w','name','Rates')

handle_rates(1) = subplot(3,1,1); hold on; grid on;
hl = gobjects(0);
hl(length(hl)+1) = plot(time,X_rec(:,9)*r2d,'color',c_state);
ylabel('p [deg/s]');

handle_rates(2) = subplot(3,1,2); hold on; grid on;
hl = gobjects(0);
hl(length(hl)+1) = plot(time,X_rec(:,10)*r2d,'color',c_state);
ylabel('q [deg/s]');

handle_rates(3) = subplot(3,1,3); hold on; grid on;
hl = gobjects(0);
hl(length(hl)+1) = plot(time,X_rec(:,11)*r2d,'color',c_state);
ylabel('r [deg/s]');

xlabel('time [s]')
linkaxes(handle_rates,'x')

%% /////////////////////////////////////////////////////////////////////////
% AUXILIARY

figure('color','w','name','Auxiliary')

handle_aux(1) = subplot(3,1,1); hold on; grid on;
hl = gobjects(0);
hl(length(hl)+1) = plot(time,tsolve*1000,'color',c_state);
ylabel('t_{solve} [ms]');

handle_aux(2) = subplot(3,1,2); hold on; grid on;
hl = gobjects(0);
hl(length(hl)+1) = plot(time,X_rec(:,14),'color',c_state);
ylabel('i_{e_T} [~]');

handle_aux(3) = subplot(3,1,3); hold on; grid on;
hl = gobjects(0);
% if horiz_disp
%     hl(length(hl)+1) = plot(horiz_time(:,1), ...
%         horiz_rec(:,1,8)*r2d, ...
%         '-', 'color', c_horiz);
%     plot(horiz_time(:,2:end), ...
%         horiz_rec(:,horiz_disp_int:horiz_disp_int:end,8)*r2d, ...
%         '-', 'color', c_horiz);
% end
hl(length(hl)+1) = plot(time,X_rec(:,14),'color',c_state);
ylabel('x_{sw} [~]');
% if horiz_disp
%     legend(hl,{'horizon','ref','state'});
% else
%     legend(hl,{'ref','state'});
% end

xlabel('time [s]')
linkaxes(handle_aux,'x')