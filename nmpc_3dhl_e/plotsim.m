
horiz_disp = false;
horiz_disp_int = 100; % every Nth record

horiz_time = (repmat(time(1:horiz_disp_int:end),N+1,1)+Ts_step*repmat((0:N)',1,length(time(1:horiz_disp_int:end))));

r2d = 180/pi;

c_horiz = [0.5 0.5 0.8];
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

hl(length(hl)+1) = plot3(p_n,p_e,-p_d,'o','color',c_ref);

% guidance
guide_disp_int=100;
if true
    plot3([aux_rec(1:guide_disp_int:end,2) X_rec(1:guide_disp_int:end,2)]',...
        [aux_rec(1:guide_disp_int:end,1) X_rec(1:guide_disp_int:end,1)]',...
        [-aux_rec(1:guide_disp_int:end,3) -X_rec(1:guide_disp_int:end,3)]','r')
    plot3(aux_rec(1:guide_disp_int:end,2),aux_rec(1:guide_disp_int:end,1),-aux_rec(1:guide_disp_int:end,3),'ro')
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
hl(length(hl)+1) = plot3(X_rec(:,2),X_rec(:,1),-X_rec(:,3),'linewidth',1.5);
xlabel('East [m]')
ylabel('North [m]')
zlabel('Height [m]');
if horiz_disp
    legend(hl,{'point','horizon','position'});
else
    legend(hl,{'point','position'});
end
% view(10,60)

%% /////////////////////////////////////////////////////////////////////////
% OBJECTIVE COSTS

figure('color','w','name','Objective Costs')
hc(1)=subplot(3,1,1); hold on; grid on;

plot(time,Y_rec(:,1));
plot(time,Y_rec(:,2));

legend('e_{lat}','e_{lon}')
ylabel('e [m]')

hc(2)=subplot(3,1,2:3); hold on; grid on;

plot(time,(yref(1)*ones(length(time),1)-Y_rec(:,1)).^2*Q_output(1)); % e_lat
plot(time,(yref(2)*ones(length(time),1)-Y_rec(:,2)).^2*Q_output(2)); % e_lon
plot(time,(yref(3)*ones(length(time),1)-Y_rec(:,3)).^2*Q_output(3)); % V
plot(time,(yref(4)*ones(length(time),1)-Y_rec(:,4)).^2*Q_output(4)); % p
plot(time,(yref(5)*ones(length(time),1)-Y_rec(:,5)).^2*Q_output(5)); % q
plot(time,(yref(6)*ones(length(time),1)-Y_rec(:,6)).^2*Q_output(6)); % r
plot(time,(yref(7)*ones(length(time),1)-Y_rec(:,7)).^2*Q_output(7)); % alpha_soft

legend('e_{lat}','e_{lon}','V','p','q','r','\alpha_{soft}')
ylabel('J(x,u)')

xlabel('time [s]')

linkaxes(hc,'x');

%% /////////////////////////////////////////////////////////////////////////
% AIRSPEED + THROTTLE

figure('color','w','name','Airspeed & Throttle')

handle_vdt(1) = subplot(3,1,1); hold on; grid on;
hl = gobjects(0);
hl(length(hl)+1) = plot(time,yref(3)*ones(1,length(time)),'color',c_ref);
hl(length(hl)+1) = plot(time,X_rec(:,4));
ylabel('V [m/s]');
legend(hl,{'ref','state'});

handle_vdt(2) = subplot(3,1,2); hold on; grid on;
hl = gobjects(0);
hl(length(hl)+1) = plot(time,alpha_p_co*ones(1,length(time))*r2d,'color',c_ref);
plot(time,alpha_m_co*ones(1,length(time))*r2d,'color',c_ref);
hl(length(hl)+1) = plot(time,(X_rec(:,8)-X_rec(:,5))*r2d);
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
hl(length(hl)+1) = plot(time,X_rec(:,12));
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

handle_att(1) = subplot(2,1,1); hold on; grid on;
hl = gobjects(0);
if horiz_disp
    hl(length(hl)+1) = plot(horiz_time(1:end-1,1), ...
        horiz_rec_U(:,1,2)*r2d, ...
        '-', 'color', c_horiz);
    plot(horiz_time(1:end-1,2:end), ...
        horiz_rec_U(:,horiz_disp_int:horiz_disp_int:end,2)*r2d, ...
        '-', 'color', c_horiz);
end
hl(length(hl)+1) = plot(time,U_rec(:,2)*r2d,'color',c_ref);
hl(length(hl)+1) = plot(time,X_rec(:,7)*r2d);
ylabel('\phi [deg]');
if horiz_disp
    legend(hl,{'horizon','ref','state'});
else
    legend(hl,{'ref','state'});
end

handle_att(2) = subplot(2,1,2); hold on; grid on;
hl = gobjects(0);
if horiz_disp
    hl(length(hl)+1) = plot(horiz_time(1:end-1,1), ...
        horiz_rec_U(:,1,3)*r2d, ...
        '-', 'color', c_horiz);
    plot(horiz_time(1:end-1,2:end), ...
        horiz_rec_U(:,horiz_disp_int:horiz_disp_int:end,3)*r2d, ...
        '-', 'color', c_horiz);
end
hl(length(hl)+1) = plot(time,U_rec(:,3)*r2d,'color',c_ref);
hl(length(hl)+1) = plot(time,X_rec(:,8)*r2d);
ylabel('\theta [deg]');
if horiz_disp
    legend(hl,{'horizon','ref','state'});
else
    legend(hl,{'ref','state'});
end

xlabel('time [s]')
linkaxes(handle_att,'x')

%% /////////////////////////////////////////////////////////////////////////
% RATES

figure('color','w','name','Rates')

handle_rates(1) = subplot(3,1,1); hold on; grid on;
plot(time,X_rec(:,9)*r2d);
ylabel('p [deg/s]');

handle_rates(2) = subplot(3,1,2); hold on; grid on;
plot(time,X_rec(:,10)*r2d);
ylabel('q [deg/s]');

handle_rates(3) = subplot(3,1,3); hold on; grid on;
plot(time,X_rec(:,11)*r2d);
ylabel('r [deg/s]');

xlabel('time [s]')
linkaxes(handle_rates,'x')

%% /////////////////////////////////////////////////////////////////////////
% AUXILIARY

figure('color','w','name','Auxiliary')

hold on; grid on;
plot(time,tsolve*1000,'.');
ylabel('t_{solve} [ms]');
xlabel('time [s]')
