
horiz_disp = true;
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
for k = 1:length(paths)

    if paths(k).pparam1 == 0
        hl(1) = plot3([paths(k).pparam3 paths(k).pparam3+100*cos(paths(k).pparam6+pi)*cos(paths(k).pparam7)], ...
            [paths(k).pparam2 paths(k).pparam2+100*sin(paths(k).pparam6+pi)*cos(paths(k).pparam7)], ...
            -[paths(k).pparam4 paths(k).pparam4+100*sin(paths(k).pparam7)], ...
            'color',c_ref,'linewidth',1);
    elseif paths(k).pparam1 == 1
        tset = linspace(paths(k).pparam6 - sign(paths(k).pparam5) * pi/2, ...
            (paths(k).pparam6 - sign(paths(k).pparam5) * pi/2) - sign(paths(k).pparam5)*2*pi*3, ltset)';
        
        r = repmat([paths(k).pparam2,paths(k).pparam3,paths(k).pparam4],ltset,1) + ...
            [abs(paths(k).pparam5) * cos(tset), abs(paths(k).pparam5) * sin(tset), ...
            abs(tset-tset(1)) * abs(paths(k).pparam5) * tan(paths(k).pparam7)];
        
        hl(1) = plot3(r(:,2),r(:,1),-r(:,3),'color',c_ref,'linewidth',1);
    elseif paths(k).pparam1 == 2
        tset = linspace(0,2*pi,ltset)';
        r = repmat([paths(k).pparam2,paths(k).pparam3,paths(k).pparam4],ltset,1) + ...
            [abs(paths(k).pparam5) * cos(tset), abs(paths(k).pparam5) * sin(tset), ...
            -paths(k).pparam3 * ones(ltset,1)];
        hl(1) = plot3(r(:,2),r(:,1),-r(:,3),'color',c_ref,'linewidth',1);
    end
    
end
% guidance
guide_disp_int=100;
if true
    plot3([aux_rec(1:guide_disp_int:end,4) X_rec(1:guide_disp_int:end,2)]',...
        [aux_rec(1:guide_disp_int:end,3) X_rec(1:guide_disp_int:end,1)]',...
        [-aux_rec(1:guide_disp_int:end,5) -X_rec(1:guide_disp_int:end,3)]','r')
    plot3(aux_rec(1:guide_disp_int:end,4),aux_rec(1:guide_disp_int:end,3),-aux_rec(1:guide_disp_int:end,5),'ro')
    quiver3(aux_rec(1:guide_disp_int:end,4),aux_rec(1:guide_disp_int:end,3),-aux_rec(1:guide_disp_int:end,5),...
        aux_rec(1:guide_disp_int:end,7),aux_rec(1:guide_disp_int:end,6),-aux_rec(1:guide_disp_int:end,8),'c')
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

% speed vectors
spd_disp_int = 100;
hl(length(hl)+1) = quiver3(X_rec(1:spd_disp_int:end,2),X_rec(1:spd_disp_int:end,1),-X_rec(1:spd_disp_int:end,3),...
    aux_rec(1:spd_disp_int:end,13),aux_rec(1:spd_disp_int:end,12),-aux_rec(1:spd_disp_int:end,14),...
    'AutoScaleFactor',0.3);
hl(length(hl)+1) = quiver3(X_rec(1:spd_disp_int:end,2),X_rec(1:spd_disp_int:end,1),-X_rec(1:spd_disp_int:end,3),...
    X_rec(1:spd_disp_int:end,4).*sin(X_rec(1:spd_disp_int:end,6)).*cos(X_rec(1:spd_disp_int:end,5)),...
    X_rec(1:spd_disp_int:end,4).*cos(X_rec(1:spd_disp_int:end,6)).*cos(X_rec(1:spd_disp_int:end,5)),...
    X_rec(1:spd_disp_int:end,4).*sin(X_rec(1:spd_disp_int:end,5)),...
    'AutoScaleFactor',0.3);
hl(length(hl)+1) = quiver3(X_rec(1:spd_disp_int:end,2),X_rec(1:spd_disp_int:end,1),-X_rec(1:spd_disp_int:end,3),...
    aux_rec(1:spd_disp_int:end,16),aux_rec(1:spd_disp_int:end,15),-aux_rec(1:spd_disp_int:end,17),...
    'AutoScaleFactor',0.3);


if horiz_disp
    legend(hl,{'path','horizon','position','vG','vA','w'});
else
    legend(hl,{'path','position','vG','vA','w'});
end
% view(10,60)

%% /////////////////////////////////////////////////////////////////////////
% OBJECTIVE COSTS

figure('color','w','name','Objective Costs')
hc(1)=subplot(4,1,1); hold on; grid on;

plot(time,aux_rec(:,1));
plot(time,aux_rec(:,2));

legend('e_{ne}','e_d')
ylabel('e [m]')

hc(2)=subplot(4,1,2); hold on; grid on;

plot(time,rad2deg(Y_rec(:,1))); % eta_lat
plot(time,rad2deg(Y_rec(:,2))); % eta_lon

legend('\eta_{lat}','\eta_{lon}')
ylabel('\eta [deg]')

hc(3)=subplot(4,1,3:4); hold on; grid on;

plot(time,(yref(1)*ones(length(time),1)-Y_rec(:,1)).^2*Q_output(1)); % eta_lat
plot(time,(yref(2)*ones(length(time),1)-Y_rec(:,2)).^2*Q_output(2)); % eta_lon
plot(time,(yref(3)*ones(length(time),1)-Y_rec(:,3)).^2*Q_output(3)); % V
plot(time,(yref(4)*ones(length(time),1)-Y_rec(:,4)).^2*Q_output(4)); % p
plot(time,(yref(5)*ones(length(time),1)-Y_rec(:,5)).^2*Q_output(5)); % q
plot(time,(yref(6)*ones(length(time),1)-Y_rec(:,6)).^2*Q_output(6)); % r
plot(time,(yref(7)*ones(length(time),1)-Y_rec(:,7)).^2*Q_output(7)); % alpha_soft

plot(time,(zref(2)*ones(length(time),1)-Y_rec(:,9)).^2*R_controls(2)); % uT
plot(time,(zref(3)*ones(length(time),1)-Y_rec(:,10)).^2*R_controls(3)); % phi_ref
plot(time,(zref(4)*ones(length(time),1)-Y_rec(:,11)).^2*R_controls(4)); % theta_ref

legend('\eta_{lat}','\eta_{lon}','V','p','q','r','\alpha_{soft}','u_T','\phi_{ref}','\theta_{ref}')
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
% SPEEDS

figure('color','w','name','Speeds')
hold on; grid on;

plot(time,yref(3)*ones(1,length(time)),'color',c_ref);
plot(time,X_rec(:,4));
plot(time,sqrt(aux_rec(:,12).^2 + aux_rec(:,13).^2));
plot(time,aux_rec(:,14));
plot(time,sqrt(aux_rec(:,15).^2 + aux_rec(:,16).^2 + aux_rec(:,17).^2));

legend('v_{A}^{ref}','v_A','v_{ne}','v_{d}','|w|')

xlabel('time [s]')

%% /////////////////////////////////////////////////////////////////////////
% ATTITUDE

figure('color','w','name','Attitude')

handle_rates(1) = subplot(2,1,1); hold on; grid on;
plot(time,rad2deg(U_rec(:,2)));
plot(time,rad2deg(aux_rec(:,18)));
plot(time,rad2deg(X_rec(:,7)));
ylabel('\phi [deg]');
legend('\phi_{ref}','phi_{ff}','\phi')

handle_rates(2) = subplot(2,1,2); hold on; grid on;
plot(time,rad2deg(U_rec(:,3)));
plot(time,rad2deg(X_rec(:,8)));
ylabel('\theta [deg/s]');
legend('\theta_{ref}','\theta')

xlabel('time [s]')
linkaxes(handle_rates,'x')

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

handle_aux(1) = subplot(2,1,1); hold on; grid on;
plot(time,tsolve*1000,'.');
ylabel('t_{solve} [ms]');

handle_aux(2) = subplot(2,1,2); hold on; grid on;
plot(time,X_rec(:,13),'o');
ylabel('x_{sw} [~]');

xlabel('time [s]')
linkaxes(handle_aux,'x')
