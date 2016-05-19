
horiz_disp = true;
horiz_disp_int = 20; % every Nth record

r2d = 180/pi;

figure('color','w','name','Position')
subplot(5,1,1:4); hold on; grid on; axis equal;
ltset=1000;
for k = 1:length(paths)

    if paths(k).pparam1 == 0
        plot3([paths(k).pparam3 paths(k).pparam6], ...
            [paths(k).pparam2 paths(k).pparam5], ...
            -[paths(k).pparam4 paths(k).pparam7],'--m','linewidth',2);
    elseif paths(k).pparam1 == 1
        tset = linspace(paths(k).pparam8, ...
            paths(k).pparam8 + paths(k).pparam9 * paths(k).pparam6, ...
            ltset)';
        r = repmat([paths(k).pparam2,paths(k).pparam3,paths(k).pparam4],ltset,1) + ...
            [paths(k).pparam5 * cos(tset), paths(k).pparam5 * sin(tset), ...
            -paths(k).pparam6 * (tset-tset(1)) * paths(k).pparam5 * tan(paths(k).pparam7)];
        plot3(r(:,2),r(:,1),-r(:,3),'--m','linewidth',2)
    end
    
end
if horiz_disp
    plot3(Horiz_e_rec(1:horiz_disp_int:end,:)', ...
        Horiz_n_rec(1:horiz_disp_int:end,:)', ...
        -Horiz_d_rec(1:horiz_disp_int:end,:)', ...
        '-', 'color', [0.7 1 1]);
end
plot3(X_rec(:,2),X_rec(:,1),-X_rec(:,3))
xlabel('East [m]')
ylabel('North [m]')
zlabel('Height [m]');
subplot(5,1,5); hold on; grid on;
plot(time,J_rec(:,1));
ylabel('e_T')
xlabel('time [s]')

% /////////////////////////////////////////////////////////////////////////
% ATTITUDE

figure('color','w','name','Attitude')
handle_att(1) = subplot(3,1,1); hold on; grid on;
if horiz_disp
    plot(repmat(time(1:horiz_disp_int:end),N,1)+Ts_step*repmat((0:N-1)',1,length(time(1:horiz_disp_int:end))), ...
        Horiz_mu_r_rec(1:horiz_disp_int:end,:)'*r2d, ...
        '-', 'color', [1 0.7 0.7]);
end
plot(time,U_rec(:,1)*r2d,'-k');
plot(time,X_rec(:,4)*r2d);
ylabel('\mu [deg]');
legend('horizon','reference','state');
ylim([-40 40]);
handle_att(2) = subplot(3,1,2); hold on; grid on;
if horiz_disp
    plot(repmat(time(1:horiz_disp_int:end),N,1)+Ts_step*repmat((0:N-1)',1,length(time(1:horiz_disp_int:end))), ...
        Horiz_gamma_r_rec(1:horiz_disp_int:end,:)'*r2d, ...
        '-', 'color', [1 0.7 0.7]);
end
plot(time,U_rec(:,2)*r2d,'-k');
plot(time,X_rec(:,5)*r2d);
ylabel('\gamma [deg]')
legend('horizon','reference','state');
ylim([-20 20]);
handle_att(3) = subplot(3,1,3); hold on; grid on;
plot(time,X_rec(:,6)*r2d);
ylabel('\xi [deg]')
xlabel('time [s]')
linkaxes(handle_att,'x')

% /////////////////////////////////////////////////////////////////////////
% RATES

figure('color','w','name','Angular Rates')
handle_attdot(1) = subplot(2,1,1); hold on; grid on;
% if horiz_disp
%     plot(repmat(time(1:horiz_disp_int:end),N,1)+Ts_step*repmat((0:N-1)',1,length(time(1:horiz_disp_int:end))), ...
%         Horiz_mu_dot_rec(1:horiz_disp_int:end,:)'*r2d, ...
%         '-', 'color', [1 0.7 0.7]);
% end
% plot(time,Horiz_mu_dot_rec(:,1)*r2d,'-k');
plot(time,X_rec(:,7)*r2d);
ylabel('$\dot{\mu} [deg/s]$','interpreter','latex')
% legend('horizon','reference','state');
% ylim([-250 250]);
handle_attdot(2) = subplot(2,1,2); hold on; grid on;
% if horiz_disp
%     plot(repmat(time(1:horiz_disp_int:end),N,1)+Ts_step*repmat((0:N-1)',1,length(time(1:horiz_disp_int:end))), ...
%         Horiz_gamma_dot_rec(1:horiz_disp_int:end,:)'*r2d, ...
%         '-', 'color', [1 0.7 0.7]);
% end
% plot(time,Horiz_gamma_dot_rec(:,1)*r2d,'-k');
plot(time,X_rec(:,8)*r2d);
ylabel('$\dot{\gamma} [deg/s]$','interpreter','latex')
% legend('horizon','reference','state');
% ylim([-150 150]);
xlabel('time [s]')
linkaxes(handle_attdot,'x')

% /////////////////////////////////////////////////////////////////////////
% AUXILLARY

figure('color','w','name','Auxillary')
stairs(time,tsolve*10^3)
ylabel('t_{solve} [ms]')
xlabel('time [s]')
