
horiz_disp = true;
horiz_disp_int = 50; % every Nth record

r2d = 180/pi;

% /////////////////////////////////////////////////////////////////////////
% POSITION

figure('color','w','name','Position')
hold on; grid on; axis equal;
idx_pos = 1;

sss = linspace(0,2*pi,1000);
h_pos(idx_pos) = plot(c_e + R * sin(sss), c_n + R * cos(sss), '-', 'color', [1 0.6 0.6]);
leg_pos{idx_pos} = 'Path';
idx_pos = idx_pos + 1;
if horiz_disp
    h_pos(idx_pos) = plot(Horiz_e_rec(1:horiz_disp_int:end,1)', ...
        Horiz_n_rec(1:horiz_disp_int:end,1)', ...
        '-', 'color', [0.6 0.6 0.6]);
    plot(Horiz_e_rec(1:horiz_disp_int:end,2:end)', ...
        Horiz_n_rec(1:horiz_disp_int:end,2:end)', ...
        '-', 'color', [0.6 0.6 0.6]);
    leg_pos{idx_pos} = 'Horizon';
    idx_pos = idx_pos + 1;
end
h_pos(idx_pos) = plot(X_rec(:,2), X_rec(:,1), '-', 'color', [0 0 0]);
leg_pos{idx_pos} = 'Flight Path';
idx_pos = idx_pos + 1;
legend(h_pos, leg_pos);
xlabel('East [m]')
ylabel('North [m]')
zlabel('Height [m]');
xlabel('time [s]')

% /////////////////////////////////////////////////////////////////////////
% ATTITUDE

figure('color','w','name','Attitude')
idx_att = 1;

h_atts(1) = subplot(3,1,1); hold on; grid on;
if horiz_disp
    h_att(idx_att) = plot(time(1)+Ts_step*(0:N-1), ...
        Horiz_mu_r_rec(1,:)'*r2d, ...
        '-', 'color', [0.7 0.7 0.7]);
    plot(repmat(time(1:horiz_disp_int:end),N,1)+Ts_step*repmat((0:N-1)',1,length(time(1:horiz_disp_int:end))), ...
        Horiz_mu_r_rec(1:horiz_disp_int:end,:)'*r2d, ...
        '-', 'color', [0.7 0.7 0.7]);
    leg_att{idx_att} = '\mu_r Horizon';
    idx_att = idx_att + 1;
    h_att(idx_att) = plot(time(1)+Ts_step*(0:N), ...
        Horiz_mu_rec(1,:)'*r2d, ...
        '-', 'color', [1 0.7 0.7]);
    plot(repmat(time(1:horiz_disp_int:end),N+1,1)+Ts_step*repmat((0:N)',1,length(time(1:horiz_disp_int:end))), ...
        Horiz_mu_rec(1:horiz_disp_int:end,:)'*r2d, ...
        '-', 'color', [1 0.7 0.7]);
    leg_att{idx_att} = '\mu Horizon';
    idx_att = idx_att + 1;
end
h_att(idx_att) = plot(time, U_rec(:,1)*r2d, '-', 'color', [0.6 0.6 0.6]);
leg_att{idx_att} = '\mu_r';
idx_att = idx_att + 1;
h_att(idx_att) = plot(time, X_rec(:,3)*r2d, '-k');
leg_att{idx_att} = '\mu';
idx_att = idx_att + 1;
ylabel('\mu [deg]');
legend(h_att, leg_att);
ylim([-40 40]);
h_atts(2) = subplot(2,1,2); hold on; grid on;
plot(time,X_rec(:,4)*r2d);
ylabel('\xi [deg]')
xlabel('time [s]')
linkaxes(h_atts,'x')

% /////////////////////////////////////////////////////////////////////////
% RATES

figure('color','w','name','Angular Rates')
h_attdot(1) = subplot(1,1,1); hold on; grid on;
plot(time,X_rec(:,5)*r2d);
ylabel('$\dot{\mu} [deg/s]$','interpreter','latex')
xlabel('time [s]')
linkaxes(h_attdot,'x')

% /////////////////////////////////////////////////////////////////////////
% AUXILLARY

figure('color','w','name','Auxillary')
stairs(time,tsolve*10^3)
ylabel('t_{solve} [ms]')
xlabel('time [s]')
