% NMPTC Simulation
% ----------------
close all; clear all; clc;

% initial conditions
N = 30;
n_U = 2;
n_X = 7;
n_XA = 3;
n_Y = 10;
n_Z = 0;

% track
defpaths

pparams = [paths(1).pparam1, ...
    paths(1).pparam2, ...
    paths(1).pparam3, ...
    paths(1).pparam4, ...
    paths(1).pparam5, ...
    paths(1).pparam6, ...
    paths(1).pparam7, ...
    paths(1).pparam8, ...
    paths(1).pparam9];
pparams_next = [paths(2).pparam1, ...
    paths(2).pparam2, ...
    paths(2).pparam3, ...
    paths(2).pparam4, ...
    paths(2).pparam5, ...
    paths(2).pparam6, ...
    paths(2).pparam7, ...
    paths(2).pparam8, ...
    paths(2).pparam9];

% wind
wn=1;
we=1;
wd=-1;

% parameters
omega_n_mu = 5;
zeta_mu = 0.8;
omega_n_gamma = 5;
zeta_gamma = 0.8;

dyn = [omega_n_mu, zeta_mu, omega_n_gamma, zeta_gamma];

% initial conditions
ic_ned  = [0, 30, -20];
ic_V    = 14;
ic_att  = [0, 0, -pi/1.05];
ic_attdot = [0, 0];
ic_u    = ic_att(1:2);
ic_augm = [0,0,0];

ic_od   = [ic_V, pparams, pparams_next, wn, we, wd, ic_u];

% acado inputs
nmpc_ic.x   = [ic_ned,ic_att(3),ic_augm]; 
nmpc_ic.u   = ic_u;
yref        = zeros(1,n_Y);
zref        = zeros(1,n_Z);
% y   = [ et; e_Gamma; e_chi; intg_et; intg_e_Gamma; intg_e_chi; mu_r; gamma_r; Delta_mu_r; Delta_gamma_r ];
Q_output    = [0.05 10 10 0.05 0 0 100 150];
QN_output   = [0.05 10 10 0.05 0 0 100 150];
R_controls  = [];
Q_prev      = [500*(linspace(0,1,N+1)'-ones(N+1,1)).^2,500*(linspace(0,1,N+1)'-ones(N+1,1)).^2];

input.x     = repmat(nmpc_ic.x, N+1,1);
input.u     = repmat(nmpc_ic.u, N,1);
input.y     = repmat([yref, zref], N,1);
input.yN    = input.y(1, 1:length(yref))';
input.od    = repmat(ic_od, N+1, 1);
QR = [Q_output, R_controls];
for i = 1:N
input.W(n_Y*(i-1)+1:n_Y*i,:) = diag([QR, Q_prev(i,:)]);
end
input.WN    = diag([QN_output, Q_prev(N+1,:)]);

% simulation
T0      = 0;
Tf      = 50;
Ts      = 0.01;
time    = T0:Ts:Tf;
KKT_MPC = []; INFO_MPC = []; controls_MPC = [];

Ts_nmpc = 0.1; % interval between nmpc calls
Ts_step = 0.1; % step size in nmpc

% initial simout
X0          = nmpc_ic.x;
simout      = [nmpc_ic.x(1:(n_X-n_XA-1)) ic_att ic_attdot];
states      = simout;
d_states    = [...
    ic_V*cos(ic_u(2))*cos(ic_u(1))+wn, ...
    ic_V*cos(ic_u(2))*sin(ic_u(1))+we, ...
    -ic_V*cos(ic_u(2))+wd, ...
    zeros(1,5)];
cost = 0;
U0 = zeros(1,2);
path_idx = 1;
for k = 1:length(time)
    
    % check path
    if pparams(1) < 0.5
        path_checks(k) = check_line_seg(states(1:3),[ic_V, pparams, pparams_next, wn, we, wd]);
    elseif pparams(1) < 1.5
        path_checks(k) = check_curve_seg(states(1:3),pparams);
    end
    if path_checks(k) && path_idx<length(paths)
        for i = 1:length(pparams)
            pparams(i) = pparams_next(i);
            eval(['pparams_next(i) = paths(path_idx+1).pparam',int2str(i),';']);
        end
        path_idx = path_idx + 1;
        ic_od   = [ic_V, pparams, pparams_next, wn, we, wd, ic_u];
        input.od    = repmat(ic_od, N+1, 1);
    end

%     path_checks(k) = check_curve_seg(states(1:3),pparams(2:end));

    
    % measure
    input.x0    = X0';
    
    if time(k)==floor(time(k)/Ts_nmpc)*Ts_nmpc
    
    %  - - - - - START NMPC - - - - -
    st_nmpc = tic;
        
     % shift initial states and controls
    if k > 1
        input.x     = output.x;%[output.x(2:end,:); output.x(end,:)]; %
%         input.x =spline(0:Ts_step:(N*Ts_step),output.x',Ts_nmpc:Ts_step:(N*Ts_step+Ts_nmpc))';
        input.u     = output.u;%[output.u(2:end,:); output.u(end,:)]; %
%         input.u = spline(0:Ts_step:((N-1)*Ts_step),output.u',Ts_nmpc:Ts_step:((N-1)*Ts_step+Ts_nmpc))';
        input.od(:,end-1:end) = [input.u(2:end,:); input.u(end,:); input.u(end,:)];
    end
    
    % generate controls
    output      = acado_nmpc_ext_step(input);
    
%     if output.u(
    
    U0_last     = U0;
    U0          = output.u(2,:);
%     U0 = [spline(0:Ts_step:((N-1)*Ts_step),output.u(:,1),Ts_nmpc), ...
%           spline(0:Ts_step:((N-1)*Ts_step),output.u(:,2),Ts_nmpc)];
    if U0(1) > 35*pi/180, U0(1) = 35*pi/180; end;
    if U0(1) < -35*pi/180, U0(1) = -35*pi/180; end;
    if U0(2) > 15*pi/180, U0(2) = 15*pi/180; end;
    if U0(2) < -15*pi/180, U0(2) = -15*pi/180; end;
%     if U0(1) - U0_last(1) > 5*pi/180, U0(1) = U0_last(1) + 5*pi/180; end;
%     if U0(1) - U0_last(1) > 5*pi/180, U0(1) = U0_last(1) - 5*pi/180; end;
%     if U0(2) - U0_last(2) > 10*pi/180, U0(1) = U0_last(1) + 10*pi/180; end;
%     if U0(2) - U0_last(2) > 10*pi/180, U0(1) = U0_last(1) - 10*pi/180; end;
    X0((n_X-n_XA+1):n_X) = output.x(1,(n_X-n_XA+1):n_X);
%     if ~isempty((n_X-n_XA+1):n_X)
%         X0((n_X-n_XA+1):n_X) = spline(0:Ts_step:(N*Ts_step),output.x(:,(n_X-n_XA+1):n_X)',Ts_nmpc);
%     end
    
    % record info
    INFO_MPC    = [INFO_MPC; output.info];
    KKT_MPC     = [KKT_MPC; output.info.kktValue];
    
%     % shift initial states and controls
%     input.x     = [output.x(2:N+1,:); output.x(N+1,:)];
%     input.u     = [output.u(2:N,:); output.u(N,:)];
%     X0(10:15)   = output.x(2,10:15);

    tsolve(k) = toc(st_nmpc);
    
    end
    % - - - - - END NMPC - - - - -
    
    % record states/controls
    X_rec(k,:)  = simout;
    Horiz_n_rec(k,:) = output.x(:,1)';
    Horiz_e_rec(k,:) = output.x(:,2)';
    Horiz_d_rec(k,:) = output.x(:,3)';
    Horiz_mu_r_rec(k,:) = output.u(:,1)';
    Horiz_gamma_r_rec(k,:) = output.u(:,2)';
%     Horiz_mu_dot_rec(k,:) = output.u(:,1)';
%     Horiz_gamma_dot_rec(k,:) = output.u(:,2)';
    XA_rec(k,:) = X0((n_X-n_XA+1):n_X);
%     Y_rec(k,:)	= [Va_cmd, theta_cmd, phi_cmd, intg_lon, intg_lat, etalon, etalat];
    J_rec(k,:)  = cost;
    U_rec(k,:) 	= U0;
    
    % apply control
    [d_states, simout, cost]  = uav3DoF(time(k), states, U0, wn, we, wd, ic_V, dyn, pparams);

    % integration (model propagation)
    states = states + d_states*Ts;
    X0(1:(n_X-n_XA)) = simout([1:(n_X-n_XA-1),6]);
    
end

plotsim