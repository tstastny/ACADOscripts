% NMPTC Simulation
% ----------------
close all; clear all; clc;

% initial conditions
N = 30;
n_U = 1;
n_X = 6;
n_XA = 3;
n_Y = 6;
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
wn=2;
we=4;

% parameters
omega_n_mu = 5;
zeta_mu = 0.8;

dyn = [omega_n_mu, zeta_mu];

% initial conditions
ic_ne  = [0, 30];
ic_V    = 14;
ic_att  = [0, -pi/1.05];
ic_attdot = [0];
ic_u    = ic_att(1);
ic_augm = [0,0,0];

ic_od   = [ic_V, pparams, pparams_next, wn, we, ic_u];

% acado inputs
nmpc_ic.x   = [ic_ne,ic_att(2),ic_augm]; 
nmpc_ic.u   = ic_u;
yref        = zeros(1,n_Y);
zref        = zeros(1,n_Z);
% y   = [ et; e_chi; intg_et; intg_e_chi; mu_r; Delta_mu_r ];
Q_output    = [0.1 10 0 0 50];
QN_output   = [0.1 10 0 0 50];
R_controls  = [];
Q_prev      = [100*(linspace(0,1,N+1)'-ones(N+1,1)).^2];

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
    ic_V*cos(ic_u(1))+wn, ...
    ic_V*sin(ic_u(1))+we, ...
    zeros(1,5)];
cost = 0;
U0 = zeros(1,2);
path_idx = 1;
endofwaypoints=false;
for k = 1:length(time)
    
    % check path
    if pparams(1) < 0.5
        path_checks(k) = check_line_seg(states(1:2),pparams(2:end));
    elseif pparams(1) < 1.5
        if time(k) > 37
            stoppp=1;
        end
        path_checks(k) = check_curve_seg(states(1:2),pparams(2:end),d_states(1),d_states(2));
    end
    if path_checks(k)
        if path_idx<length(paths)-1
            for i = 1:length(pparams)
                pparams(i) = pparams_next(i);
                eval(['pparams_next(i) = paths(path_idx+2).pparam',int2str(i),';']);
            end
            path_idx = path_idx + 1;
            ic_od   = [ic_V, pparams, pparams_next, wn, we, ic_u];
            input.od    = repmat(ic_od, N+1, 1);
            output.x(:,end) = zeros(N+1,1);
            X0(6) = 0;
        elseif ~endofwaypoints
            endofwaypoints=true;
            for i = 1:length(pparams)
                pparams(i) = pparams_next(i);
%                 eval(['pparams_next(i) = paths(path_idx+1).pparam',int2str(i),';']);
            end
            path_idx = path_idx + 1;
            ic_od   = [ic_V, pparams, pparams_next, wn, we, ic_u];
            input.od    = repmat(ic_od, N+1, 1);
            output.x(:,end) = zeros(N+1,1);
            X0(6) = 0;
        end
    end

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
        input.od(:,end) = [input.u(2:end,:); input.u(end,:); input.u(end,:)];
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
%     if U0(1) - U0_last(1) > 5*pi/180, U0(1) = U0_last(1) + 5*pi/180; end;
%     if U0(1) - U0_last(1) > 5*pi/180, U0(1) = U0_last(1) - 5*pi/180; end;
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
    Horiz_sw_rec(k,:) = output.x(:,6)';
    Horiz_mu_r_rec(k,:) = output.u(:,1)';
    XA_rec(k,:) = X0((n_X-n_XA+1):n_X);
    J_rec(k,:)  = cost;
    U_rec(k,:) 	= U0;
    
    % apply control
    [d_states, simout, cost]  = uav3DoF(time(k), states, U0, wn, we, ic_V, dyn, pparams);

    % integration (model propagation)
    states = states + d_states*Ts;
    X0(1:(n_X-n_XA)) = simout([1:(n_X-n_XA-1),4]);
    
end

plotsim