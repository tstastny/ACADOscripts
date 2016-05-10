% NMPTC Simulation
% ----------------
close all; clear all; clc;

% initial conditions
N = 30;
n_U = 2;
n_X = 8;
n_XA = 0;
n_Y = 3;
n_Z = 4;

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

% wind
wn=5;
we=3;
wd=1;

% parameters
Kp_mu = 1;
Kd_mu = 10;
Kp_gam = 1;
Kd_gam = 10;
PD_gains = [Kp_mu,Kd_mu,Kp_gam,Kd_gam];

k_mu = 1;
k_mu_dot = 10;
k_gamma = 1;
k_gamma_dot = 10;

% initial conditions
ic_ned  = [0, 30, -20];
ic_V    = 14;
ic_att  = [0, 0, -pi];
ic_attdot = [0, 0];
ic_od   = [ic_V, pparams, wn, we, wd, k_mu, k_gamma, k_mu_dot, k_gamma_dot];

% acado inputs
nmpc_ic.x   = [ic_ned,ic_att,ic_attdot]; 
nmpc_ic.u   = ic_attdot;
yref        = zeros(1,n_Y);
zref        = zeros(1,n_Z);
Q_output    = [0.01 15 15];
R_controls  = [5 5 1 5];

input.x     = repmat(nmpc_ic.x, N+1,1);
input.u     = repmat(nmpc_ic.u, N,1);
input.y     = repmat([yref, zref], N,1);
input.yN    = input.y(1, 1:length(yref))';
input.od    = repmat(ic_od, N+1, 1);
input.W     = diag([Q_output, R_controls]);
input.WN    = diag(Q_output);

% simulation
T0      = 0;
Tf      = 45;
Ts      = 0.01;
time    = T0:Ts:Tf;
KKT_MPC = []; INFO_MPC = []; controls_MPC = [];

Ts_nmpc = 0.05; % interval between nmpc calls
Ts_step = 0.1; % step size in nmpc

% initial simout
X0          = nmpc_ic.x;
simout      = nmpc_ic.x(1:n_X-n_XA);
states      = simout;
d_states    = [...
    ic_V*cos(ic_att(2))*cos(ic_att(3))+wn, ...
    ic_V*cos(ic_att(2))*sin(ic_att(3))+we, ...
    -ic_V*cos(ic_att(2))+wd, ...
    zeros(1,3)];
cost = 0;

for k = 1:length(time)
    
    % measure
    input.x0    = X0';
    
    if time(k)==floor(time(k)/Ts_nmpc)*Ts_nmpc
    
    %  - - - - - START NMPC - - - - -
    st_nmpc = tic;
        
     % shift initial states and controls
    if k > 1
%         input.x     = [output.x(2:end,:); output.x(end,:)];
        input.x =spline(0:Ts_step:(N*Ts_step),output.x',Ts_nmpc:Ts_step:(N*Ts_step+Ts_nmpc))';
%         input.u     = [output.u(2:end,:); output.u(end,:)];
        input.u = spline(0:Ts_step:((N-1)*Ts_step),output.u',Ts_nmpc:Ts_step:((N-1)*Ts_step+Ts_nmpc))';
        
    end
    
    % generate controls
    output      = acado_nmpc_ext_step(input);
%     U0          = output.u(2,:);
    U0 = [spline(0:Ts_step:((N-1)*Ts_step),output.u(:,1),Ts_nmpc), ...
          spline(0:Ts_step:((N-1)*Ts_step),output.u(:,2),Ts_nmpc)];
    if U0(1) > 35*pi/180, U0(1) = 35*pi/180; end;
    if U0(1) < -35*pi/180, U0(1) = -35*pi/180; end;
    if U0(2) > 15*pi/180, U0(2) = 15*pi/180; end;
    if U0(2) < -15*pi/180, U0(2) = -15*pi/180; end;
    X0((n_X-n_XA+1):n_X) = output.x(2,(n_X-n_XA+1):n_X);
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
    Horiz_mu_rec(k,:) = output.x(:,4)';
    Horiz_gam_rec(k,:) = output.x(:,5)';
    Horiz_mu_cmd(k,:) = output.u(:,1)';
    Horiz_gam_cmd(k,:) = output.u(:,2)';
    XA_rec(k,:) = X0((n_X-n_XA+1):n_X);
%     Y_rec(k,:)	= [Va_cmd, theta_cmd, phi_cmd, intg_lon, intg_lat, etalon, etalat];
    J_rec(k,:)  = cost;
    U_rec(k,:) 	= U0;
    
    % apply control
    [d_states, simout, cost]  = uav3DoF(time(k), states, U0, wn, we, wd, ic_V, PD_gains, pparams);

    % integration (model propagation)
    states = states + d_states*Ts;
    X0(1:(n_X-n_XA)) = simout;
    
end

plotsim