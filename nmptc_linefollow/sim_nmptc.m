% NMPTC Simulation
% ----------------
close all; clear all; clc;

% initial conditions
N = 20;
n_U = 2;
n_X = 6;
n_XA = 0;
n_Y = 1;
n_Z = 4;

% track
aa = [0 0 0];
bb = [2000 0 0];

% wind
wn=3;
we=5;
wd=2;

% parameters
tau_mu = 0.1;
tau_gamma = 0.1;

% initial conditions
ic_ned  = [0, 30, -20];
ic_V    = 14;
ic_att  = [0, 0, 0];
ic_od   = [ic_V, wn, we, wd, aa(1), aa(2), aa(3), bb(1), bb(2), bb(3)];

% acado inputs
nmpc_ic.x   = [ic_ned,ic_att]; 
nmpc_ic.u   = ic_att(1:2);
yref        = zeros(1,n_Y);
zref        = zeros(1,n_Z);
Q_output    = [1/50];
R_controls  = [5/(15*pi/180) 15/(45*pi/180) 0.5/(100*pi/180) 5/(100*pi/180)];

input.x     = repmat(nmpc_ic.x, N+1,1);
input.u     = repmat(nmpc_ic.u, N,1);
input.y     = repmat([yref, zref], N,1);
input.yN    = input.y(1, 1:length(yref))';
input.od    = repmat(ic_od, N+1, 1);
input.W     = diag([Q_output, R_controls]);
input.WN    = diag(Q_output);

% simulation
T0      = 0;
Tf      = 30;
Ts      = 0.01;
time    = T0:Ts:Tf;
KKT_MPC = []; INFO_MPC = []; controls_MPC = [];

Ts_nmpc = 0.05; 
Ts_step = 0.2;

% initial simout
X0          = nmpc_ic.x;
simout      = nmpc_ic.x;
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
        input.x     = spline(0:Ts_step:(N*Ts_step),output.x',Ts_nmpc:Ts_step:(N*Ts_step+Ts_nmpc))';
        input.u     = spline(0:Ts_step:((N-1)*Ts_step),output.u',Ts_nmpc:Ts_step:((N-1)*Ts_step+Ts_nmpc))';
        X0((n_X-n_XA+1):n_X)   = output.x(2,(n_X-n_XA+1):n_X);
    end
    
    % generate controls
    output      = acado_nmpc_step(input);
%     U0          = output.x(2,4:5);
    U0 = [spline(0:Ts_step:(N*Ts_step),output.x(:,4),Ts_nmpc), ...
          spline(0:Ts_step:(N*Ts_step),output.x(:,5),Ts_nmpc)];
    
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
%     XA_rec(k,:) = X0((n_X-n_XA+1):n_X);
%     Y_rec(k,:)	= [Va_cmd, theta_cmd, phi_cmd, intg_lon, intg_lat, etalon, etalat];
    J_rec(k,:)  = cost;
    U_rec(k,:) 	= U0;
    
    % apply control
    [d_states, simout, cost]  = uav3DoF(time(k), states, U0, wn, we, wd, ic_V, tau_mu, tau_gamma, aa, bb);

    % integration (model propagation)
    states = states + d_states*Ts;
    X0(1:(n_X-n_XA)) = simout(1:(n_X-n_XA));
    
end

plotsim