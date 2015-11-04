close all; clear all; clc;

% initial conditions
N       = 10;
n_U     = 1;
n_Y     = 1;
n_X     = 1;

X0      = 50;
U0      = 0;

input.x     = repmat(X0,N+1,n_X);
input.u     = repmat(U0,N,n_U);
yref        = 0;
input.y     = [repmat(yref,N,1), zeros(N,n_U)];
input.yN    = input.y(1,n_Y)';

Q_output    = 1;
R_controls  = 1;

input.W     = diag([Q_output, R_controls]);
input.WN    = diag(Q_output);

% simulation
T0      = 0;
Tf      = 100;
Ts      = 0.1;
time    = T0:Ts:Tf;
KKT_MPC = []; INFO_MPC = []; controls_MPC = [];

for k = 1:length(time)
    
    %  - - - - - START NMPC - - - - -
    
    st_nmpc = tic;
    
    % measure
    input.x0    = X0';
    
    % reference
    yref        = 0; 
    input.y     = [repmat(yref,N,1), zeros(N,n_U)];
    input.yN    = input.y(end,n_Y)';
    
    % generate controls
    output      = acado_nmpc_step(input);
    U0          = output.u(1,:);
    
    % record info
    INFO_MPC    = [INFO_MPC; output.info];
    KKT_MPC     = [KKT_MPC; output.info.kktValue];
    
    % record states/controls
    X_rec(k,:)       = X0;
    U_rec(k,:)       = U0;
    
    % shift initial states and controls
    input.x     = [output.x(2:N+1,:); output.x(N+1,:)];
    input.u     = [output.u(2:N,:); output.u(N,:)];

    tsolve(k) = toc(st_nmpc);
    
    % - - - - - END NMPC - - - - -
    
    % apply control
    xdot_k = U0;
    X0 = xdot_k * Ts + X0;

end

figure('color','w');
subplot(2,1,1); hold on; grid on;
plot(time,X_rec);
subplot(2,1,2); hold on; grid on;
plot(time,U_rec);

