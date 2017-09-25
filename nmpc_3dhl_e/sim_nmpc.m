% NMPC Simulation
% ----------------
close all; clear all; clc;

% initial conditions
Ts_step = 0.1; % step size in nmpc
N = 80;
n_U = 3;
n_X = 12; % number of nmpc states
n_Y = 6+4;
n_Z = 4;

% wind
wn=0;
we=0;
wd=0;

% params
alpha_p_co = deg2rad(8);        % angle of attack upper cut-off
alpha_m_co = deg2rad(-3);       % angle of attack lower cut-off
alpha_delta_co = deg2rad(2);    % angle of attack cut-off transition length\
p_n = 100;
p_e = 100;
p_d = 0;

% model parameters
load parameters_20161209.mat;

u_trim = [0.375, 0, 1.7*pi/180];

% initial conditions
ic_u    = u_trim;
ic_ned  = [0, 0, 0];
ic_vV   = [14, 0, 0];
ic_att  = [0, 0];
ic_attdot = [0, 0, 0];
ic_augm = [ic_u(1)];
ic_od   = [wn, we, wd, ...
    alpha_p_co, alpha_m_co, alpha_delta_co, ...
    p_n, p_e, p_d];

% acado inputs
nmpc_ic.x   = [ic_ned,ic_vV,ic_att,ic_attdot,ic_augm]; 
nmpc_ic.u   = ic_u;
yref        = [0 0, 14, 0 0 0, 0];
zref        = [0, u_trim];

Q_scale     = [1 1, 1, deg2rad(50) deg2rad(50) deg2rad(50), 1];
R_scale     = [1, 1 deg2rad(30) deg2rad(15)];

Q_output    = [.05 1, 10, 20 20 5, 1]./Q_scale.^2;
QN_output   = [.05 1, 10, 20 20 5, 1]./Q_scale.^2;
R_controls  = [40 30 50 50]./R_scale.^2;

input.x     = repmat(nmpc_ic.x, N+1,1);
input.u     = repmat(nmpc_ic.u, N,1);
input.y     = repmat([yref, zref], N,1);
input.yN    = input.y(1, 1:length(yref))';
input.od    = repmat(ic_od, N+1, 1);
input.W     = repmat(diag([Q_output, R_controls]), [N 1]);
input.WN    = diag(QN_output);

% simulation
T0      = 0;
Tf      = 100;
Ts      = 0.01;
time    = T0:Ts:Tf;
KKT_MPC = []; INFO_MPC = []; controls_MPC = [];

Ts_nmpc = 0.05; % interval between nmpc calls

% initial simout
X0          = nmpc_ic.x;
simout      = nmpc_ic.x(1:12);
states      = simout;
d_states    = [...
    ic_vV(1)*cos(ic_vV(3))*cos(ic_vV(2))+wn, ...
    ic_vV(1)*sin(ic_vV(3))*cos(ic_vV(2))+we, ...
    -ic_vV(1)*sin(ic_vV(2))+wd, ...
    0,0,0, ...
    0,0, ...
    0,0,0, ...
    0];
U0 = ic_u;
path_idx = 1;
endofwaypoints=false;
for k = 1:length(time)
    
    % set position setpoint
    [p1] = set_position_setpoint(states(1:3),norm([wn,we,wd]),states(4),[p_n,p_e,p_d],Ts_step,N);
    ic_od = [wn, we, wd, ...
        alpha_p_co, alpha_m_co, alpha_delta_co, ...
        p1(1), p1(2), p1(3)];
    input.od    = repmat(ic_od, N+1, 1);
    
    % measure
    input.x0    = X0';
    
    if time(k)==floor(time(k)/Ts_nmpc)*Ts_nmpc
    
        %  - - - - - START NMPC - - - - -
        st_nmpc = tic;

         % shift initial states and controls
        if k > 1
            input.x     = output.x;
            input.u     = output.u;
        end

        % generate controls
        output = acado_nmpc_3dhl_e_step(input);

        U0  = output.u(1,:);

        % record info
        INFO_MPC = [INFO_MPC; output.info];
        KKT_MPC = [KKT_MPC; output.info.kktValue];

        % augmented states measurement update
        X0(12) = output.x(1,12) + (output.x(2,12) - output.x(1,12))*Ts_nmpc/Ts_step;

        tsolve(k) = toc(st_nmpc);
    
    end
    % - - - - - END NMPC - - - - -
    
    % apply control
    [d_states, simout]  = uav_dyn(time(k), states, U0, [wn,we,wd], parameters);

    % integration (model propagation)
    states = states + d_states*Ts;
    
    % measurement update
    X0(1:11) = simout(1:11);
    
    % record states/controls
    X_rec(k,:) = [simout];
    horiz_rec(:,k,:) = output.x(:,:);
    horiz_rec_U(:,k,:) = output.u(:,:);
    U_rec(k,:) = U0;
    Y_rec(k,:) = eval_obj([X0,U0,ic_od],[n_X,n_U]);
    aux_rec(k,:) = [p1];

end

plotsim