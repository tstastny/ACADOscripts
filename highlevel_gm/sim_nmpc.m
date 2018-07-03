% NMPC Simulation
% ----------------
close all; clear all; clc;

%% NMPC SETUP -------------------------------------------------------------
N = 100;
n_U = 2;
n_X = 6; % number of nmpc states
n_Y = 2;
n_Z = 4;

Ts_step = 0.1; % step size in nmpc
Ts_nmpc = 0.1; % interval between nmpc calls

%% ONLINE DATA ------------------------------------------------------------

% flight condition
v = 9.6;

% disturbances
w_n = -2;
w_e = 5;
w_d = 0;

% path reference
b_n = 0;
b_e = 0;
b_d = 0;
Gamma_p = deg2rad(-5);
chi_p = 0;

% terrain data
terrain_data = zeros(5);

%% INITIALIZATION ---------------------------------------------------------

% initial states
ic_r = [-60, 20, 0];
ic_att = [deg2rad(0), deg2rad(-30), deg2rad(0)];

% initial controls
ic_u = [0 0];

% initial online data
ic_od = [v w_n w_e w_d b_n b_e b_d Gamma_p chi_p reshape(terrain_data.',1,[])];

% acado inputs
nmpc_ic.x = [ic_r, ic_att]; 
nmpc_ic.u = ic_u;
yref = [0 0];
zref = [Gamma_p 0 0 0];

Q_scale = [1 1];
R_scale = [deg2rad(1) deg2rad(1) deg2rad(5) deg2rad(5)];

Q_output    = [10 10]./Q_scale.^2;
QN_output   = [10 10]./Q_scale.^2;
R_controls  = [1 1 1 1]./R_scale.^2;

input.x     = repmat(nmpc_ic.x, N+1,1);
input.u     = repmat(nmpc_ic.u, N,1);
input.y     = repmat([yref, zref], N,1);
input.yN    = yref;
input.od    = repmat(ic_od, N+1, 1);
input.W     = repmat(diag([Q_output, R_controls]), [N 1]);
input.WN    = diag(QN_output);

%% SIMULATION -------------------------------------------------------------
T0 = 0;
Tf = 30;
Ts = 0.01;
time = T0:Ts:Tf;
KKT_MPC = []; INFO_MPC = []; controls_MPC = [];

% initial simout
states = nmpc_ic.x;
measurements = states;
controls = nmpc_ic.u;
[dstates,simout] = uav_dyn(0,measurements,controls,input.od(1,:));

% properly init position states
input.x(:,1:3) = input.x(:,1:3) + repmat(dstates(1:3),N+1,1).*repmat((0:Ts_nmpc:N*Ts_nmpc)',1,3);

for k = 1:length(time)
    
    % measure
    input.x0 = measurements;
    
    if time(k)==floor(time(k)/Ts_nmpc)*Ts_nmpc
    
        % START NMPC - - - - -
        st_nmpc = tic;

         % shift initial states and controls
        if k > 1
            input.x     = output.x;
            input.u     = output.u;
        end

        % generate controls
        output = acado_nmpc_step(input);

        controls  = output.u(1,:);

        % record info
        INFO_MPC = [INFO_MPC; output.info];
        KKT_MPC = [KKT_MPC; output.info.kktValue];

        % timing
        tsolve(k) = toc(st_nmpc);
    
    end
    % END NMPC - - - - -
    
    % apply control
    [dstates, simout]  = uav_dyn(time(k), states, controls, input.od(1,:));

    % integration (model propagation)
    states = states + dstates*Ts;
    
    % measurement update
    measurements = states;
    
    % record states/controls
    rec.x(k,:) = simout;
    rec.x_hor(:,k,:) = output.x(:,:);
    rec.u_hor(:,k,:) = output.u(:,:);
    rec.u(k,:) = controls;
    [out,aux] = eval_obj(simout,controls,input.od(1,:));
    rec.yz(k,:) = out;
    rec.aux(k,:) = aux;
    
end

%% PLOTTING ---------------------------------------------------------------

plotsim