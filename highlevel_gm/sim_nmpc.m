% NMPC Simulation
% ----------------
close all; clear all; clc;

%% NMPC SETUP -------------------------------------------------------------
N = 100;
n_U = 2;
n_X = 6; % number of nmpc states
n_Y = 4;
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
b_d = -10;
Gamma_p = deg2rad(0);
chi_p = 0;

% terrain data
len_local_idx_n = 61;
len_local_idx_e = 61;
delta_h = 10;
terrain_constructor;

%% INITIALIZATION ---------------------------------------------------------

% initial states
posk = [-5, 20, -15];
attk = [deg2rad(0), deg2rad(-30), deg2rad(0)];

% initial controls
inputk = [0 0];

% initial online data
get_local_terrain;
onlinedatak = [v w_n w_e w_d ...
    b_n b_e b_d Gamma_p chi_p ...
    delta_h terr_local_origin_n terr_local_origin_e terrain_data];

% acado inputs
nmpc_ic.x = [posk, attk]; 
nmpc_ic.u = inputk;
yref = [0 0 0 0];
zref = [Gamma_p 0 0 0];

Q_scale = [1 1 1 1];
R_scale = [deg2rad(1) deg2rad(1) deg2rad(5) deg2rad(5)];

Q_output    = [10 10 1 10000]./Q_scale.^2;
QN_output   = [10 10 1 10000]./Q_scale.^2;
R_controls  = [1 1 1 1]./R_scale.^2;

input.x     = repmat(nmpc_ic.x, N+1,1);
input.u     = repmat(nmpc_ic.u, N,1);
input.y     = repmat([yref, zref], N,1);
input.yN    = yref;
input.od    = repmat(onlinedatak, N+1, 1);
input.W     = repmat(diag([Q_output, R_controls]), [N 1]);
input.WN    = diag(QN_output);

%% SIMULATION -------------------------------------------------------------
T0 = 0;
Tf = 100;
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
tsolve_k = 0;
tarray_k = 0;
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
        
        % update online data
        posk = input.x0(1:3);
        st_array_allo = tic;
        get_local_terrain;
        onlinedatak = [v w_n w_e w_d ...
            b_n b_e b_d Gamma_p chi_p ...
            delta_h terr_local_origin_n terr_local_origin_e terrain_data];
        input.od    = repmat(onlinedatak, N+1, 1);
        tarray_k = toc(st_array_allo);

        % generate controls
        output = acado_nmpc_step(input);

        controls  = output.u(1,:);

        % record info
        INFO_MPC = [INFO_MPC; output.info];
        KKT_MPC = [KKT_MPC; output.info.kktValue];

        % timing
        tsolve_k = toc(st_nmpc);
    
    end
    % END NMPC - - - - -
    tarray(k) = tarray_k;
    tsolve(k) = tsolve_k;
    
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
    [out,aux] = eval_obj([simout,controls,input.od(1,:)]);
    rec.yz(k,:) = out;
    rec.aux(k,:) = aux;
    
    % tell time
    if time(k)==floor(time(k)/10)*10
        clc;
        disp(['sim time = ',num2str(time(k)),' s']);
    end
    
end

%% PLOTTING ---------------------------------------------------------------

plot_sim