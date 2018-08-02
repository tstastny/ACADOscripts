% NMPC Simulation
% ----------------
close all; clear all; clc;
if isempty(strfind(path, ['/home/thomas/Documents/ACADOscripts/highlevel_gm/functions', pathsep]))
    addpath('/home/thomas/Documents/ACADOscripts/highlevel_gm/functions');
end

%% NMPC SETUP -------------------------------------------------------------
N = 70;
n_U = 2;
n_X = 6; % number of nmpc states
n_Y = 3;
n_Z = 4;

Ts_step = 0.1; % step size in nmpc
Ts_nmpc = 0.1; % interval between nmpc calls

%% ONLINE DATA ------------------------------------------------------------

% flight condition
v = 14;

% disturbances
w_n = 0;
w_e = 0;
w_d = 0;

% path reference
b_n = 0;
b_e = 0;
b_d = -15;
Gamma_p = deg2rad(0);
chi_p = 0;

% guidance
T_b_lat = 5;
T_b_lon = 5;

% terrain data
len_local_idx_n = 61;
len_local_idx_e = 61;
delta_h = 10;
terrain_constructor;

%% INITIALIZATION ---------------------------------------------------------

% initial states
posk = [-5, 100, -15];
attk = [deg2rad(0), deg2rad(rad2deg(-1.123276351637727)+180), deg2rad(0)];

% initial controls
inputk = [0 0];

% initial online data
get_local_terrain;
onlinedatak = [v, w_n, w_e, w_d, ...
    b_n, b_e, b_d, Gamma_p, chi_p, ...
    T_b_lat, T_b_lon, ...
    delta_h, terr_local_origin_n, terr_local_origin_e, terrain_data];

% acado inputs
nmpc_ic.x = [posk, attk]; 
nmpc_ic.u = inputk;
yref = [0 0 0];
zref = [0 0 0 0];

Q_scale = [deg2rad(1) deg2rad(1) 1];
R_scale = [deg2rad(1) deg2rad(1) deg2rad(5) deg2rad(5)];

Q_output    = [5 5 100000]./Q_scale.^2;
QN_output   = [5 5 100000]./Q_scale.^2;
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
Tf = 200;
Ts = 0.01;
time = T0:Ts:Tf;
len_t = length(time);

% initial simout
states = nmpc_ic.x;
measurements = states;
controls = nmpc_ic.u;
[dstates,simout] = uav_dyn(0,measurements,controls,input.od(1,:));

% properly init position states
input.x(:,1:3) = input.x(:,1:3) + repmat(dstates(1:3),N+1,1).*repmat((0:Ts_nmpc:N*Ts_nmpc)',1,3);

% [out,aux] = eval_obj([simout,controls,input.od(1,:)]);

% init arrays
tsolve_k = 0;
tarray_k = 0;
trec = zeros(len_t,1);
tarray = zeros(len_t,1);
tsolve = zeros(len_t,1);
tsim = zeros(len_t,1);
nmpc_executed = zeros(len_t,1);
nmpc_counter_interval = floor(Ts_nmpc/Ts);
len_nmpc_storage = floor(len_t/nmpc_counter_interval)+1;
KKT_MPC = zeros(len_nmpc_storage,1);
INFO_MPC(len_nmpc_storage) = struct('status',0,'cpuTime',0,'kktValue',0,'objValue',0,'QP_iter',0,'QP_violation',0);
nmpc_counter = 0;
rec.x = zeros(len_t,n_X);
rec.x_hor = zeros(N+1,len_t,n_X);
rec.u_hor = zeros(N,len_t,n_U);
rec.u = zeros(len_t,n_U);
rec.yz = zeros(len_t,n_Y+n_Z);
rec.aux = zeros(len_t,7);

% simulate
for k = 1:len_t
    
    st_sim = tic;
    
    % measure
    input.x0 = measurements;
    
    if mod(k,nmpc_counter_interval)==0 || k==1
        
        nmpc_counter = nmpc_counter+1;
        
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
        onlinedatak = [v, w_n, w_e, w_d, ...
            b_n, b_e, b_d, Gamma_p, chi_p, ...
            T_b_lat, T_b_lon, ...
            delta_h, terr_local_origin_n, terr_local_origin_e, terrain_data];
        input.od    = repmat(onlinedatak, N+1, 1);
        tarray_k = toc(st_array_allo);

        % generate controls
        output = acado_nmpc_step(input);

        controls  = output.u(1,:);

        % record info
        INFO_MPC(nmpc_counter) = output.info;
        KKT_MPC(nmpc_counter) = output.info.kktValue;

        tsolve_k = toc(st_nmpc);
        nmpc_executed(k) = 1;
    end
    % END NMPC - - - - -
    
    % apply control
    [dstates, simout]  = uav_dyn(time(k), states, controls, input.od(1,:));

    % integration (model propagation)
    states = states + dstates*Ts;
    
    % measurement update
    measurements = states;
    
    % record states/controls
    st_rec = tic;
    rec.x(k,:) = simout;
    rec.x_hor(:,k,:) = output.x(:,:);
    rec.u_hor(:,k,:) = output.u(:,:);
    rec.u(k,:) = controls;
    if time(k)>52
        stoppp=1;
    end
    [out,aux] = eval_obj([simout,controls,input.od(1,:)]);
    rec.yz(k,:) = out;
    rec.aux(k,:) = aux;
    trec(k) = toc(st_rec);
    
    % tell time
    if time(k)==floor(time(k)/10)*10
        clc;
        disp(['sim time = ',num2str(time(k)),' s']);
    end
    
    tarray(k) = tarray_k;
    tsolve(k) = tsolve_k;
    tsim(k) = toc(st_sim);
end

%% PLOTTING ---------------------------------------------------------------

plot_sim