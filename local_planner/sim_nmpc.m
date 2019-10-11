% NMPC Simulation
% ----------------
clear all; clc; close all; 

% add function paths
if isempty(strfind(path, ['/home/thomas/Documents/ACADOscripts/local_planner/functions', pathsep]))
    addpath('/home/thomas/Documents/ACADOscripts/local_planner/functions');
end

% load model parameters / config
load('model_config/model_params.mat');
run('model_config/sysid_config_Techpod.m');

%% NMPC SETUP -------------------------------------------------------------
N = 40;
n_U = 3;
n_X = 9; % number of nmpc states
n_Y = 9;
n_Z = 3;
n_OD = 34+3249;%841;

Ts_step = 0.1; % step size in nmpc
Ts_nmpc = 0.1; % interval between nmpc calls

%% ONLINE DATA ------------------------------------------------------------

% environment
rho = 1.15;

% disturbances
w_n = 0;
w_e = 0;
w_d = 0;

% path reference
b_n = 0;
b_e = 0;
b_d = -50;
Gamma_p = deg2rad(5);
chi_p = deg2rad(0);

% guidance
T_b_lat = 5;
T_b_lon = 5;
gamma_app_max = deg2rad(17)*2/pi;

% control augmented attitude time constants and gains
tau_phi = 0.5;
tau_theta = 0.5;
k_phi = 1.1;
k_theta = 1.1;

% angle of attack soft constraint
delta_aoa = deg2rad(3);
aoa_m = deg2rad(-5);
aoa_p = deg2rad(8);
sig_aoa_1 = 0.001;

% height soft constraint 
h_offset = 10;
delta_h = 20;
delta_y = 5;
sig_h_1 = 0.001;

% radial soft constraint
r_offset = 0;
delta_r0 = 10;
k_r = 3/tand(35)/9.81;
sig_r_1 = 0.001;

% terrain lookup
len_local_idx_n = 57;%29;
len_local_idx_e = 57;%29;
terr_dis = 5;
terrain_constructor;

%% REFERENCES -------------------------------------------------------------

v_ref = 14;

%% OPTIONS ----------------------------------------------------------------

% horizon handling
shift_states_controls = false;

% defines
USE_EXP_SOFT_COST = 0;
USE_OCC_GRAD_AS_GUIDANCE = 0;
defines = [USE_EXP_SOFT_COST;USE_OCC_GRAD_AS_GUIDANCE];

%% INITIALIZATION ---------------------------------------------------------

% initial states
x_init = [ ...
    -5, 100, -20, ... % r_n, r_e, r_d
    14, deg2rad(0), deg2rad(-20), ... % v, gamma, xi
    deg2rad(0), deg2rad(2), ... % phi, theta
    104 ... % n_p
    ];
posk = x_init(1:3);

% initial controls
u_init = [0.5 0 deg2rad(3)]; % u_T, phi_ref, theta_ref

% acado inputs
nmpc_ic.x = x_init;
nmpc_ic.u = u_init;
yref = [0 0 0 0 0 v_ref 0 0 0];
zref = [0.5 0 deg2rad(3)];

Q_scale = [1 1 1 1 1 1 1 1 1];
R_scale = [0.1 deg2rad(1) deg2rad(1)];

Q_output    = [0 0, 1e2 1e2 1e2, 5e2, 1e8 1e7 1e4]./Q_scale.^2;
QN_output   = [0 0, 1e2 1e2 1e2, 5e2, 1e8 1e7 1e4]./Q_scale.^2;
R_controls  = [1e1 1e0 1e1]./R_scale.^2;

% weight dependent parameters
log_sqrt_w_over_sig1_aoa = sqrt(Q_output(7)/sig_aoa_1);
one_over_sqrt_w_aoa = 1/max(sqrt(Q_output(7)),0.00001);
log_sqrt_w_over_sig1_h = sqrt(Q_output(8)/sig_h_1);
one_over_sqrt_w_h = 1/max(sqrt(Q_output(8)),0.00001);
log_sqrt_w_over_sig1_r = sqrt(Q_output(9)/sig_r_1);
one_over_sqrt_w_r = 1/max(sqrt(Q_output(9)),0.00001);

% initial online data
get_local_terrain;
onlinedatak = [ ...
    rho, w_n, w_e, w_d, ...
    b_n, b_e, b_d, Gamma_p, chi_p, ...
    T_b_lat, T_b_lon, gamma_app_max, ...
    tau_phi, tau_theta, k_phi, k_theta, ...
    delta_aoa, aoa_m, aoa_p, log_sqrt_w_over_sig1_aoa, one_over_sqrt_w_aoa, ...
    h_offset, delta_h, delta_y, log_sqrt_w_over_sig1_h, one_over_sqrt_w_h, ...
    r_offset, delta_r0, k_r, log_sqrt_w_over_sig1_r, one_over_sqrt_w_r, ...
    terr_local_origin_n, terr_local_origin_e, terr_dis, terrain_data ...
    ];

input.x     = repmat(nmpc_ic.x, N+1,1);
input.u     = repmat(nmpc_ic.u, N,1);
input.y     = repmat([yref, zref], N,1);
input.yN    = yref;
input.od    = repmat(onlinedatak, N+1, 1);
% input.od    = [onlinedatak; zeros(N, n_OD)];
input.W     = repmat(diag([Q_output, R_controls]), [N 1]);
R_slew = 100;
zero2one = linspace(0,1,N);
for i = 1:N
    input.W((n_Y+n_Z)*(i-1)+n_Y+1:(n_Y+n_Z)*(i-1)+n_Y+n_Z,:) = ...
        input.W((n_Y+n_Z)*(i-1)+n_Y+1:(n_Y+n_Z)*(i-1)+n_Y+n_Z,:) * ...
        (1 + R_slew*(1 - zero2one(i))^2);
end
% input.W     = diag([Q_output, R_controls]);
input.WN    = diag(QN_output);

%% SIMULATION -------------------------------------------------------------
T0 = 0;
Tf = 30;
Ts = 0.01;
time = T0:Ts:Tf;
len_t = length(time);

% initial simout
states = nmpc_ic.x;
measurements = states;
controls = nmpc_ic.u;
ctrl_hor = repmat(controls, [N,1]);
[dstates,simout] = model_dynamics(0,measurements,controls,input.od(1,:),model_params,sysid_config);

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
rec.aux = zeros(len_t,38);

% simulate
for k = 1:len_t
    
    st_sim = tic;
    
    % measure
    input.x0 = measurements;
    
    if mod(k,nmpc_counter_interval)==0 || k==1
        
        nmpc_counter = nmpc_counter+1;
        
        % START NMPC - - - - -
        st_nmpc = tic;

         % shift (or dont) initial states and controls
        if k > 1
            if shift_states_controls
                % shift
                v_end = output.x(end,4)*[...
                    cos(output.x(end,5))*cos(output.x(end,6)), ...
                    cos(output.x(end,5))*sin(output.x(end,6)), ...
                    -sin(output.x(end,5))] + ...
                    [w_n, w_e, w_d];
                input.x = [output.x(2:end,:); [output.x(end,1:3)+v_end*Ts_nmpc output.x(end,4:end)] ];
                input.u = [output.u(2:end,:); output.u(end,:)];
            else
                % dont
                input.x = output.x;
                input.u = output.u;
            end
        end
        
        % update online data
        st_array_allo = tic;
        posk = input.x0(1:3);
        get_local_terrain;
        onlinedatak = [ ...
            rho, w_n, w_e, w_d, ...
            b_n, b_e, b_d, Gamma_p, chi_p, ...
            T_b_lat, T_b_lon, gamma_app_max, ...
            tau_phi, tau_theta, k_phi, k_theta, ...
            delta_aoa, aoa_m, aoa_p, log_sqrt_w_over_sig1_aoa, one_over_sqrt_w_aoa, ...
            h_offset, delta_h, delta_y, log_sqrt_w_over_sig1_h, one_over_sqrt_w_h, ...
            r_offset, delta_r0, k_r, log_sqrt_w_over_sig1_r, one_over_sqrt_w_r, ...
            terr_local_origin_n, terr_local_origin_e, terr_dis, terrain_data ...
            ];
        input.od = repmat(onlinedatak, N+1, 1);
%         input.od(1,:) = onlinedatak;
        input.y(:,end-2:end) = ctrl_hor;
        tarray_k = toc(st_array_allo);

        % generate controls
        output = acado_nmpc_step(input);

        controls  = output.u(1,:);
        ctrl_hor = output.u(:,:);

        % record info
        INFO_MPC(nmpc_counter) = output.info;
        KKT_MPC(nmpc_counter) = output.info.kktValue;

        tsolve_k = toc(st_nmpc);
        nmpc_executed(k) = 1;
    end
    % END NMPC - - - - -
    
    % apply control
    [dstates, simout]  = model_dynamics(time(k), states, controls, input.od(1,:),model_params,sysid_config);

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
    [out,aux] = eval_obj([simout,controls,input.od(1,:)], len_local_idx_n, len_local_idx_e, defines);
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