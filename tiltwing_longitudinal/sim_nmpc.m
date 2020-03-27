% NMPC Simulation
% ----------------
clear; clc; close all; 

% add function paths
addpath('functions');

%% NMPC SETUP - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
N = 20;
n_U = 3;
n_X = 4;
n_Ys = 7;
n_Yc = 3;
n_OD = 0;

Ts_step = 0.1; % step size in nmpc
Ts_nmpc = 0.1; % interval between nmpc calls

%% OPTIONS - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

% shift the states and controls one step for the new iteration's horizon initial guess
shift_states_controls = false;

%% INITIALIZATION - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

% initial states
x_init = [0.0, 0.0, 0.0, 0.0]; % XXX: put something realistic here!

% initial controls
u_init = [0.0, 0.0, 0.0]; % XXX: put something realistic here!

% initial control constraints
constraint_lb = [0.0, -1.0, 0.0];
constraint_ub = [pi/2, 1.0, 30.0];

% initial condition struct
nmpc_ic.x = x_init;
nmpc_ic.u = u_init;

% objective references -- these are where the NMPC will be attempting to
%                         drive the objective states
% XXX: define
y_ref = [...
    v_x_ref;
    v_z_ref;
    theta_ref;
    zeta_w_ref;
    delta_w;
    T_w_ref;
    theta_ref;
    ];

% objective weights
Q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; % XXX: put something realistic here
QN = Q(1:n_Ys); % << only state dependent (no controls)!

%% ACADO MEX INPUT STRUCTURE - - - - - - - - - - - - - - - - - - - - - - - 

% the structure must contain the following fields:
% x     states
% u     controls
% y     objective references
% yN    end term objective references
% od    online data (if defined)
% W     objective weights
% WN    objective weights for end term

% input struct
input.x = repmat(nmpc_ic.x, N+1,1);
input.u = repmat(nmpc_ic.u, N,1);
input.y = repmat(y_ref, N,1);
input.yN = yref(1:n_Ys); % << only state dependent (no controls)!
% input.od = repmat(online_data, N+1, n_OD);
input.W = diag(Q);
input.WN = diag(QN);
input.lbValues = reshape(repmat(constraints_lb, N, 1).',N*n_U,1);
input.ubValues = reshape(repmat(constraints_ub, N, 1).',N*n_U,1);

%% SIMULATION - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

T0 = 0;     % sim start time
Tf = 20;    % sim end time
Ts = 0.01;  % simulation time step -- the model will be propagated at this rate
time = T0:Ts:Tf;
len_t = length(time);
len_nmpc = floor(len_t*Ts/Ts_nmpc); % if the MPC is running at a different rate than the simulation time

% initial simout
states = nmpc_ic.x;
measurements = states;
controls = nmpc_ic.u;
[dstates,simout] = model_dynamics(0,measurements,controls,online_data,model_params); % XXX: define what actual inputs you use, 0 here is t=0

% allocate timing arrays
tsolve_k = 0;
tarray_k = 0;
trec = zeros(len_t,1);
tarray = zeros(len_t,1);
tsolve = zeros(len_t,1);
tsim = zeros(len_t,1);

% nmpc counter
nmpc_executed = zeros(len_t,1);
nmpc_counter_interval = floor(Ts_nmpc/Ts);
nmpc_counter = 0;

% allocate nmpc info
len_nmpc_storage = floor(len_t/nmpc_counter_interval)+1;
KKT_MPC = zeros(len_nmpc_storage,1);
INFO_MPC(len_nmpc_storage) = struct('status',0,'cpuTime',0,'kktValue',0,'objValue',0,'QP_iter',0,'QP_violation',0);

% allocate record struct
rec.x = zeros(len_t,n_X);
rec.u = zeros(len_t,n_U);
rec.x_hor = zeros(N+1,len_t,n_X);
rec.u_hor = zeros(N,len_t,n_U);

k_nmpc = 0;

% simulate
for k = 1:len_t
    
    st_sim = tic;
    
    % measure -- we are pretending to measure or estimate states
    input.x0 = measurements;
    
    if mod(k,nmpc_counter_interval)==0 || k==1 % only execute the mpc at the defined iteration rate
        
        nmpc_counter = nmpc_counter+1;
        
        % START NMPC - - - - -
        st_nmpc = tic;
        
        % update states - - - - - - - - - - - - - - - - - - - - - - - - - -
        st_array_allo = tic;

        % shift (or dont) initial states and controls
        if k > 1
            
            if shift_states_controls
                % shift
                input.x = [output.x(2:end,:); output.x(end,:)]; % copy last horizon state to end
                input.u = [output.u(2:end,:); output.u(end,:)];
            else
                % dont
                input.x = output.x;
                input.u = output.u;
            end
        end
        
        % array allocation timing
        tarray_k = toc(st_array_allo);

        % control - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        
        % generate controls
        output = acado_nmpc_step(input);
        
        % set controls -- this is applied to the vehicle
        controls  = output.u(1,:);
        
        % record - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        
        % record info
        INFO_MPC(nmpc_counter) = output.info;
        KKT_MPC(nmpc_counter) = output.info.kktValue;

        tsolve_k = toc(st_nmpc);
        nmpc_executed(k) = 1;
    end
    % END NMPC - - - - -
    
    % apply control
    [dstates, simout]  = model_dynamics(time(k), states, controls, online_data, model_params);

    % integration (model propagation)
    states = states + dstates*Ts;
    
    % measurement update
    measurements = states;
    
    % record states/controls
    st_rec = tic;
    
    rec.x(k,:) = simout;
    rec.u(k,:) = controls;
    rec.x_hor(:,k,:) = output.x(:,:);
    rec.u_hor(:,k,:) = output.u(:,:);
    
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

%% PLOTTING - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

% XXX: make a plot script you can call here.