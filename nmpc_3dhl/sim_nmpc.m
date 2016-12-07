% NMPC Simulation
% ----------------
close all; clear all; clc;

% initial conditions
N = 40;
n_U = 3;
n_X = 14; % number of nmpc states
n_Y = 18;
n_Z = 7;

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
wn=0;
we=0;
wd=0;

% switch conditions
R_acpt = 30;    % switching acceptance radius
ceta_acpt = 0.8; % switching acceptance cosine of error angle

% gains
k_t_d = 6/15;     % longitudinal logistic gain
e_d_co = 30;    % longitudinal logistic cutoff

k_t_ne = 6/15;    % lateral logistic gain
e_ne_co = 60;   % lateral logistic cutoff

eps_v = 0.5;     % unit ground speed threshold

alpha_p_co = 8*pi/180;   % angle of attack upper cutoff
alpha_m_co = -3*pi/180;   % angle of attack lower cutoff
alpha_delta_co = 2*pi/180;   % angle of attack cutoff transition length

i_e_t_co = 7;
W_i_e_t=0;


% model parameters
load parameters_20161117.mat;

% initial conditions
ic_ned  = [-300, 0, 10];
ic_vV   = [13.5, 0, 0];
ic_att  = [0, 0];
ic_attdot = [0, 0, 0];
ic_u    = [0.375, 0, 1.7*pi/180];
ic_augm = [ic_u(1), 0, 0];

ic_od   = [pparams, pparams_next, R_acpt, ceta_acpt, ...
    wn, we, wd, ...
    k_t_d, e_d_co, k_t_ne, e_ne_co, eps_v, ...
    alpha_p_co, alpha_m_co, alpha_delta_co, ...
    i_e_t_co];

% acado inputs
nmpc_ic.x   = [ic_ned,ic_vV,ic_att,ic_attdot,ic_augm]; 
nmpc_ic.u   = ic_u;
yref        = [0 0 0, 0 0 0, 13.5, 0 0 0, 0];
zref        = [0, ic_u, ic_u];

% y   = [ e_ne_1; e_d_1; i_e_t; e_vbar_1_n; e_vbar_1_e; e_vbar_1_d; Vsafe; p; q; r; a_soft ];

Q_scale     = [1 1 1, 1 1 1, 1, 50*pi/180 50*pi/180 50*pi/180, 1];
R_scale     = [1, 1 30*pi/180 15*pi/180, 1 5*pi/180 5*pi/180];

Q_output    = [70 150 3 10 10 5 5 30 30 5 10]./Q_scale.^2;
QN_output   = [70 150 3 10 10 5 5 30 30 5 10]./Q_scale.^2;
R_controls  = [100 15 5 20 0 20 30]./R_scale.^2;

Q_prev      = [(linspace(1,0,N+1)').^0,...
    (linspace(1,0,N+1)').^0,...
    (linspace(1,0,N+1)').^0];

input.x     = repmat(nmpc_ic.x, N+1,1);
input.u     = repmat(nmpc_ic.u, N,1);
input.y     = repmat([yref, zref], N,1);
input.yN    = input.y(1, 1:length(yref))';
input.od    = repmat(ic_od, N+1, 1);
QR = [Q_output(1:2),Q_output(3)*W_i_e_t,Q_output(4:end), R_controls(1:4)];
for i = 1:N
input.W(n_Y*(i-1)+1:n_Y*i,:) = diag([QR, (R_controls(5:7)).*Q_prev(i,:)]);
end
load QN.mat;
input.WN    = diag([QN_output(1:2),QN_output(3)*W_i_e_t,QN_output(4:end)]);%Py;%

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
    
    % check path
    if pparams(1) < 0.5
        path_checks(k) = check_line_seg(states(1:3),d_states(1:3),ic_od(2:end));
    elseif pparams(1) < 1.5
        path_checks(k) = check_curve_seg(states(1:3),d_states(1:3),ic_od(2:end));
    end
    if path_checks(k)
        if path_idx<length(paths)-1
            for i = 1:length(pparams)
                pparams(i) = pparams_next(i);
                eval(['pparams_next(i) = paths(path_idx+2).pparam',int2str(i),';']);
            end
            path_idx = path_idx + 1;
            ic_od = [pparams, pparams_next, R_acpt, ceta_acpt, ...
                wn, we, wd, ...
                k_t_d, e_d_co, k_t_ne, e_ne_co, eps_v, ...
                alpha_p_co, alpha_m_co, alpha_delta_co, ...
                i_e_t_co];
            input.od = repmat(ic_od, N+1, 1);
            output.x(:,end-1) = zeros(N+1,1);
            output.x(:,end) = zeros(N+1,1);
            X0(end-1) = 0; % i_e_t
            X0(end) = 0; % sw
        elseif ~endofwaypoints
            endofwaypoints=true;
            for i = 1:length(pparams)
                pparams(i) = pparams_next(i);
%                 eval(['pparams_next(i) = paths(path_idx+1).pparam',int2str(i),';']);
            end
            path_idx = path_idx + 1;
            ic_od = [pparams, pparams_next, R_acpt, ceta_acpt, ...
                wn, we, wd, ...
                k_t_d, e_d_co, k_t_ne, e_ne_co, eps_v, ...
                alpha_p_co, alpha_m_co, alpha_delta_co, ...
                i_e_t_co];
            input.od = repmat(ic_od, N+1, 1);
            output.x(:,end-1) = zeros(N+1,1);
            output.x(:,end) = zeros(N+1,1);
            X0(end-1) = 0; % i_e_t
            X0(end) = 0; % sw
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
%             input.x =spline(0:Ts_step:(N*Ts_step),output.x',Ts_nmpc:Ts_step:(N*Ts_step+Ts_nmpc))';
            input.u     = output.u;%[output.u(2:end,:); output.u(end,:)]; %
%             input.u = spline(0:Ts_step:((N-1)*Ts_step),output.u',Ts_nmpc:Ts_step:((N-1)*Ts_step+Ts_nmpc))';
            input.y(:,(end-2):end) = output.u;%[output.u(1,1)*ones(N,1),output.u(:,2:3)];%repmat(output.u(1,:),N,1);%
        end

        % generate controls
        output = acado_nmpc_ext_step(input);

        U0  = output.u(1,:);

        % record info
        INFO_MPC = [INFO_MPC; output.info];
        KKT_MPC = [KKT_MPC; output.info.kktValue];

%         % shift initial states and controls
%         input.x     = [output.x(2:N+1,:); output.x(N+1,:)];
%         input.u     = [output.u(2:N,:); output.u(N,:)];
%         X0(10:15)   = output.x(2,10:15);

        % augmented states measurement update
        X0(12:14) = output.x(1,12:14) + (output.x(2,12:14) - output.x(1,12:14))*Ts_nmpc/Ts_step;
%         X0((n_X-n_XA-n_Xno+1):n_X) = output.x(1,(n_X-n_XA+1):n_X);

        tsolve(k) = toc(st_nmpc);
    
    end
    % - - - - - END NMPC - - - - -
    
    % apply control
    [d_states, simout]  = uav_dyn(time(k), states, U0, [wn,we,wd], parameters);

    % integration (model propagation)
    states = states + d_states*Ts;
    
    % measurement update
    X0(1:11) = simout(1:11);
%     X0(12) = U0(1);
    
    % record states/controls
    X_rec(k,:) = [simout, X0(13:14)];
    horiz_rec(:,k,:) = output.x(:,:);
    horiz_rec_U(:,k,:) = output.u(:,:);
    U_rec(k,:) = U0;
    J_rec(k,:) = calculate_cost([X0,U0,ic_od],[n_X,n_U]);
    
    e_t = norm(J_rec(k,1:2));
    if (e_t < i_e_t_co*0.8), W_i_e_t = 1;
    elseif (e_t < i_e_t_co), W_i_e_t = cos((e_t/i_e_t_co-0.8)/0.2*3.141592653589793) * 0.5 + 0.5;
    end
    for i = 1:N
        input.W(n_Y*(i-1)+3,3) = Q_output(3)*W_i_e_t;
    end
    input.WN(3,3) = QN_output(3)*W_i_e_t;
end

plotsim