% NMPC Simulation
% ----------------
close all; clear all; clc;

% initial conditions
N = 40;
n_U = 3;
n_X = 13; % number of nmpc states
n_Y = 9+7;
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
wn=1;
we=3;
wd=0;

wn_err=-1;
we_err=+2;
wd_err=-1;

% params
R_acpt = 30;                    % switching acceptance radius
ceta_acpt = 0.8;                % switching acceptance cosine of error angle
alpha_p_co = deg2rad(8);        % angle of attack upper cut-off
alpha_m_co = deg2rad(-3);       % angle of attack lower cut-off
alpha_delta_co = deg2rad(2);    % angle of attack cut-off transition length\
T_b_ne = 1.5;                   % lateral-directional track-error boundary constant
T_b_d = 3.5;                    % longitudinal track-error boundary constant
vG_min = 1;                     % minimum ground speed for track-error boundary

% integral action
k_i_lat = 0.1;
k_i_lon = 0.1;

% model parameters
load parameters_20161209.mat;

% initial conditions
ic_ned  = [-300, 0, 10];
ic_vV   = [14, 0, 0];
ic_att  = [0, 0];
ic_attdot = [0, 0, 0];
ic_u    = [0.375, 0, 1.7*pi/180];
ic_augm = [ic_u(1), 0];

ic_od   = [pparams, pparams_next, R_acpt, ceta_acpt, ...
    wn+wn_err, we+we_err, wd+wd_err, ...
    alpha_p_co, alpha_m_co, alpha_delta_co, ...
    T_b_ne, T_b_d, vG_min];

W_e_lat = 0;
W_e_lon = 0;

i_e_lat = 0;
i_e_lon = 0;

% acado inputs
nmpc_ic.x   = [ic_ned,ic_vV,ic_att,ic_attdot,ic_augm]; 
nmpc_ic.u   = ic_u;
yref        = [i_e_lat i_e_lon, 0 0, 14, 0 0 0, 0];
zref        = [0, ic_u, ic_u];

Q_scale     = [1 1, pi/2 pi/2, 1, 50*pi/180 50*pi/180 50*pi/180, 1];
R_scale     = [1, 1 30*pi/180 15*pi/180, 1 5*pi/180 5*pi/180];

Q_output    = [10 10, 150 100, 10, 20 20 5, 1]./Q_scale.^2;
QN_output   = [1 1, 150 100, 10, 20 20 5, 1]./Q_scale.^2;
R_controls  = [40 0.1 0.1 0.1 30 20 30]./R_scale.^2;

Q_prev      = [(linspace(1,0,N+1)').^0,...
    (linspace(1,0,N+1)').^0,...
    (linspace(1,0,N+1)').^0];

input.x     = repmat(nmpc_ic.x, N+1,1);
input.u     = repmat(nmpc_ic.u, N,1);
input.y     = repmat([yref, zref], N,1);
input.yN    = input.y(1, 1:length(yref))';
input.od    = repmat(ic_od, N+1, 1);
for i = 1:N
input.W(n_Y*(i-1)+1:n_Y*i,:) = diag([Q_output(1)*W_e_lat, Q_output(2)*W_e_lon, Q_output(3:end), R_controls(1:4), R_controls(5:7).*Q_prev(i,:)]);
end
input.WN = diag(QN_output);

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
    0,0];
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
                wn+wn_err, we+we_err, wd+wd_err, ...
                alpha_p_co, alpha_m_co, alpha_delta_co, ...
                T_b_ne, T_b_d, vG_min];
            input.od = repmat(ic_od, N+1, 1);
            X0(end) = 0; % sw
            output.x(:,end)=zeros(N+1,1);
%             i_e_lat = 0; % reset
%             i_e_lon = 0; % reset
        elseif ~endofwaypoints
            endofwaypoints=true;
            for i = 1:length(pparams)
                pparams(i) = pparams_next(i);
            end
            path_idx = path_idx + 1;
            ic_od = [pparams, pparams_next, R_acpt, ceta_acpt, ...
                wn+wn_err, we+we_err, wd+wd_err, ...
                alpha_p_co, alpha_m_co, alpha_delta_co, ...
                T_b_ne, T_b_d, vG_min];
            input.od = repmat(ic_od, N+1, 1);
            X0(end) = 0; % sw
            output.x(:,end)=zeros(N+1,1);
%             i_e_lat = 0; % reset
%             i_e_lon = 0; % reset
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
            input.u     = output.u;%[output.u(2:end,:); output.u(end,:)]; %
            input.y(:,(end-2):end) = output.u;%[output.u(1,1)*ones(N,1),output.u(:,2:3)];%repmat(output.u(1,:),N,1);%
            input.y(:,1:2) = [i_e_lat*ones(N,1), i_e_lon*ones(N,1)];
        end

        % generate controls
        output = acado_nmpc_ext_step(input);

        U0  = output.u(1,:);

        % record info
        INFO_MPC = [INFO_MPC; output.info];
        KKT_MPC = [KKT_MPC; output.info.kktValue];

        % augmented states measurement update
        X0(12:13) = output.x(1,12:13) + (output.x(2,12:13) - output.x(1,12:13))*Ts_nmpc/Ts_step;

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
    X_rec(k,:) = [simout, X0(13)];
    horiz_rec(:,k,:) = output.x(:,:);
    horiz_rec_U(:,k,:) = output.u(:,:);
    U_rec(k,:) = U0;
    ic_od_nowerr = [pparams, pparams_next, R_acpt, ceta_acpt, ...
                wn, we, wd, ...
                alpha_p_co, alpha_m_co, alpha_delta_co, ...
                T_b_ne, T_b_d, vG_min];
    J_rec(k,:) = [calculate_cost([X0,U0,ic_od_nowerr],[n_X,n_U]), i_e_lat, i_e_lon];
    
    J_meas(k,:) = [calculate_cost([X0,U0,ic_od],[n_X,n_U]), i_e_lat, i_e_lon];
    
    % track-error weighting
    i_e_lat_0 = i_e_lat;
    i_e_lon_0 = i_e_lon;
    
    e_lat1 = J_meas(k,11);
    if (abs(e_lat1) < 0.8), W_e_lat = 1;
    elseif (abs(e_lat1) < 1), W_e_lat = cos((e_lat1-0.8)/0.2*3.141592653589793) * 0.5 + 0.5;
    else
        W_e_lat = 0;
        i_e_lat_0 = 0; % reset
    end
    
    e_lon1 = J_meas(k,12);
    if (abs(e_lon1) < 0.8), W_e_lon = 1;
    elseif (abs(e_lon1) < 1), W_e_lon = cos((e_lon1-0.8)/0.2*3.141592653589793) * 0.5 + 0.5;
    else
        W_e_lon = 0;
        i_e_lon_0 = 0; % reset
    end
    
    for i = 1:N
        input.W(n_Y*(i-1)+1,1) = Q_output(1)*W_e_lat;
        input.W(n_Y*(i-1)+2,2) = Q_output(2)*W_e_lon;
    end
    input.WN(1,1) = QN_output(1)*W_e_lat;
    input.WN(2,2) = QN_output(2)*W_e_lon;
    
    i_e_lat = i_e_lat_0 - k_i_lat * W_e_lat * e_lat1 * Ts_nmpc;
    i_e_lon = i_e_lon_0 - k_i_lon * W_e_lon * e_lon1 * Ts_nmpc;
    
%     % integral action
%     i_e_lat_0 = i_e_lat;
%     i_e_lon_0 = i_e_lon;

%     e_ne1 = J_rec(k,11);
%     if (abs(e_ne1) < 0.8), W_i_ne = 1;
%     elseif (abs(e_ne1) < 1), W_i_ne = cos((e_ne1-0.8)/0.2*3.141592653589793) * 0.5 + 0.5;
%     else
%         W_i_ne = 0;
%         i_e_lat_0 = 0; % reset
%     end
%     
%     e_d1 = J_rec(k,12);
%     if (abs(e_d1) < 0.8), W_i_d = 1;
%     elseif (abs(e_d1) < 1), W_i_d = cos((e_d1-0.8)/0.2*3.141592653589793) * 0.5 + 0.5;
%     else
%         W_i_d = 0;
%         i_e_lon_0 = 0; % reset
%     end
%     
%     i_e_lat = i_e_lat_0 - k_i_lat * W_i_ne * max(min(e_ne1,1),-1) * Ts_nmpc;
%     i_e_lon = 0;%i_e_lon_0 + k_i_lon * W_i_d * max(min(e_d1,1),-1) * Ts_nmpc;

end

plotsim