% NMPTC Simulation
% ----------------
close all; clear all; clc;

% initial conditions
N = 30;
n_U = 2;
n_X = 13;
n_XA = 5;
n_Y = 14;
n_Z = 0;

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
wn=5;
we=3;
wd=1;

% parameters
omega_n_mu_nmpc = 15;
zeta_mu_nmpc = 0.9;
omega_n_gamma_nmpc = 6;
zeta_gamma_nmpc = 0.7;

dyn_nmpc = [omega_n_mu_nmpc, zeta_mu_nmpc, omega_n_gamma_nmpc, zeta_gamma_nmpc];

omega_n_mu = 15;
zeta_mu = 0.9;
omega_n_gamma = 6;
zeta_gamma = 0.7;

dyn = [omega_n_mu, zeta_mu, omega_n_gamma, zeta_gamma];

aw2 = -10;
bw2 = 1;
cw2 = -100;
dw2 = 10;

% initial conditions
ic_ned  = [0, 30, 0];
ic_V    = 14;
ic_att  = [0, 0, -pi/1.05];
ic_attdot = [0, 0];
ic_u    = [0,0];
ic_augm = [0,0,0,0,0];

ic_od   = [ic_V, pparams, pparams_next, wn, we, wd, dyn_nmpc, aw2, bw2, cw2, dw2];

% pparams00 = [0, ...
%     ic_ned(1)+10, ...
%     ic_ned(2)+10, ...
%     ic_ned(3), ...
%     1000*cos(ic_att(3))+ic_ned(1)+10, ...
%     1000*sin(ic_att(3))+ic_ned(2)+10, ...
%     ic_ned(3), ...
%     0, ...
%     0];
% ic_od00   = [ic_V, pparams00, pparams, wn, we, wd, dyn, aw2, bw2, cw2, dw2];


% acado inputs
nmpc_ic.x   = [ic_ned,ic_att,ic_attdot,ic_augm]; 
nmpc_ic.u   = ic_u;
yref        = zeros(1,n_Y);
zref        = zeros(1,n_Z);
% y   = [ mu; gamma; mu_dot; gamma_dot; et; e_Gamma; e_chi; intg_et; intg_e_Gamma; intg_e_chi; mu_r; gamma_r; z_w2_mu; z_w2_gamma ];
Q_output    = [0 0 0 0 0.07 5 5 0.001 0 0 0.1 0.5 1 1];
QN_output   = [0 0 0 0 0.07 5 5 0.001 0 0 0.1 0.5 1 1];
R_controls  = [];


input.x     = repmat(nmpc_ic.x, N+1,1);
input.u     = repmat(nmpc_ic.u, N,1);
input.y     = repmat([yref, zref], N,1);
input.yN    = input.y(1, 1:length(yref))';
input.od    = repmat(ic_od, N+1, 1);
input.W     = diag([Q_output, R_controls]);
input.WN    = diag(QN_output);

% simulation
T0      = 0;
Tf      = 15;
Ts      = 0.01;
time    = T0:Ts:Tf;
KKT_MPC = []; INFO_MPC = []; controls_MPC = [];

Ts_nmpc = 0.05; % interval between nmpc calls
Ts_step = 0.1; % step size in nmpc

% initial simout
X0          = nmpc_ic.x;
simout      = nmpc_ic.x(1:(n_X-n_XA));
states      = simout;
d_states    = [...
    ic_V*cos(ic_u(2))*cos(ic_u(1))+wn, ...
    ic_V*cos(ic_u(2))*sin(ic_u(1))+we, ...
    -ic_V*cos(ic_u(2))+wd, ...
    zeros(1,5)];
cost = 0;

for k = 1:length(time)
    
%     if time(k) < 5
%         input.od = repmat(ic_od00, N+1, 1);
%     else
%         input.od = repmat(ic_od, N+1, 1);
%     end
    % measure
    input.x0    = X0';
    
    if time(k)==floor(time(k)/Ts_nmpc)*Ts_nmpc
    
    %  - - - - - START NMPC - - - - -
    st_nmpc = tic;
        
     % shift initial states and controls
    if k > 1
%         input.x     = [output.x(2:end,:); output.x(end,:)];
        input.x =spline(0:Ts_step:(N*Ts_step),output.x',Ts_nmpc:Ts_step:(N*Ts_step+Ts_nmpc))';
%         input.u     = [output.u(2:end,:); output.u(end,:)];
        input.u = spline(0:Ts_step:((N-1)*Ts_step),output.u',Ts_nmpc:Ts_step:((N-1)*Ts_step+Ts_nmpc))';
        
    end
    
    % generate controls
    output      = acado_nmpc_ext_step(input);
%     U0          = output.u(2,:);
    U0 = [spline(0:Ts_step:((N-1)*Ts_step),output.u(:,1),Ts_nmpc), ...
          spline(0:Ts_step:((N-1)*Ts_step),output.u(:,2),Ts_nmpc)];
    if U0(1) > 35*pi/180, U0(1) = 35*pi/180; end;
    if U0(1) < -35*pi/180, U0(1) = -35*pi/180; end;
    if U0(2) > 15*pi/180, U0(2) = 15*pi/180; end;
    if U0(2) < -15*pi/180, U0(2) = -15*pi/180; end;
    X0((n_X-n_XA+1):n_X) = output.x(2,(n_X-n_XA+1):n_X);
%     if ~isempty((n_X-n_XA+1):n_X)
%         X0((n_X-n_XA+1):n_X) = spline(0:Ts_step:(N*Ts_step),output.x(:,(n_X-n_XA+1):n_X)',Ts_nmpc);
%     end
    
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
    Horiz_n_rec(k,:) = output.x(:,1)';
    Horiz_e_rec(k,:) = output.x(:,2)';
    Horiz_d_rec(k,:) = output.x(:,3)';
    Horiz_mu_rec(k,:) = output.x(:,4)';
    Horiz_gamma_rec(k,:) = output.x(:,5)';
    Horiz_mu_r_rec(k,:) = output.u(:,1)';
    Horiz_gamma_r_rec(k,:) = output.u(:,2)';
    XA_rec(k,:) = X0((n_X-n_XA+1):n_X);
%     Y_rec(k,:)	= [Va_cmd, theta_cmd, phi_cmd, intg_lon, intg_lat, etalon, etalat];
    J_rec(k,:)  = cost;
    U_rec(k,:) 	= U0;
    
    % apply control
    [d_states, simout, cost]  = uav3DoF(time(k), states, U0, wn, we, wd, ic_V, dyn, pparams);

    % integration (model propagation)
    states = states + d_states*Ts;
    X0(1:(n_X-n_XA)) = simout;
    
end

plotsim
