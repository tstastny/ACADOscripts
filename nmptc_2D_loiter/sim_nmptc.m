% NMPTC Simulation
% ----------------
close all; clear all; clc;

% initial conditions
N = 80;
n_U = 1;
n_X = 5;
n_Y = 5;
n_Z = 2;

% online data
V = 14;                   % (airspeed)                    [m/s]
c_n = 0;                 % (easting loiter center)       [m]        
c_e = 0;                 % (northing loiter center)      [m]
R = 30;                   % (loiter radius)               [m]
ldir = -1;                % (loiter direction)            [~]
w_n = 0;                 % (northing wind)               [m/s]
w_e = -10;                 % (easting wind)                [m/s]

% initial conditions
n0 = 20;
e0 = 20;
mu0 = 0;
xi0 = 0;
mu_dot0 = 0;
mu_r0 = 0;
ic_x = [n0, e0, mu0, xi0, mu_dot0];
ic_u = mu_r0;
ic_od = [V, c_n, c_e, R, ldir, w_n, w_e];

% acado inputs
nmpc_ic.x   = ic_x; 
nmpc_ic.u   = ic_u;
yref        = zeros(1,n_Y);
zref        = zeros(1,n_Z);
% y = [e_t; e_vn; e_ve; mu; mu_dot]; z = [mu_r; mu_r];
Q           = [0.1 10 10 0 0.01 0 500];
QN          = [0.1 10 10 0 0.01];
Q_delta     = (linspace(0,1,N+1)'-ones(N+1,1)).^2;

input.x     = repmat(nmpc_ic.x, N+1,1);
input.u     = repmat(nmpc_ic.u, N,1);
input.y     = repmat([yref, zref], N,1);
input.yN    = input.y(1, 1:length(yref)); %%%FURIERI: I removed the transpose operator " ' "
input.od    = repmat(ic_od, N+1, 1);
for i = 1:N
input.W((n_Y+n_Z)*(i-1)+1:(n_Y+n_Z)*i,:) = diag([Q(1:end-1), Q(end)*Q_delta(i,:)]);
end
input.WN    = diag(QN);

% simulation
T0      = 0;
Tf      = 30;
Ts      = 0.01;
time    = T0:Ts:Tf;
KKT_MPC = []; INFO_MPC = []; controls_MPC = [];

Ts_nmpc = 0.05; % interval between nmpc calls
Ts_step = 0.1; % step size in nmpc

% initial simout
X0 = nmpc_ic.x;
simout = nmpc_ic.x;
states = simout;
d_states = [...
    V * cos(xi0) + w_n, ...
    V * sin(xi0) + w_e, ...
    zeros(1,length(ic_x)-2)];
U0 = 0;
for k = 1:length(time)
   
    % measure
    input.x0 = X0;   %%%FURIERI: I removed the transpose operator " ' "
    
    if time(k)==floor(time(k)/Ts_nmpc)*Ts_nmpc
    
    %  - - - - - START NMPC - - - - -
    st_nmpc = tic;
        
     % shift initial states and controls
    if k > 1
        input.x = output.x;%[output.x(2:end,:); output.x(end,:)]; %
%         input.x = spline(0:Ts_step:(N*Ts_step),output.x',Ts_nmpc:Ts_step:(N*Ts_step+Ts_nmpc))';
        input.u = output.u;%[output.u(2:end,:); output.u(end,:)]; %
%         input.u = spline(0:Ts_step:((N-1)*Ts_step),output.u',Ts_nmpc:Ts_step:((N-1)*Ts_step+Ts_nmpc))';
        input.y(:,end) = [input.u(2:end,:); input.u(end,:)];
    end
    
    % generate controls
    output = acado_nmpc_step(input);
        
    U0 = output.u(2,:);
%     U0 = [spline(0:Ts_step:((N-1)*Ts_step),output.u(:,1),Ts_nmpc), ...
%           spline(0:Ts_step:((N-1)*Ts_step),output.u(:,2),Ts_nmpc)];
    if U0(1) > 35*pi/180, U0(1) = 35*pi/180; end;
    if U0(1) < -35*pi/180, U0(1) = -35*pi/180; end;
    
    % record info
    INFO_MPC    = [INFO_MPC; output.info];
    KKT_MPC     = [KKT_MPC; output.info.kktValue];
    
%     % shift initial states and controls
%     input.x     = [output.x(2:N+1,:); output.x(N+1,:)];
%     input.u     = [output.u(2:N,:); output.u(N,:)];

    tsolve(k) = toc(st_nmpc);
    
    end
    % - - - - - END NMPC - - - - -
    
    % record states/controls
    X_rec(k,:)  = simout;
    Horiz_n_rec(k,:) = output.x(:,1)';
    Horiz_e_rec(k,:) = output.x(:,2)';
    Horiz_mu_rec(k,:) = output.x(:,3)';
    Horiz_mu_r_rec(k,:) = output.u(:,1)';
    U_rec(k,:) 	= U0;
    
    % apply control
    [d_states, simout]  = uav3DoF(time(k), states, U0, w_n, w_e, V);

    % integration (model propagation)
    states = states + d_states*Ts;
    X0 = simout;
    
end

plotsim