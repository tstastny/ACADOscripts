% Techpod NMPC simulation
% -----------------------
close all; clear all; clc;

% load aircraft parameters
% load parameters_2016.03.09_1741.mat
load parameters_2016.03.10_1254.mat;
for i = 1:length(parameters)
    eval([parameters(i).Name,' = ',num2str(parameters(i).Value),';'])
end

% initial conditions
N       = 20;
n_U     = 5;

% other guidance
V_cmd       = 14;
gamma_cmd = 0;
intg_V      = 0;
ki_V         = 0.1;
ki_gam       = 0.1;
intg_gam    = 0;

% weighting matrices
% Aw2 = 0;
% Bw2 = 0;
% Cw2 = 0;
% Dw2 = 1;
Aw2 = -8;
Bw2 = 1;
Cw2 = -127;
Dw2 = 16;

% wind
wn=0;
we=0;
wd=0;

% states
n = 0;
e = 0;
d = 0;

% acado inputs
vba_IC      = [14.0518073499692,0,0.0270024970840995];
pqr_IC      = [0,8.77619910557108e-06,0]; %[250,250,250]*pi/180;
phth_IC     = [0,0.0269160032018684]; %[.6,0.4];
uTdum_IC    = [0.500360707220786,0];
intg_IC     = [0,-0.00135591591126331];
x_w2_IC     = [0.0566259641462802,-0.000748298294471934,0,0];

nmpc_ic.x   = [vba_IC,pqr_IC,phth_IC,uTdum_IC,intg_IC,x_w2_IC]; 
nmpc_ic.u   = [0.3913, -0.0064, 0, 0, 0];
yref        = [0, 0, 0, 0, 0, 0, 0]; 
Q_output    = [5 150 50 500 0.1 5 0.1];
R_controls  = [.1 1 .1 .1 100];

input.x     = repmat(nmpc_ic.x,N+1,1);
input.u     = repmat(nmpc_ic.u,N,1);
input.y     = [repmat(yref,N,1), zeros(N,5)];
input.yN    = input.y(1,1:length(yref))';
input.od    = repmat([Aw2, Bw2, Cw2, Dw2, ki_V, ki_gam, V_cmd, gamma_cmd],N+1,1);
input.W     = diag([Q_output, R_controls]);
input.WN    = diag(Q_output);

% simulation
T0      = 0;
Tf      = 15;
Ts      = 0.01;
time    = T0:Ts:Tf;
KKT_MPC = []; INFO_MPC = []; controls_MPC = [];

% initial simout
X0          = nmpc_ic.x;
simout      = [nmpc_ic.x(1:8), 0, n, e, d, nmpc_ic.x(9)];
states      = simout(1:13);
d_states    = [zeros(1,9), 13, 0, 0, 0];
pth_idx     = 1;

Ts_nmpc = 0.05; 
Ts_step = 0.05;

for k = 1:length(time)
    
    % measure
    input.x0    = X0';
    
    yref        = [0, 0, 0, 0, 0, 0, 0]; 
    input.y     = [repmat(yref,N,1), zeros(N,5)];
    input.yN    = input.y(end,1:7)';
    
    if time(k)==floor(time(k)/Ts_nmpc)*Ts_nmpc
    
    %  - - - - - START NMPC - - - - -
    st_nmpc = tic;
        
     % shift initial states and controls
    if k > 1
    input.x     = spline(0:Ts_step:(N*Ts_step),output.x',Ts_nmpc:Ts_step:(N*Ts_step+Ts_nmpc))';
    input.u     = spline(0:Ts_step:((N-1)*Ts_step),output.u',Ts_nmpc:Ts_step:((N-1)*Ts_step+Ts_nmpc))';
    X0(10:16)   = output.x(2,10:16);
    end
    
    % generate controls
    output      = acado_nmpc_step(input);
    U0          = output.u(1,:);
    
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
    X_rec(k,:)       = [simout X0([11,12])];
    X_w2_rec(k,:)    = X0(10:end);
    Y_rec(k,:)       = [V_cmd, 0, 0, 0, 0, asin(-d_states(12)/states(1))];
    U_rec(k,:)       = U0;
    
    % apply control
    [d_states, simout]  = techpod_nonlin_model_13DoF(time(k), states, U0(1:4), ...
        cD0, cDa, cDa2, ...
        cL0, cLa, cLa2, cLa3, ...
        cm0, cma, cmq, cmde, ...
        cT0, cT1, cT2, tauT, ...
        clb, clp, clr, clda, ...
        cYb, ...
        cnb, cnp, cnr, cndr, ...
        wn, we, wd);

    % integration (model propagation)
    states = states + d_states*Ts;
    X0(1:9) = simout([1:8,13]);
    
end

plotsim_ICS
