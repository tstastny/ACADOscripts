% Techpod NMPC simulation
% -----------------------
close all; clear all; clc;

% load aircraft parameters
load parameters_2015.09.10_1956_6DoF.mat
for i = 1:length(parameters)
    eval([parameters(i).Name,' = ',num2str(parameters(i).Value),';'])
end

% initial conditions
N       = 10;
n_U     = 5;

% L1 guidance
L1p_lon     = 10;
L1d_lon     = 0.1;
kilon       = 0.01;
intg_lon    = 0;%-8.3e-3;
theta_cmd_min = -25*pi/180;
theta_cmd_max = 25*pi/180;

L1p_lat     = 10;
L1d_lat     = 0.707;
kilat       = 0;
intg_lat    = 0;
phi_cmd_min = -45*pi/180;
phi_cmd_max = 45*pi/180;

% other guidance
V_cmd       = 13;
intg_V      = 0;
kiV         = 0;

defpaths

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
wn=0;%1.5;
we=0;%-2;
wd=0;

% states
n = 0;
e = 0;
d = 0;
pp = [n,e,d];

% acado inputs
input.od    = repmat([Aw2, Bw2, Cw2, Dw2, kiV],N+1,1);

% X0          = ...
%     [13*cos(0.0805), 0, 13*sin(0.0805), ... % body velocities
%      zeros(1,3), ...                        % body angular rates
%      0, 0.0805, 0, ...                      % euler angles
%      0, ...                                 % slack dummy
%      intg_V, intg_gam, ...                  % integrators
%      zeros(1,4)];                           % weighting matrix states
 
% input.x  	= repmat(X0,N+1,1);

% U0          = [0.39, -0.0189, 0, 0, 0];
% input.u     = repmat(U0,N,1);

load initXU;
initXh = [initXh(:,1:8), zeros(N+1,7)];
input.x = initXh;%[initXh, zeros(N+1,1)];%; repmat(initXh(end,:),0,1)];
X0      = initXh(1,:);
input.u = [initUh; repmat(initUh(end,:),0,1)];

yref        = [V_cmd, 0, 0, 0, 0, 0, 0]; %zeros(1,4);%
input.y     = [repmat(yref,N,1), zeros(N,n_U)];
% input.y     = [repmat(yref,N,1), repmat(U0,N,1)];
input.yN    = input.y(1,1:7)';

Q_output    = [50 500 250 1500 1 10 1];
R_controls  = [1 1 1 1, 2000];

input.W     = diag([Q_output, R_controls]);
input.WN    = diag(Q_output);
% input.W = diag([0.01 5 0.1 100 0.001 1 1 1 0.1]);
% input.WN = diag([0.01 5 0.1 100]);

% simulation
T0      = 0;
Tf      = 50;
Ts      = 0.05;
time    = T0:Ts:Tf;
KKT_MPC = []; INFO_MPC = []; controls_MPC = [];

% initial simout
simout      = [13, 0, 0.0805, zeros(1,3), 0, 0.0805, 0, n, e, d];
psi = simout(9);
d_states    = [zeros(1,9), 13, 0, 0];
pth_idx     = 1;
passed_xi0  = false;

tg0 = 30;
tg01 = 0.8;
tg12 = 0;
tg23 = 0.3;
wdmax = -5;
wdmin = wd;

for k = 1:length(time)
    
%     if time(k) < tg0
%         wd = wdmin;
%     elseif time(k) < tg0+tg01
%         wd = (wdmax-wdmin)/2*(1-cos(pi*(time(k)-tg0)/tg01))+wdmin;
%     elseif time(k) < tg0+tg01+tg12
%         wd = wdmax;
%     elseif time(k) < tg0+tg01+tg12+tg23
%         wd = -(wdmax-wdmin)/2*(1-cos(pi*(time(k)-(tg0+tg01+tg12))/tg23))+wdmax;
%     else
%         wd = wdmin;
%     end
%     input.od    = repmat([Aw2, Bw2, Cw2, Dw2, wn, we, wd, kiV],N+1,1);

    %  - - - - - START NMPC - - - - -
    
    st_nmpc = tic;
    
    % measure
    input.x0    = X0';
    
    % guidance        
    
    if paths(pth_idx).type == 0
        
        [theta_cmd, phi_cmd, etalon, etalat, p_travel] = ...
            L1guide_line(simout(10:12), d_states(10:12), ...
            paths(pth_idx).aa, paths(pth_idx).bb, ...
            L1p_lon, L1d_lon, intg_lon, ...
            L1p_lat, L1d_lat, intg_lat, ...
            theta_cmd_min, theta_cmd_max, phi_cmd_min, phi_cmd_max);
        
        if p_travel > norm(paths(pth_idx).bb - paths(pth_idx).aa) - 10 && pth_idx<length(paths)
            pth_idx = pth_idx + 1;
        end
        
    elseif paths(pth_idx).type == 1
        
        [theta_cmd, phi_cmd, etalon, etalat, p_travel] = ...
            L1guide_spiral(simout(10:12), d_states(10:12), ...
            paths(pth_idx).cc, paths(pth_idx).R, paths(pth_idx).ldir, paths(pth_idx).xi0, paths(pth_idx).gam, ...
            L1p_lon, L1d_lon, intg_lon, ...
            L1p_lat, L1d_lat, intg_lat,...
            theta_cmd_min, theta_cmd_max, phi_cmd_min, phi_cmd_max);
        
        if ~passed_xi0
            xik = atan2(simout(11) - paths(pth_idx).cc(2), simout(10) - paths(pth_idx).cc(1));
            deltaxik = xik - paths(pth_idx).xi0;
            if paths(pth_idx).ldir>0 && paths(pth_idx).xi0>xik
                deltaxik = deltaxik + 2*pi;
            elseif paths(pth_idx).ldir<0 && xik>paths(pth_idx).xi0
                deltaxik = deltaxik - 2*pi;
            end
            if deltaxik > pi, deltaxik = deltaxik - 2*pi; end;
            if deltaxik < -pi, deltaxik = deltaxik + 2*pi; end;
            passed_xi0 = paths(pth_idx).ldir * deltaxik >= 0;
        end
        if paths(pth_idx).ldir * p_travel > paths(pth_idx).deltaxi - 10*pi/180 && passed_xi0 && pth_idx<length(paths)
            pth_idx = pth_idx + 1;
            passed_xi0 = false;
        end
        
    else
        
        theta_cmd = 0; phi_cmd = 0; etalon = 0; etalat = 0;
        
    end
    
    
    intg_lon = intg_lon + kilon * etalon * Ts;
    intg_lat = intg_lat + kilat * etalat * Ts;
    
%     if time(k) > 10 && time(k) < 30
%         if V_cmd > 10.5
%             V_cmd = V_cmd - 0.05;
%         end
%     elseif time(k) <  50
%         if V_cmd < 13
%             V_cmd = V_cmd + 0.05;
%         end
%     end
    yref        = [V_cmd, theta_cmd, phi_cmd, 0, 0, 0, 0]; 
%     input.y     = [repmat(yref,N,1), input.u]; //NO. this doesnt make
%     sense with z's
    input.y     = [repmat(yref,N,1), zeros(N,5)];
    input.yN    = input.y(end,1:7)';
    
    % generate controls
    output      = acado_nmpc_step(input);
    U0          = output.u(1,:);
    
    % record info
    INFO_MPC    = [INFO_MPC; output.info];
    KKT_MPC     = [KKT_MPC; output.info.kktValue];
    
    % record states/controls
    X_rec(k,:)       = [simout, X0(10)];
    Y_rec(k,:)       = [V_cmd, theta_cmd, phi_cmd, intg_lon, intg_lat, etalon, etalat];
    U_rec(k,:)       = U0;
    
    % shift initial states and controls
    input.x     = [output.x(2:N+1,:); output.x(N+1,:)];
    input.u     = [output.u(2:N,:); output.u(N,:)];
    X0(9:15)   = output.x(2,9:15);

    tsolve(k) = toc(st_nmpc);
    
    % - - - - - END NMPC - - - - -
    
    % apply control
    [d_states, simout]  = techpod_nonlin_model_12DoF(time(1), [X0(1:8), psi, pp], U0(1:4), ...
        cD0, cDa, cDa2, ...
        cL0, cLa, cLa2, cLa3, ...
        cm0, cma, cmq, cmde, ...
        cT1, ...
        clb, clp, clr, clda, ...
        cYb, ...
        cnb, cnp, cnr, cndr, ...
        wn, we, wd);

    X0(1:8) = X0(1:8) + d_states(1:8)*Ts;
    psi = psi + d_states(9)*Ts;
    pp = pp + d_states(10:12)*Ts;
    
end

plotsim
