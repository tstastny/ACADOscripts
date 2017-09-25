% Techpod NMPC simulation
% -----------------------
close all; clear all; clc; clear mex;

% load aircraft parameters
load parameters_2015.09.10_1956_6DoF.mat
for i = 1:length(parameters)
    eval([parameters(i).Name,' = ',num2str(parameters(i).Value),';'])
end

% initial conditions
N       = 20;
n_U     = 5;

% L1 guidance
L1p_lon     = 10;
L1d_lon     = 0.1;
kilon       = 0.01;
intg_lon    = 0;
theta_cmd_min = -15*pi/180;
theta_cmd_max = 15*pi/180;

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
Aw2 = -8;
Bw2 = 1;
Cw2 = -127;
Dw2 = 16;

% wind
wn=0;
we=0;
wd=0;

load initXU;
% initXh = [initXh(:,1:8), zeros(N+1,1), ...
%     zeros(N+1,1)+linspace(0,0.05*V_cmd*N,N+1)', zeros(N+1,2), ...
%     zeros(N+1,3), zeros(N+1,5)];
% input.x = initXh;
% X0      = initXh(1,:);
% input.u = initUh;
% U0      = initUh(1,:);
initXh = [repmat(initXh(end,1:8),N+1,1), zeros(N+1,1), ...
    zeros(N+1,1)+linspace(0,0.05*V_cmd*N,N+1)', zeros(N+1,1), 0*ones(N+1,1), ...
    zeros(N+1,3), zeros(N+1,5)];
input.x = initXh;
X0      = initXh(1,:);
input.u = repmat(initUh(end,:),N,1);
U0      = initUh(1,:);

yref        = zeros(1,7);
input.y     = [repmat(yref,N,1), zeros(N,n_U)];
input.yN    = input.y(1,1:7)';

Q_output    = [50 500 250 1500 1 10 1];
R_controls  = [1 1 1 1, 2000];

input.W     = diag([Q_output, R_controls]);
input.WN    = diag(Q_output);

% simulation
T0      = 0;
Tf      = 120;
Ts      = 0.05;
time    = T0:Ts:Tf;
KKT_MPC = []; INFO_MPC = []; controls_MPC = [];

% initial simout
simout      = [13, 0, 0.0805, zeros(1,3), 0, 0.0805, 0, 0, 0, 0];
% d_states    = [zeros(1,9), V_cmd, 0, 0];
pth_idx     = 1;

pth_Racpt = 20;
pth_Aacpt = 30*pi/180;
in_transition = false;
transition_count = 0;

for k = 1:length(time)

    %  - - - - - START NMPC - - - - -
    
    st_nmpc = tic;
    
    % measure
    input.x0    = X0';
    
    % guidance
    if paths(pth_idx).pparam1 == 0
        
        % calculate vector from waypoint a to b
        v_bb = [paths(pth_idx).pparam5, paths(pth_idx).pparam6, paths(pth_idx).pparam7] - [paths(pth_idx).pparam2, paths(pth_idx).pparam3, paths(pth_idx).pparam4];
        normv_bb = norm(v_bb);

        % calculate closest point on line a->b
        aa_pp       = X0(10:12) - [paths(pth_idx).pparam2, paths(pth_idx).pparam3, paths(pth_idx).pparam4];

        % calculate projected 1-D distance along track
        p_track    = dot(v_bb, aa_pp) / normv_bb;
        
        if (p_track > normv_bb - pth_Racpt && ...
                norm(cross(v_bb, d_states(10:12))) / normv_bb / norm(d_states(10:12)) < pth_Aacpt && ...
                pth_idx<length(paths)) || in_transition
            
%             if ~in_transition
%                 in_transition = true;
%                 transition_count = 1;
%             else
%                 transition_count = transition_count + 1;
%             end
            
%             if transition_count > N
                pth_idx = pth_idx + 1;
%                 transition_count = 0;
%                 in_transition = false;
%             end
        end
        
    elseif paths(pth_idx).pparam1 == 1
        
        bb = [paths(pth_idx).pparam2, paths(pth_idx).pparam3, paths(pth_idx).pparam4] + paths(pth_idx).pparam5 * ...
            [cos(paths(pth_idx).pparam8 + paths(pth_idx).pparam6 * paths(pth_idx).pparam9), ...
            sin(paths(pth_idx).pparam8 + paths(pth_idx).pparam6 * paths(pth_idx).pparam9), ...
            -tan(paths(pth_idx).pparam7) * paths(pth_idx).pparam9];
        
        xiend = paths(pth_idx).pparam8 + paths(pth_idx).pparam6 * (paths(pth_idx).pparam9 + pi/2);
        
        v_bb = [cos(xiend), sin(xiend), -sin(paths(pth_idx).pparam7)];
        
        if (norm(bb - X0(10:12)) < pth_Racpt && ...
                norm(cross(v_bb, d_states(10:12))) / norm(v_bb) / norm(d_states(10:12)) < pth_Aacpt && ...
                pth_idx<length(paths)) || in_transition
            
%             if ~in_transition
%                 in_transition = true;
%                 transition_count = 1;
%             else
%                 transition_count = transition_count + 1;
%             end
            
%             if transition_count > N
                pth_idx = pth_idx + 1;
%                 transition_count = 0;
%                 in_transition = false;
%             end
        end
        
    end
    
%     if in_transition
%         pathparams = [repmat([paths(pth_idx).pparam1, paths(pth_idx).pparam2, paths(pth_idx).pparam3, paths(pth_idx).pparam4, paths(pth_idx).pparam5, paths(pth_idx).pparam6, paths(pth_idx).pparam7, paths(pth_idx).pparam8, paths(pth_idx).pparam9],...
%             N+1-transition_count,1);
%             repmat([paths(pth_idx+1).pparam1, paths(pth_idx+1).pparam2, paths(pth_idx+1).pparam3, paths(pth_idx+1).pparam4, paths(pth_idx+1).pparam5, paths(pth_idx+1).pparam6, paths(pth_idx+1).pparam7, paths(pth_idx+1).pparam8, paths(pth_idx+1).pparam9],...
%             transition_count,1)];
%     else
        pathparams = repmat([paths(pth_idx).pparam1, paths(pth_idx).pparam2, paths(pth_idx).pparam3, paths(pth_idx).pparam4, paths(pth_idx).pparam5, paths(pth_idx).pparam6, paths(pth_idx).pparam7, paths(pth_idx).pparam8, paths(pth_idx).pparam9],N+1,1);
%     end
    
    input.od    = [repmat([L1p_lon, L1d_lon, L1p_lat, L1d_lat, ...
        kilon, kilat, kiV, V_cmd],N+1,1), ...
        pathparams, ...
        repmat([Aw2, Bw2, Cw2, Dw2, wn, we, wd],N+1,1)];

    yref        = zeros(1,7); 
    input.y     = [repmat(yref,N,1), zeros(N,5)];
    input.yN    = input.y(end,1:7)';
    
    % generate controls
    output      = acado_nmpc_ext_step(input);
    U0          = output.u(1,:);
    
    % record info
    INFO_MPC    = [INFO_MPC; output.info];
    KKT_MPC     = [KKT_MPC; output.info.kktValue];
    
    % record states/controls
    X_rec(k,:)      = simout;
    XX_rec(k,:)     = output.x(1,13:end);
    Y_rec(k,:)      = input.y(1,:);
    U_rec(k,:)      = U0;
    
    % shift initial states and controls
    input.x     = [output.x(2:N+1,:); output.x(N+1,:)];
    input.u     = [output.u(2:N,:); output.u(N,:)];

    tsolve(k) = toc(st_nmpc);
    
    % - - - - - END NMPC - - - - -
    
    % apply control
    [d_states, simout]  = techpod_nonlin_model_12DoF(time(1), X0(1:12), U0(1:4), ...
        cD0, cDa, cDa2, ...
        cL0, cLa, cLa2, cLa3, ...
        cm0, cma, cmq, cmde, ...
        cT1, ...
        clb, clp, clr, clda, ...
        cYb, ...
        cnb, cnp, cnr, cndr, ...
        wn, we, wd);

    X0(1:12) = X0(1:12) + d_states(1:12)*Ts;
    X0(13:end) = output.x(2,13:end);
    
end

plotsim
