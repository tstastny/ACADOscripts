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

% L1 guidance
L1p_lon     = 10;
L1d_lon     = 0.1;
kilon       = 0.01;
intg_lon    = 0;
theta_cmd_min = -25*pi/180;
theta_cmd_max = 25*pi/180;

L1p_lat     = 10;
L1d_lat     = 0.707;
kilat       = 0;
intg_lat    = 0;
phi_cmd_min = -45*pi/180;
phi_cmd_max = 45*pi/180;

% other guidance
V_cmd       = 14;
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
wn=0;
we=0;
wd=0;

% states
n = 0;
e = 0;
d = 0;

% acado inputs
nmpc_ic.x   = [14,0,0.0150406852113097,0,0.00112015092434004,0,0,0.0150406852113097,0.393056460359436,0,0,zeros(1,4)]; 
nmpc_ic.u   = [0.3913, -0.0064, 0, 0, 0];
yref        = [V_cmd, 0, 0, 0, 0, 0, 0]; 
Q_output    = [5 50 50 500 0.1 5 0.1];
R_controls  = [.1 1 .1 .1 100];

input.x     = repmat(nmpc_ic.x,N+1,1);
input.u     = repmat(nmpc_ic.u,N,1);
input.y     = [repmat(yref,N,1), zeros(N,5)];
input.yN    = input.y(1,1:length(yref))';
input.od    = repmat([Aw2, Bw2, Cw2, Dw2, kiV],N+1,1);
input.W     = diag([Q_output, R_controls]);
input.WN    = diag(Q_output);

% simulation
T0      = 0;
Tf      = 60;
Ts      = 0.01;
time    = T0:Ts:Tf;
KKT_MPC = []; INFO_MPC = []; controls_MPC = [];

% initial simout
X0          = nmpc_ic.x;
simout      = [nmpc_ic.x(1:8), 0, n, e, d, nmpc_ic.x(9)];
states      = simout(1:13);
d_states    = [zeros(1,9), 13, 0, 0, 0];
pth_idx     = 1;

pth_Racpt = 20;
pth_Aacpt = 30*pi/180;
in_trans = false;
seg_term = false;
trans_count = 0;
N_trans = 1;

Ts_L1 = 0.05;
Ts_nmpc = 0.02; 
Ts_step = 0.05;

for k = 1:length(time)
    
    
%     if time(k) > 15 %&& V_cmd > 5
%         
%         V_cmd = 5;%V_cmd - 8/20/5;
%         
%     end
    
    
    
    % measure
    input.x0    = X0';
    
    % guidance        
            
    if ~in_trans && time(k)==floor(time(k)/Ts_L1)*Ts_L1

        if paths(pth_idx).pparam1 == 0

            % calculate vector from waypoint a to b
            v_bb = [paths(pth_idx).pparam5, paths(pth_idx).pparam6, paths(pth_idx).pparam7] - [paths(pth_idx).pparam2, paths(pth_idx).pparam3, paths(pth_idx).pparam4];
            normv_bb = norm(v_bb);

            % calculate closest point on line a->b
            aa_pp       = simout(10:12) - [paths(pth_idx).pparam2, paths(pth_idx).pparam3, paths(pth_idx).pparam4];

            % calculate projected 1-D distance along track
            p_track    = dot(v_bb, aa_pp) / normv_bb;

            seg_term = p_track > normv_bb - pth_Racpt && ...
                norm(cross(v_bb, d_states(10:12))) / normv_bb / norm(d_states(10:12)) < pth_Aacpt && ...
                pth_idx<length(paths);

        elseif paths(pth_idx).pparam1 == 1

            bb = [paths(pth_idx).pparam2, paths(pth_idx).pparam3, paths(pth_idx).pparam4] + paths(pth_idx).pparam5 * ...
                [cos(paths(pth_idx).pparam8 + paths(pth_idx).pparam6 * paths(pth_idx).pparam9), ...
                sin(paths(pth_idx).pparam8 + paths(pth_idx).pparam6 * paths(pth_idx).pparam9), ...
                -tan(paths(pth_idx).pparam7) * paths(pth_idx).pparam9];
        
            xiend = paths(pth_idx).pparam8 + paths(pth_idx).pparam6 * (paths(pth_idx).pparam9 + pi/2);
        
            v_bb = [cos(xiend), sin(xiend), -sin(paths(pth_idx).pparam7)];
        
            seg_term = norm(bb - simout(10:12)) < pth_Racpt && ...
                norm(cross(v_bb, d_states(10:12))) / norm(v_bb) / norm(d_states(10:12)) < pth_Aacpt && ...
                pth_idx<length(paths);
            
        end

    end

    if (seg_term || in_trans) && time(k)==floor(time(k)/Ts_L1)*Ts_L1

        if ~in_trans
            in_trans = true;
            trans_count = 1;
        else
            trans_count = trans_count + 1;
        end

        % path current ----------------------------------------------------
        if paths(pth_idx).pparam1 == 0
            
            [theta_cmd_0, phi_cmd_0, etalon_0, etalat_0] = ...
                L1guide_line(simout(10:12), d_states(10:12), ...
                [paths(pth_idx).pparam2, paths(pth_idx).pparam3, paths(pth_idx).pparam4], ...
                [paths(pth_idx).pparam5, paths(pth_idx).pparam6, paths(pth_idx).pparam7], ...
                L1p_lon, L1d_lon, intg_lon, ...
                L1p_lat, L1d_lat, intg_lat, ...
                theta_cmd_min, theta_cmd_max, phi_cmd_min, phi_cmd_max);
        
        elseif paths(pth_idx).pparam1 == 1
                
            [theta_cmd_0, phi_cmd_0, etalon_0, etalat_0] = ...
                L1guide_spiral(simout(10:12), d_states(10:12), ...
                [paths(pth_idx).pparam2, paths(pth_idx).pparam3, paths(pth_idx).pparam4], ...
                paths(pth_idx).pparam5, paths(pth_idx).pparam6, paths(pth_idx).pparam8, paths(pth_idx).pparam7, ...
                L1p_lon, L1d_lon, intg_lon, ...
                L1p_lat, L1d_lat, intg_lat,...
                theta_cmd_min, theta_cmd_max, phi_cmd_min, phi_cmd_max, paths(pth_idx).pparam9);
            
        end
        
        % path next -------------------------------------------------------
        if paths(pth_idx+1).pparam1 == 0
            
            [theta_cmd_1, phi_cmd_1, etalon_1, etalat_1] = ...
                L1guide_line(simout(10:12), d_states(10:12), ...
                [paths(pth_idx+1).pparam2, paths(pth_idx+1).pparam3, paths(pth_idx+1).pparam4], ...
                [paths(pth_idx+1).pparam5, paths(pth_idx+1).pparam6, paths(pth_idx+1).pparam7], ...
                L1p_lon, L1d_lon, intg_lon, ...
                L1p_lat, L1d_lat, intg_lat, ...
                theta_cmd_min, theta_cmd_max, phi_cmd_min, phi_cmd_max);
        
        elseif paths(pth_idx+1).pparam1 == 1
                
            [theta_cmd_1, phi_cmd_1, etalon_1, etalat_1] = ...
                L1guide_spiral(simout(10:12), d_states(10:12), ...
                [paths(pth_idx+1).pparam2, paths(pth_idx+1).pparam3, paths(pth_idx+1).pparam4], ...
                paths(pth_idx+1).pparam5, paths(pth_idx+1).pparam6, paths(pth_idx+1).pparam8, paths(pth_idx+1).pparam7, ...
                L1p_lon, L1d_lon, intg_lon, ...
                L1p_lat, L1d_lat, intg_lat,...
                theta_cmd_min, theta_cmd_max, phi_cmd_min, phi_cmd_max, paths(pth_idx+1).pparam9);
            
        end
        
        gam_trans = trans_count / N_trans;
        one_m_gam_trans = (1 - gam_trans);
        
        theta_cmd   = one_m_gam_trans * theta_cmd_0 + gam_trans * theta_cmd_1;
        phi_cmd     = one_m_gam_trans * phi_cmd_0 + gam_trans * phi_cmd_1;
        etalon      = one_m_gam_trans * etalon_0 + gam_trans * etalon_1;
        etalat      = one_m_gam_trans * etalat_0 + gam_trans * etalat_1;

        if trans_count >= N_trans
            pth_idx = pth_idx + 1;
            trans_count = 0;
            in_trans = false;
            seg_term = false;
        end
        
    elseif time(k)==floor(time(k)/Ts_L1)*Ts_L1

        if paths(pth_idx).pparam1 == 0
            
            [theta_cmd, phi_cmd, etalon, etalat] = ...
                L1guide_line(simout(10:12), d_states(10:12), ...
                [paths(pth_idx).pparam2, paths(pth_idx).pparam3, paths(pth_idx).pparam4], ...
                [paths(pth_idx).pparam5, paths(pth_idx).pparam6, paths(pth_idx).pparam7], ...
                L1p_lon, L1d_lon, intg_lon, ...
                L1p_lat, L1d_lat, intg_lat, ...
                theta_cmd_min, theta_cmd_max, phi_cmd_min, phi_cmd_max);
        
        elseif paths(pth_idx).pparam1 == 1
                
            [theta_cmd, phi_cmd, etalon, etalat] = ...
                L1guide_spiral(simout(10:12), d_states(10:12), ...
                [paths(pth_idx).pparam2, paths(pth_idx).pparam3, paths(pth_idx).pparam4], ...
                paths(pth_idx).pparam5, paths(pth_idx).pparam6, paths(pth_idx).pparam8, paths(pth_idx).pparam7, ...
                L1p_lon, L1d_lon, intg_lon, ...
                L1p_lat, L1d_lat, intg_lat,...
                theta_cmd_min, theta_cmd_max, phi_cmd_min, phi_cmd_max, paths(pth_idx).pparam9);
            
        else
            
            theta_cmd = 0; phi_cmd = 0; etalon = 0; etalat = 0;
            
        end

    end

    intg_lon = intg_lon + kilon * etalon * Ts;
    intg_lat = intg_lat + kilat * etalat * Ts;

    yref        = [V_cmd, theta_cmd, phi_cmd, 0, 0, 0, 0]; 
    input.y     = [repmat(yref,N,1), zeros(N,5)];
    input.yN    = input.y(end,1:7)';
    
    if time(k)==floor(time(k)/Ts_nmpc)*Ts_nmpc
    
    %  - - - - - START NMPC - - - - -
    st_nmpc = tic;
        
     % shift initial states and controls
    if k > 1
    input.x     = spline(0:Ts_step:(N*Ts_step),output.x',Ts_nmpc:Ts_step:(N*Ts_step+Ts_nmpc))';
    input.u     = spline(0:Ts_step:((N-1)*Ts_step),output.u',Ts_nmpc:Ts_step:((N-1)*Ts_step+Ts_nmpc))';
    X0(10:15)   = output.x(2,10:15);
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
    X_rec(k,:)       = [simout, X0(11)];
    Y_rec(k,:)       = [V_cmd, theta_cmd, phi_cmd, intg_lon, intg_lat, etalon, etalat];
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

plotsim
