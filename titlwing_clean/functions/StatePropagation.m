% Model Propagation
% ----------------
clear; clc; close all; 

len_t = 2001;
n_X = 4;


% allocate record struct
rec.x = zeros(len_t,n_X);


k_nmpc = 0;

% Initial States
v_x = 5;
v_z = 2;
theta = 0;
zeta_w =  deg2rad(45);

states = [v_x, v_z, theta, zeta_w];
controls = [0,0,0];

T0 = 0;     % sim start time
Tf = 20;    % sim end time
Ts = 0.01;  % simulation time step -- the model will be propagated at this rate
time = T0:Ts:Tf;

% simulate
for k = 1:len_t
    
    
    st_sim = tic;
    % apply control
    [dstates, simout]  = model_dynamics(time(k), states, controls);
    simout
    

    % integration (model propagation)
    states = states + dstates*Ts;
    
    
    % measurement update
    measurements = states;
    
    % record states/controls
    st_rec = tic;
    
    rec.x(k,:) = simout;
    trec(k) = toc(st_rec);

    % tell time
    if time(k)==floor(time(k)/10)*10
        clc;
        disp(['sim time = ',num2str(time(k)),' s']);
    end
   
    tsim(k) = toc(st_sim);
end

%% PLOTTING - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

figure
subplot(4,1,1);
plot(time(1:264),rec.x(1:264,1))
title('v_x');
subplot(4,1,2);
plot(time(1:264),rec.x(1:264,2));
title('v_z');
subplot(4,1,3);
plot(time,rad2deg(rec.x(:,3)));
title('\theta');
subplot(4,1,4);
plot(time,rad2deg(rec.x(:,4)));
title('\zeta_w');
