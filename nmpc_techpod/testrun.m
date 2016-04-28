
% Techpod NMPC simulation
% -----------------------
close all; clear all; clc;

% load aircraft parameters
load parameters_2016.03.09_1741.mat
for i = 1:length(parameters)
    eval([parameters(i).Name,' = ',num2str(parameters(i).Value),';'])
end

V_cmd = 14;

n = 0;
e = 0;
d = 0;

wn=0;
we=0;
wd=0;

% acado inputs
nmpc_ic.x   = [13.9806213280341,0.5,0.423306757146083,0,0.00112015092434004,0,0,0.0150406852113097,0.393056460359436,0,0,0.0492695583401457,-0.000810804484686847,0,0]; 
nmpc_ic.u   = [0.3913, -0.0064, 0, 0, 0];
yref        = [V_cmd, 0, 0, 0, 0, 0, 0]; 
Q_output    = [.01 1 .1 10 .1 1 .1];
R_controls  = [1 1 1 1, 2000];

% simulation
T0      = 0;
Tf      = 0.75;
Ts      = 0.05;
time    = T0:Ts:Tf;
KKT_MPC = []; INFO_MPC = []; controls_MPC = [];

% initial simout
X0          = nmpc_ic.x;
states      = [nmpc_ic.x(1:8), 0, n, e, d, nmpc_ic.x(9)];

[d_states, simout]  = techpod_nonlin_model_13DoF(time(1), states, nmpc_ic.u(1:4), ...
        cD0, cDa, cDa2, ...
        cL0, cLa, cLa2, cLa3, cLde, ...
        cm0, cma, cmq, cmde, ...
        cT0, cT1, cT2, tauT, ...
        clb, clp, clr, clda, ...
        cYb, ...
        cnb, cnp, cnr, cndr, ...
        wn, we, wd);
    
d_states([2,4,6,7,9])