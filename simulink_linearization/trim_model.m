% linearize model
clear; clc;

% PARAMETERS
load parameters_2016.03.10_1254.mat
for i=1:length(parameters)
    eval(['slparams.',parameters(i).Name,'=',num2str(parameters(i).Value),';']);
end
busInfo = Simulink.Bus.createObject(slparams);

states_IC = [14.048;
%     0;
    0.027;
%     0;
    0;
%     0;
%     0;
    0.027;
    0.452];

ctrls_IC = [0.452;
    -0.005];
%     0;
%     0];

op = operpoint('nonlinear_model');
op.States.x = states_IC;
op.Inputs.u = ctrls_IC;

linsys = linearize('nonlinear_model',op);

