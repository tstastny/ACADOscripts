% simulink start up script

clear; clc;

% PARAMETERS
load parameters_2016.03.07_1836.mat
for i=1:length(parameters)
    eval(['slparams.',parameters(i).Name,'=',num2str(parameters(i).Value),';']);
end
busInfo = Simulink.Bus.createObject(slparams);

% INITIAL CONDITIONS
states_IC = [ ...
    13.9;
    0;
    1.22;
    0;
    0;
    0;
    0;
    0.0873;
    0;
    0;
    0;
    0;
    0.4];

    
    