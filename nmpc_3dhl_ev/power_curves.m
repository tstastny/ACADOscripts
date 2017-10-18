clear; clc;

gamma = deg2rad(8);

load parameters_20161209.mat;
for i = 1:length(parameters)
    eval([parameters(i).Name,'=',num2str(parameters(i).Value),';']);
end

syms V theta P;

% environment/
g = 9.81;
m = 2.65;
rho = 1.225;
S = 0.39;
qbarS = 0.5*rho*V^2*S;

% aero coefficients
alpha = (theta - gamma);
cD = cD0 + cDa * alpha + cDa2 * alpha^2;
cL = cL0 + cLa * alpha + cLa2 * alpha^2;

% forces1010
P0 = (cT1*delta_T+cT2*delta_T^2+cT3*delta_T^3);
T = P/V/cos(alpha); 
D = qbarS*cD;
L = qbarS*cL;

eqn = (P*tan(alpha)/V+L - m*g*cos(gamma))/m/V;

P = V*(m*g*cos(gamma) - L)/tan(alpha);