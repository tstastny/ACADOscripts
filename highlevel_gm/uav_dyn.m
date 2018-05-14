function [d_states, output] = uav_dyn(time, states, controls, onlinedata)
%
% UAV DYNAMICS
%
%--------------------------------------------------------------------------

% states
% n = states(1);
% e = states(2);
% d = states(3);
gamma = states(4);
xi = states(5);
mu = states(6);

% controls
gamma_ref = controls(1);
mu_ref = controls(2);

% online data
v = onlinedata(1);
wn = onlinedata(2);
we = onlinedata(3);
wd = onlinedata(4);

% model parameters
tau_gamma = 1;
tau_mu = 0.7;

% constants
g = 9.81; % acceleration of gravity [m/s2]

% state differentials
d_states(1) = v*cos(gamma)*cos(xi) + wn;
d_states(2) = v*cos(gamma)*sin(xi) + we;
d_states(3) = -v*sin(gamma) + wd;
d_states(4) = (gamma_ref - gamma)/tau_gamma;
d_states(5) = g*tan(mu)/v/cos(gamma);
d_states(6) = (mu_ref - mu)/tau_mu;

% output
output = states;

