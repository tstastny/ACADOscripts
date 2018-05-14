function [out,aux] = eval_obj(states,controls,onlinedata)

% states
n = states(1);
e = states(2);
d = states(3);
gamma = states(4);
% xi = states(5);
mu = states(6);

% controls
gamma_ref = controls(1);
mu_ref = controls(2);

% online data
% v = onlinedata(1);
% wn = onlinedata(2);
% we = onlinedata(3);
% wd = onlinedata(4);
bn = onlinedata(5);
be = onlinedata(6);
bd = onlinedata(7);
gamma_p = onlinedata(8);
chi_p = onlinedata(9);

% model parameters
tau_gamma = 1;
tau_mu = 0.7;

% constants
% g = 9.81; % acceleration of gravity [m/s2]

% state differentials
% d_states(1) = v*cos(gamma)*cos(xi) + wn;
% d_states(2) = v*cos(gamma)*sin(xi) + we;
% d_states(3) = -v*sin(gamma) + wd;
dot_gamma = (gamma_ref - gamma)/tau_gamma;
% d_states(5) = g*tan(mu)/v/cos(gamma);
dot_mu = (mu_ref - mu)/tau_mu;

% closest point on track
dot_vp_br = cos(chi_p)*cos(gamma_p)*(n-bn) + sin(chi_p)*cos(gamma_p)*(e-be) - ...
    sin(gamma_p)*(d-bd);
pn = bn + dot_vp_br*cos(chi_p)*cos(gamma_p);
pe = be + dot_vp_br*sin(chi_p)*cos(gamma_p);
pd = bd - dot_vp_br*sin(gamma_p);

% path objectives
e_lat = (n-pn)*sin(chi_p) - (e-pe)*cos(chi_p);
e_lon = -(pd-d);

% state output
y = [ e_lat, e_lon];

% control output
z = [ gamma_ref, mu_ref, dot_gamma, dot_mu ];

out = [y, z];
aux = 0;