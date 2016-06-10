function [d_states, simout] = uav3DoF(time, states, ctrls, wn, we, V)

% state defines
n = states(1);
e = states(2);
mu = states(3);
xi = states(4);
mu_dot = states(5);

% control defines
mu_r = ctrls(1);

% differentials
d_states(1) = V * cos(xi) + wn;
d_states(2) = V * sin(xi) + we;
d_states(3) = mu_dot;
d_states(4) = 9.81 * tan(mu) / V;
d_states(5) = 13.48 * mu_r - 6.577 * mu_dot - 13.97 * mu;

% output
simout(1) = n;
simout(2) = e;
simout(3) = mu;
simout(4) = xi;
simout(5) = mu_dot;
