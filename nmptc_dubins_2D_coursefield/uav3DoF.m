function [d_states, simout, cost] = uav3DoF(time, states, ctrls, wn, we, V, dyn, pparams)

% state defines
n = states(1);
e = states(2);
mu = states(3);
xi = states(4);
mu_dot = states(5);

% control defines
mu_r = ctrls(1);

omega_n_mu = dyn(1);
zeta_mu = dyn(2);

% differentials
d_states(1) = V * cos(xi) + wn;
d_states(2) = V * sin(xi) + we;
d_states(3) = mu_dot;
d_states(4) = 9.81 * tan(mu) / V;
d_states(5) = omega_n_mu * (omega_n_mu * (mu_r - mu) - 2 * zeta_mu * mu_dot);

% output
simout(1) = n;
simout(2) = e;
simout(3) = mu;
simout(4) = xi;
simout(5) = mu_dot;

% cost
pparam1 = pparams(1);
pparam2 = pparams(2);
pparam3 = pparams(3);
pparam4 = pparams(4);
pparam5 = pparams(5);
pparam6 = pparams(6);
pparam7 = pparams(7);
pparam8 = pparams(8);
pparam9 = pparams(9);

cost(1) = 0;
