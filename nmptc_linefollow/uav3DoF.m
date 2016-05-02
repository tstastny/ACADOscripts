function [d_states, simout, cost] = uav3DoF(time, states, ctrls, wn, we, wd, V, tau_mu, tau_gamma, a, b)

% state defines
n = states(1);
e = states(2);
d = states(3);
mu = states(4);
gamma = states(5);
xi = states(6);

% control defines
mu_cmd = ctrls(1);
gamma_cmd = ctrls(2);

% differentials
d_states(1) = V * cos(gamma) * cos(xi) + wn;
d_states(2) = V* cos(gamma) * sin(xi) + we;
d_states(3) = -V * sin(gamma) + wd;
d_states(4) = (mu_cmd - mu) / tau_mu;
d_states(5) = (gamma_cmd - gamma) / tau_gamma;
d_states(6) = 9.81 * tan(mu) / V;

% output
simout(1) = n;
simout(2) = e;
simout(3) = d;
simout(4) = mu;
simout(5) = gamma;
simout(6) = xi;

% cost
an = a(1);
ae = a(2);
ad = a(3);
bn = b(1);
be = b(2);
bd = b(3);

% calculate vector from waypoint a to b
abn = bn - an;
abe = be - ae;
abd = bd - ad;
norm_ab2 = abn*abn + abe*abe + abd*abd;
norm_ab = sqrt(norm_ab2);
abn_unit = abn / norm_ab;
abe_unit = abe / norm_ab;
abd_unit = abd / norm_ab;

% track position error
pan = an - n;
pae = ae - e;
pad = ad - d;
cx = abe_unit*pad - pae*abd_unit;
cy = -(abn_unit*pad - pan*abd_unit);
cz = abn_unit*pae - pan*abe_unit;
et = sqrt( cx^2 + cy^2 + cz^2 );

cost(1) = et;
