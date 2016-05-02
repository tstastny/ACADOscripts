function [d_states, simout, cost] = uav3DoF(time, states, ctrls, wn, we, wd, V, tau_mu, tau_gamma, c, R, dir)

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
cn = c(1);
ce = c(2);
cd = c(3);

% calculate closest point on loiter circle
cpn = n - cn;
cpe = e - ce;
norm_cp = sqrt( cpn^2 + cpe^2 );
cdn = R * cpn / norm_cp;
cde = R * cpe / norm_cp;
dn = n + cdn - cpn;
de = e + cde - cpe;
dd = cd;

% calculate tangent
vdn = dir*-cde;
vde = dir*cdn;
vdd = 0;

% track position error
pdn = dn - n;
pde = de - e;
pdd = dd - d;
cx = vde*pdd - pde*vdd;
cy = -(vdn*pdd - pdn*vdd);
cz = vdn*pde - pdn*vde;
et = sqrt( cx^2 + cy^2 + cz^2 );

cost(1) = et;
