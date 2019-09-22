clear; clc;

syms vne vnedot vd vddot psidot mass T alpha L D g phi;

vA = sqrt(vne^2 + vd^2);
vAdot = (vne*vnedot + vd*vddot) / vA;

rhs_vAdot = 1/mass*(T*cos(alpha) - D) + g*vd/vA;

gamdot = (vd/vA^2*vAdot - vddot/vA)/sqrt(1-(vd/vA)^2);

rhs_gamdot = 1/mass/vA*((T*sin(alpha) + L)*cos(phi) - mass*g*vne/vA);

sol = solve([vAdot == rhs_vAdot; gamdot == rhs_gamdot], [vnedot; vddot]);

sol_vnedot = simplify(sol.vnedot,'IgnoreAnalyticConstraints',true);
sol_vddot = simplify(sol.vddot,'IgnoreAnalyticConstraints',true);