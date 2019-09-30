% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% symbolic jacobian for radial ray casting cost
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

clear; clc;

r_n = sym('r_n','real');      % aircraft position
r_e = sym('r_e','real');
r_d = sym('r_d','real');
v = sym('v','real');        % velocity axis
xi = sym('xi','real');
gamma = sym('gamma','real');
w_n = sym('w_n','real');      % wind
w_e = sym('w_e','real');
w_d = sym('w_d','real');
terr_dis = sym('terr_dis','real'); % terrain discretization
p1_n = sym('p1_n','real');     % first terrain interpolation point
p1_e = sym('p1_e','real');
p1_h = sym('p1_h','real');
p2_n = sym('p2_n','real');     % second terrain interpolation point
p2_e = sym('p2_e','real');
p2_h = sym('p2_h','real');
p3_n = sym('p3_n','real');     % third terrain interpolation point
p3_e = sym('p3_e','real');
p3_h = sym('p3_h','real');
delta_r0 = sym('delta_r0','real'); % radial cost buffer params
g = sym('g','real');
phi_max = sym('phi_max','real');
k_r = sym('k_r','real');

vG = [ ...
    v*cos(gamma)*cos(xi) + w_n;
    v*cos(gamma)*sin(xi) + w_e;
    -v*sin(gamma) + w_d;
    ];
vG_2 = vG(1)^2 + vG(2)^2 + vG(3)^2;
norm_vG = sqrt(vG_2);

% A = y2 * z3 - y3 * z2 - y1 * (z3 - z2) + z1 * (y3 - y2);
% B = x1 * (z3 - z2 ) - (x2 * z3 - x3 * z2) + z1 * (x2 - x3);
% C = x1 * (y2 - y3) - y1 * (x2 - x3) + (x2 * y3 - x3 * y2);
% D = x1 * (y2 * z3 - y3 * z2) + y1 * (x2 * z3 - x3 * z2) - z1 * (x2 * y3 - x3 * y2);

% triangulated plane coefficients
% (top left, right triangle)
A_tl = terr_dis * (p3_h - p2_h);
B_tl = terr_dis * (p2_h - p1_h);
C_tl = -terr_dis^2;
D_tl = p1_h * terr_dis^2; 
% (bottom right, right triangle)
A_br = terr_dis * (p1_h - p2_h);
B_br = terr_dis * (p2_h - p3_h);
C_br = terr_dis^2;
D_br = -p1_h * terr_dis^2;

% distance to triangulated plane (ul)
r_tl = -(A_tl*(r_e-p1_e) + B_tl*(r_n-p1_n) + C_tl*(-r_d-p1_h) + D_tl) * norm_vG / (A_tl*vG(2) + B_tl*vG(1) + C_tl*-vG(3));
% distance to triangulated plane (br)
r_br = -(A_br*(r_e-p1_e) + B_br*(r_n-p1_n) + C_br*(-r_d-p1_h) + D_br) * norm_vG / (A_br*vG(2) + B_br*vG(1) + C_br*-vG(3));

% radial cost (when buffer is violated)
delta_r = delta_r0 + vG_2/g/tan(phi_max)*k_r;
sig_r_tl = (delta_r - r_tl)^3;
sig_r_br = (delta_r - r_br)^3;

% jacobian of radial cost
jac_sig_r_tl = jacobian(sig_r_tl, [r_n; r_e; r_d; v; gamma; xi]);
jac_sig_r_br = jacobian(sig_r_br, [r_n; r_e; r_d; v; gamma; xi]);


%% export to m code
matlabFunction(jac_sig_r_tl,'File','jac_sig_r_tl.m', ...
    'Vars',{r_n, r_e, r_d, v, gamma, xi, w_e, w_n, w_d, ...
    terr_dis, p1_n, p1_e, p1_h, p2_n, p2_e, p2_h, p3_n, p3_e, p3_h, phi_max, ...
    delta_r0, g, k_r},'Outputs',{'out'});
matlabFunction(jac_sig_r_br,'File','jac_sig_r_br.m', ...
    'Vars',{r_n, r_e, r_d, v, gamma, xi, w_e, w_n, w_d, ...
    terr_dis, p1_n, p1_e, p1_h, p2_n, p2_e, p2_h, p3_n, p3_e, p3_h, phi_max, ...
    delta_r0, g, k_r},'Outputs',{'out'});

%% export to c code
if 0
    ccode(jac_sig_r_ul,'jac_sig_r_tl_ccode.c');
    ccode(jac_sig_r_br,'jac_sig_r_br_ccode.c');
end
