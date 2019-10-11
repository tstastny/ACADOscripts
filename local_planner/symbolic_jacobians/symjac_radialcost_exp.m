% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% symbolic jacobian for radial cost (cubic formulation)
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
k_r = sym('k_r','real');
r_offset = sym('r_offset','real');
log_sqrt_w_over_sig1_r = sym('log_sqrt_w_over_sig1_r','real');

vG_n_expr = v*cos(gamma)*cos(xi) + w_n;
vG_e_expr = v*cos(gamma)*sin(xi) + w_e;
vG_d_expr = -v*sin(gamma) + w_d;
vG_sq_expr = vG_n_expr^2 + vG_e_expr^2 + vG_d_expr^2;
vG_expr = sqrt(vG_sq_expr);

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
r_tl_expr = -(A_tl*(r_e-p1_e) + B_tl*(r_n-p1_n) + C_tl*(-r_d-p1_h) + D_tl) * vG_expr / (A_tl*vG_e_expr + B_tl*vG_n_expr + C_tl*-vG_d_expr);
% distance to triangulated plane (br)
r_br_expr = -(A_br*(r_e-p1_e) + B_br*(r_n-p1_n) + C_br*(-r_d-p1_h) + D_br) * vG_expr / (A_br*vG_e_expr + B_br*vG_n_expr + C_br*-vG_d_expr);

% radial cost (when buffer is violated)
delta_r_expr = delta_r0 + vG_sq_expr*k_r;
sig_r_tl = exp(-(r_tl_expr - r_offset)/delta_r_expr*log_sqrt_w_over_sig1_r);
sig_r_br = exp(-(r_br_expr - r_offset)/delta_r_expr*log_sqrt_w_over_sig1_r);

% jacobian of radial cost
jac_sig_r_tl_exp = jacobian(sig_r_tl, [r_n; r_e; r_d; v; gamma; xi]);
jac_sig_r_br_exp = jacobian(sig_r_br, [r_n; r_e; r_d; v; gamma; xi]);

%% substitute calculations we'll have already done
sig_r = sym('sig_r','real');
jac_sig_r_tl_exp = subs(jac_sig_r_tl_exp , sig_r_tl, sig_r);
jac_sig_r_br_exp = subs(jac_sig_r_br_exp , sig_r_br, sig_r);
d_occ = sym('d_occ','real');
jac_sig_r_tl_exp = subs(jac_sig_r_tl_exp , r_tl_expr, d_occ);
jac_sig_r_br_exp = subs(jac_sig_r_br_exp , r_br_expr, d_occ);
delta_r = sym('delta_r','real');
jac_sig_r_tl_exp = subs(jac_sig_r_tl_exp , delta_r_expr, delta_r);
jac_sig_r_br_exp = subs(jac_sig_r_br_exp , delta_r_expr, delta_r);
vG = sym('vG','real');
jac_sig_r_tl_exp = subs(jac_sig_r_tl_exp , vG_expr, vG);
jac_sig_r_br_exp = subs(jac_sig_r_br_exp , vG_expr, vG);
vG_sq = sym('vG_sq','real');
% jac_sig_r_tl_exp = subs(jac_sig_r_tl_exp , vG_sq_expr, vG_sq);
% jac_sig_r_br_exp = subs(jac_sig_r_br_exp , vG_sq_expr, vG_sq);
jac_sig_r_tl_exp = subs(jac_sig_r_tl_exp , vG^2, vG_sq);
jac_sig_r_br_exp = subs(jac_sig_r_br_exp , vG^2, vG_sq);
% vG = sym('vG','real');
jac_sig_r_tl_exp = subs(jac_sig_r_tl_exp , vG_sq^(1/2), vG);
jac_sig_r_br_exp = subs(jac_sig_r_br_exp , vG_sq^(1/2), vG);
vG_n = sym('vG_n','real');
jac_sig_r_tl_exp = subs(jac_sig_r_tl_exp , vG_n_expr, vG_n);
jac_sig_r_br_exp = subs(jac_sig_r_br_exp , vG_n_expr, vG_n);
vG_e = sym('vG_e','real');
jac_sig_r_tl_exp = subs(jac_sig_r_tl_exp , vG_e_expr, vG_e);
jac_sig_r_br_exp = subs(jac_sig_r_br_exp , vG_e_expr, vG_e);
vG_d = sym('vG_d','real');
jac_sig_r_tl_exp = subs(jac_sig_r_tl_exp , vG_d_expr, vG_d);
jac_sig_r_br_exp = subs(jac_sig_r_br_exp , vG_d_expr, vG_d);

%% export to m code
if 1
matlabFunction(jac_sig_r_tl_exp,'File','jac_sig_r_tl_exp.m', ...
    'Vars',{r_n, r_e, r_d, v, gamma, xi, w_e, w_n, w_d, ...
    terr_dis, p1_n, p1_e, p1_h, p2_n, p2_e, p2_h, p3_n, p3_e, p3_h, ...
    r_offset, delta_r0, k_r, log_sqrt_w_over_sig1_r, ...
    sig_r, d_occ, delta_r, vG_sq, vG, vG_n, vG_e, vG_d},'Outputs',{'out'});
matlabFunction(jac_sig_r_br_exp,'File','jac_sig_r_br_exp.m', ...
    'Vars',{r_n, r_e, r_d, v, gamma, xi, w_e, w_n, w_d, ...
    terr_dis, p1_n, p1_e, p1_h, p2_n, p2_e, p2_h, p3_n, p3_e, p3_h, ...
    r_offset, delta_r0, k_r, log_sqrt_w_over_sig1_r, ...
    sig_r, d_occ, delta_r, vG_sq, vG, vG_n, vG_e, vG_d},'Outputs',{'out'});
end

%% export to c code
if 1
ccode(jac_sig_r_tl_exp,'file','jac_sig_r_tl_exp_ccode.c');
ccode(jac_sig_r_br_exp,'file','jac_sig_r_br_exp_ccode.c');
end

%% prep c code for mpc model functions
if 1
% br
fid = fopen('jac_sig_r_br_exp_ccode.c');
txt = textscan(fid,'%s','delimiter','\n'); 
fclose(fid);
txt = txt{1};
for i = 1:length(txt)
    
    str1 = txt{i};
    
    if strcmp(str1(1),'t')
        str1 = ['const double ',str1];
        txt{i} = str1;
        % t's 
    elseif length(str1) > 5
        % A0's
        if strcmp(str1(1:6),'A0[0][')
            str1 = ['jac[',str1(7:end)];
            txt{i} = str1;
        end
    end
end
fid = fopen('jac_sig_r_br_exp_ccode.c','w');
for k = 1:length(txt)
    fprintf(fid,[char(txt{k}),' \n']);
end
fclose(fid);
% tl
fid = fopen('jac_sig_r_tl_exp_ccode.c');
txt = textscan(fid,'%s','delimiter','\n'); 
fclose(fid);
txt = txt{1};
for i = 1:length(txt)
    
    str1 = txt{i};
    
    if strcmp(str1(1),'t')
        str1 = ['const double ',str1];
        txt{i} = str1;
        % t's 
    elseif length(str1) > 5
        % A0's
        if strcmp(str1(1:6),'A0[0][')
            str1 = ['jac[',str1(7:end)];
            txt{i} = str1;
        end
    end
end
fid = fopen('jac_sig_r_tl_exp_ccode.c','w');
for k = 1:length(txt)
    fprintf(fid,[char(txt{k}),' \n']);
end
end

