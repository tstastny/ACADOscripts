% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% symbolic jacobian for radial distance to triangle on grid
% > individual r_offset / delta_r tuning
% > relative speed formulation
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
p_occ_n = sym('p_occ_n','real');     % triangle-ray intersection
p_occ_e = sym('p_occ_e','real');
p_occ_h = sym('p_occ_h','real');
n_occ_n = sym('n_occ_n','real');     % plane unit normal vector
n_occ_e = sym('n_occ_e','real');
n_occ_h = sym('n_occ_h','real');
delta_r0 = sym('delta_r0','real'); % radial cost buffer params
k_r_offset = sym('k_r_offset','real');
k_delta_r = sym('k_delta_r','real');
r_offset = sym('r_offset','real');
v_ray_n = sym('v_ray_n','real');
v_ray_e = sym('v_ray_e','real');
v_ray_h = sym('v_ray_h','real');

vG_n_expr = v*cos(gamma)*cos(xi) + w_n;
vG_e_expr = v*cos(gamma)*sin(xi) + w_e;
vG_d_expr = -v*sin(gamma) + w_d;
vG_sq_expr = vG_n_expr^2 + vG_e_expr^2 + vG_d_expr^2;
vG_expr = sqrt(vG_sq_expr);

% distance to triangulated plane
r_expr = -(n_occ_e*(r_e-p_occ_e) + n_occ_n*(r_n-p_occ_n) + n_occ_h*(-r_d-p_occ_h)) / ...
    (n_occ_e*v_ray_e + n_occ_n*v_ray_n + n_occ_h*v_ray_h);

% relative speed
v_rel_expr = vG_n_expr*v_ray_n + vG_e_expr*v_ray_e + -vG_d_expr*v_ray_h;
v_rel_sq_expr = v_rel_expr^2;

% radial distance function
delta_r_expr = delta_r0 + v_rel_sq_expr*k_delta_r;
r_offset_1_expr = r_offset + v_rel_sq_expr*k_r_offset;

% jacobian of radial distance function
jac_r = jacobian(r_expr, [r_n; r_e; r_d; v; gamma; xi]);
jac_r0 = jacobian(r_offset_1_expr, [r_n; r_e; r_d; v; gamma; xi]);
jac_Delta_r = jacobian(delta_r_expr, [r_n; r_e; r_d; v; gamma; xi]);

%% substitute calculations we'll have already done
r_occ = sym('r_occ','real');

jac_r = subs(jac_r , r_expr, r_occ);
jac_r = simplify(jac_r);

v_rel = sym('v_rel','real');
v_rel_sq = sym('v_rel_sq','real');
vG = sym('vG','real');
vG_sq = sym('vG_sq','real');
vG_n = sym('vG_n','real');
vG_e = sym('vG_e','real');
vG_d = sym('vG_d','real');
r_offset_1 = sym('r_offset_1','real');

jac_r0 = subs(jac_r0 , v_rel_expr, v_rel);
jac_r0 = subs(jac_r0 , v_rel_sq_expr, v_rel_sq);
jac_r0 = subs(jac_r0 , v_rel^2, v_rel_sq);
jac_r0 = subs(jac_r0 , vG_expr, vG);
jac_r0 = subs(jac_r0 , vG_sq_expr, vG_sq);
jac_r0 = subs(jac_r0 , vG^2, vG_sq);
jac_r0 = subs(jac_r0 , vG_sq^(1/2), vG);
jac_r0 = subs(jac_r0 , vG_n_expr, vG_n);
jac_r0 = subs(jac_r0 , vG_e_expr, vG_e);
jac_r0 = subs(jac_r0 , vG_d_expr, vG_d);
jac_r0 = subs(jac_r0 , r_offset + v_rel_sq*k_r_offset, r_offset_1);
jac_r0 = simplify(jac_r0);

delta_r = sym('delta_r','real');

jac_Delta_r = subs(jac_Delta_r , v_rel_expr, v_rel);
jac_Delta_r = subs(jac_Delta_r , v_rel_sq_expr, v_rel_sq);
jac_Delta_r = subs(jac_Delta_r , v_rel^2, v_rel_sq);
jac_Delta_r = subs(jac_Delta_r , vG_expr, vG);
jac_Delta_r = subs(jac_Delta_r , vG_sq_expr, vG_sq);
jac_Delta_r = subs(jac_Delta_r , vG^2, vG_sq);
jac_Delta_r = subs(jac_Delta_r , vG_sq^(1/2), vG);
jac_Delta_r = subs(jac_Delta_r , vG_n_expr, vG_n);
jac_Delta_r = subs(jac_Delta_r , vG_e_expr, vG_e);
jac_Delta_r = subs(jac_Delta_r , vG_d_expr, vG_d);
jac_Delta_r = subs(jac_Delta_r , delta_r0 + v_rel_sq*k_delta_r, delta_r);
jac_Delta_r = subs(jac_Delta_r , delta_r_expr, delta_r);
jac_Delta_r = simplify(jac_Delta_r);

%% loop it
strs = {'r', 'r0', 'Delta_r'};
sym_jacs = {jac_r, jac_r0, jac_Delta_r};
doit = true;
for ss = 1:length(strs)

%% get input arguments (used variables)
input_arg = sym2cell(symvar(sym_jacs{ss}));

if doit
%% export to m code
matlabFunction(sym_jacs{ss},'File',['jac_',strs{ss},'.m'],'Vars',input_arg,'Outputs',{'out'});
end

if doit
%% export to c code
ccode(sym_jacs{ss},'file',['jac_',strs{ss},'_ccode.c']);
end

if doit
%% prep c code for mpc model functions
fid = fopen(['jac_',strs{ss},'_ccode.c']);
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
txt_top = cell(2,1);
txt_top{1} = ['void jacobian_',strs{ss},'(double *jac,'];
txt_top{2} = ['const double ', char(input_arg{1})];
for k = 2:length(input_arg)
    txt_top{2} = [txt_top{2}, ', const double ', char(input_arg{k})];
end
txt_top{2} = [txt_top{2}, ') {\n'];
txt_top{3} = '/* w.r.t.:';
txt_top{4} = '    r_n';
txt_top{5} = '    r_e';
txt_top{6} = '    r_d';
txt_top{7} = '    v';
txt_top{8} = '    gamma';
txt_top{9} = '    xi';
txt_top{10} = '*/\n';
fid = fopen(['jac_',strs{ss},'_ccode.c'],'w');
for k = 1:length(txt_top)
    fprintf(fid,[char(txt_top{k}),' \n']);
end
for k = 1:length(txt)
    fprintf(fid,[char(txt{k}),' \n']);
end
fprintf(fid,[char('}'),'\n']);
fclose(fid);
end

end
