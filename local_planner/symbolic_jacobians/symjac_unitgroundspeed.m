% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% symbolic jacobian for unit ground speed
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

clear; clc;

v = sym('v','real');
gamma = sym('gamma','real');
xi = sym('xi','real');
w_n = sym('w_n','real');
w_e = sym('w_e','real');
w_d = sym('w_d','real');
v_n_cmd = sym('v_n_cmd','real');
v_e_cmd = sym('v_e_cmd','real');
v_d_cmd = sym('v_d_cmd','real');

% airspeed
v_n_expr = v*cos(gamma)*cos(xi);
v_e_expr = v*cos(gamma)*sin(xi);
v_d_expr = -v*sin(gamma);

% ground speed
vG_n_expr = v_n_expr + w_n;
vG_e_expr = v_e_expr + w_e;
vG_d_expr = v_d_expr + w_d;
vG_norm_expr = sqrt(vG_n_expr^2 + vG_e_expr^2 + vG_d_expr^2);
one_over_vG_norm_expr = 1/vG_norm_expr;

% unit ground speed
vG_n_unit_expr = vG_n_expr * one_over_vG_norm_expr;
vG_e_unit_expr = vG_e_expr * one_over_vG_norm_expr;
vG_d_unit_expr = vG_d_expr * one_over_vG_norm_expr;

% jacobian
jac_v_n = jacobian(vG_n_unit_expr, [v; gamma; xi]);
jac_v_e = jacobian(vG_e_unit_expr, [v; gamma; xi]);
jac_v_d = jacobian(vG_d_unit_expr, [v; gamma; xi]);

%% substitute calculations we'll have already done
vG_n_unit = sym('vG_n_unit','real');
vG_e_unit = sym('vG_e_unit','real');
vG_d_unit = sym('vG_d_unit','real');
vG_norm = sym('vG_norm','real');
one_over_vG_norm = sym('one_over_vG_norm','real');
vG_n = sym('vG_n','real');
vG_e = sym('vG_e','real');
vG_d = sym('vG_d','real');
v_n = sym('v_n','real');
v_e = sym('v_e','real');
v_d = sym('v_d','real');

jac_v_n = subs(jac_v_n, one_over_vG_norm_expr, one_over_vG_norm);
jac_v_n = subs(jac_v_n, vG_n_expr, vG_n);
jac_v_n = subs(jac_v_n, vG_e_expr, vG_e);
jac_v_n = subs(jac_v_n, vG_d_expr, vG_d);
jac_v_n = subs(jac_v_n, v_n_expr, v_n);
jac_v_n = subs(jac_v_n, v_e_expr, v_e);
jac_v_n = subs(jac_v_n, v_d_expr, v_d);
jac_v_n = subs(jac_v_n, vG_norm_expr, vG_norm);
jac_v_n = subs(jac_v_n, vG_n/vG_norm, vG_n_unit);
jac_v_n = subs(jac_v_n, vG_n*one_over_vG_norm, vG_n_unit);
jac_v_n = subs(jac_v_n, vG_e/vG_norm, vG_e_unit);
jac_v_n = subs(jac_v_n, vG_e*one_over_vG_norm, vG_e_unit);
jac_v_n = subs(jac_v_n, vG_d/vG_norm, vG_d_unit);
jac_v_n = subs(jac_v_n, vG_d*one_over_vG_norm, vG_d_unit);

jac_v_e = subs(jac_v_e, one_over_vG_norm_expr, one_over_vG_norm);
jac_v_e = subs(jac_v_e, vG_n_expr, vG_n);
jac_v_e = subs(jac_v_e, vG_e_expr, vG_e);
jac_v_e = subs(jac_v_e, vG_d_expr, vG_d);
jac_v_e = subs(jac_v_e, v_n_expr, v_n);
jac_v_e = subs(jac_v_e, v_e_expr, v_e);
jac_v_e = subs(jac_v_e, v_d_expr, v_d);
jac_v_e = subs(jac_v_e, vG_norm_expr, vG_norm);
jac_v_e = subs(jac_v_e, vG_n/vG_norm, vG_n_unit);
jac_v_e = subs(jac_v_e, vG_n*one_over_vG_norm, vG_n_unit);
jac_v_e = subs(jac_v_e, vG_e/vG_norm, vG_e_unit);
jac_v_e = subs(jac_v_e, vG_e*one_over_vG_norm, vG_e_unit);
jac_v_e = subs(jac_v_e, vG_d/vG_norm, vG_d_unit);
jac_v_e = subs(jac_v_e, vG_d*one_over_vG_norm, vG_d_unit);

jac_v_d = subs(jac_v_d, vG_n_unit_expr, vG_n_unit);
jac_v_d = subs(jac_v_d, vG_e_unit_expr, vG_e_unit);
jac_v_d = subs(jac_v_d, vG_d_unit_expr, vG_d_unit);
jac_v_d = subs(jac_v_d, one_over_vG_norm_expr, one_over_vG_norm);
jac_v_d = subs(jac_v_d, vG_n_expr, vG_n);
jac_v_d = subs(jac_v_d, vG_e_expr, vG_e);
jac_v_d = subs(jac_v_d, vG_d_expr, vG_d);
jac_v_d = subs(jac_v_d, v_n_expr, v_n);
jac_v_d = subs(jac_v_d, v_e_expr, v_e);
jac_v_d = subs(jac_v_d, v_d_expr, v_d);
jac_v_d = subs(jac_v_d, vG_norm_expr, vG_norm);
jac_v_d = subs(jac_v_d, vG_n/vG_norm, vG_n_unit);
jac_v_d = subs(jac_v_d, vG_n*one_over_vG_norm, vG_n_unit);
jac_v_d = subs(jac_v_d, vG_e/vG_norm, vG_e_unit);
jac_v_d = subs(jac_v_d, vG_e*one_over_vG_norm, vG_e_unit);
jac_v_d = subs(jac_v_d, vG_d/vG_norm, vG_d_unit);
jac_v_d = subs(jac_v_d, vG_d*one_over_vG_norm, vG_d_unit);

jac_vg_unit = [jac_v_n, jac_v_e, jac_v_d];

%% get input arguments (used variables)
input_arg = sym2cell(symvar(jac_vg_unit));

if 0
%% export to m code
matlabFunction(jac_vg_unit,'File','jac_vg_unit.m','Vars',input_arg,'Outputs',{'out'});
end

if 0
%% export to c code
ccode(jac_vg_unit,'file','jac_vg_unit_ccode.c');
end

if 0
%% prep c code for mpc model functions
fid = fopen('jac_vg_unit_ccode.c');
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
fid = fopen('jac_vg_unit_ccode.c','w');
for k = 1:length(txt)
    fprintf(fid,[char(txt{k}),' \n']);
end
fclose(fid);
end

