% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% symbolic jacobian for height cost (exponential formulation)
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

clear; clc;

r_n = sym('r_n','real');
r_e = sym('r_e','real');
r_d = sym('r_d','real');
xi = sym('xi','real');
h_offset = sym('h_offset','real');
delta_h = sym('delta_h','real');
delta_y = sym('delta_y','real');
log_sqrt_w_over_sig1_h = sym('log_sqrt_w_over_sig1_h','real');
terr_dis = sym('terr_dis','real');
sgn_n = sym('sgn_n','real');
sgn_e = sym('sgn_e','real');
n_floor = sym('n_floor','real');
e_floor = sym('e_floor','real');
h1 = sym('h1','real');
h2 = sym('h2','real');
h3 = sym('h3','real');
h4 = sym('h4','real');

% bilinear interpolation
dn_expr = (r_n + sgn_n * sin(xi) * delta_y) / terr_dis - n_floor;
de_expr = (r_e + sgn_e * cos(xi) * delta_y) / terr_dis - e_floor;
h12_expr = [h1 h2] * [(1-dn_expr); dn_expr];
h34_expr = [h3 h4] * [(1-dn_expr); dn_expr];
h_terr = [(1-de_expr) de_expr] * [h12_expr; h34_expr];

% cost
sig_h_expr = exp(-(-r_d - h_terr - h_offset)/delta_h*log_sqrt_w_over_sig1_h);
sig_h_lin_expr = 1.0 + -log_sqrt_w_over_sig1_h/delta_h * (-r_d - h_terr - h_offset);

% jacobian
jac_sig_h_exp = jacobian(sig_h_expr, [r_n; r_e; r_d; xi]);
jac_sig_h_lin = jacobian(sig_h_lin_expr, [r_n; r_e; r_d; xi]);

%% substitute calculations we'll have already done
sig_h = sym('sig_h','real');
jac_sig_h_exp = subs(jac_sig_h_exp , sig_h_expr, sig_h);
h12 = sym('h12','real');
jac_sig_h_exp = subs(jac_sig_h_exp , h12_expr, h12);
h34 = sym('h34','real');
jac_sig_h_exp = subs(jac_sig_h_exp , h34_expr, h34);
dn = sym('dn','real');
jac_sig_h_exp = subs(jac_sig_h_exp , dn_expr, dn);
de = sym('de','real');
jac_sig_h_exp = subs(jac_sig_h_exp , de_expr, de);

jac_sig_h_lin = subs(jac_sig_h_lin , sig_h_expr, sig_h);
jac_sig_h_lin = subs(jac_sig_h_lin , h12_expr, h12);
jac_sig_h_lin = subs(jac_sig_h_lin , h34_expr, h34);
jac_sig_h_lin = subs(jac_sig_h_lin , dn_expr, dn);
jac_sig_h_lin = subs(jac_sig_h_lin , de_expr, de);

jac_sig_h_exp = simplify(jac_sig_h_exp);
jac_sig_h_lin = simplify(jac_sig_h_lin);

%% get input arguments (used variables)
input_arg = sym2cell(symvar(jac_sig_h_exp));
input_arg_lin = sym2cell(symvar(jac_sig_h_lin));

if 0
%% export to m code
matlabFunction(jac_sig_h_exp,'File','jac_sig_h_exp.m','Vars',input_arg,'Outputs',{'out'});
%%
matlabFunction(jac_sig_h_lin,'File','jac_sig_h_lin.m','Vars',input_arg_lin,'Outputs',{'out'});
end

if 0
%% export to c code
ccode(jac_sig_h_exp,'file','jac_sig_h_exp_ccode.c');
%%
ccode(jac_sig_h_lin,'file','jac_sig_h_lin_ccode.c');
end

if 0
%% prep c code for mpc model functions
fid = fopen('jac_sig_h_exp_ccode.c');
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
fid = fopen('jac_sig_h_exp_ccode.c','w');
for k = 1:length(txt)
    fprintf(fid,[char(txt{k}),' \n']);
end
fclose(fid);
%%
fid = fopen('jac_sig_h_lin_ccode.c');
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
fid = fopen('jac_sig_h_lin_ccode.c','w');
for k = 1:length(txt)
    fprintf(fid,[char(txt{k}),' \n']);
end
fclose(fid);
end

