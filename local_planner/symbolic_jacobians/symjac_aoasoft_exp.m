% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% symbolic jacobian for soft angle of attack constraint (exponential formulation)
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

clear; clc;

gamma = sym('gamma','real');
theta = sym('theta','real');
aoa_p = sym('aoa_p','real');
aoa_m = sym('aoa_m','real');
delta_aoa = sym('delta_aoa','real');
log_sqrt_w_over_sig1_aoa = sym('log_sqrt_w_over_sig1_aoa','real');

aoa = theta - gamma;

% positive bound 
sig_aoa_p_expr = exp((aoa - aoa_p)/delta_aoa*log_sqrt_w_over_sig1_aoa);

% negative bound
sig_aoa_m_expr = exp(-(aoa - aoa_m)/delta_aoa*log_sqrt_w_over_sig1_aoa);

% combined
sig_aoa = sig_aoa_p_expr + sig_aoa_m_expr;

% jacobian
jac_sig_aoa_exp = jacobian(sig_aoa, [gamma; theta]);

%% substitute calculations we'll have already done
sig_aoa_p = sym('sig_aoa_p','real');
jac_sig_aoa_exp = subs(jac_sig_aoa_exp , sig_aoa_p_expr, sig_aoa_p);
sig_aoa_m = sym('sig_aoa_m','real');
jac_sig_aoa_exp = subs(jac_sig_aoa_exp , sig_aoa_m_expr, sig_aoa_m);

%% get input arguments (used variables)
input_arg = sym2cell(symvar(jac_sig_aoa_exp));

if 0
%% export to m code
matlabFunction(jac_sig_aoa_exp,'File','jac_sig_aoa_exp.m','Vars',input_arg,'Outputs',{'out'});
end

if 0
%% export to c code
ccode(jac_sig_aoa_exp,'file','jac_sig_aoa_exp_ccode.c');
end

if 0
%% prep c code for mpc model functions
fid = fopen('jac_sig_aoa_exp_ccode.c');
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
fid = fopen('jac_sig_aoa_exp_ccode.c','w');
for k = 1:length(txt)
    fprintf(fid,[char(txt{k}),' \n']);
end
fclose(fid);
end

