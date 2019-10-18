% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% symbolic jacobian for soft angle of attack constraint (exponential formulation)
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

clear; clc;

gamma = sym('gamma','real');
theta = sym('theta','real');
aoa_p = sym('aoa_p','real');
aoa_m = sym('aoa_m','real');
delta_aoa = sym('delta_aoa','real');

aoa_expr = theta - gamma;

% positive bound 
sig_aoa_p_expr = ((delta_aoa - (aoa_p - aoa_expr)) / delta_aoa)^3;

% negative bound
sig_aoa_m_expr = ((delta_aoa - (aoa_expr - aoa_m)) / delta_aoa)^3;

% jacobian
jac_sig_aoa_p_cubic = jacobian(sig_aoa_p_expr, [gamma; theta]);
jac_sig_aoa_m_cubic = jacobian(sig_aoa_m_expr, [gamma; theta]);

%% substitute calculations we'll have already done
sig_aoa_p = sym('sig_aoa_p','real');
jac_sig_aoa_p_cubic = subs(jac_sig_aoa_p_cubic , sig_aoa_p_expr, sig_aoa_p);
sig_aoa_m = sym('sig_aoa_m','real');
jac_sig_aoa_m_cubic = subs(jac_sig_aoa_m_cubic , sig_aoa_m_expr, sig_aoa_m);

aoa = sym('aoa','real');
jac_sig_aoa_p_cubic = subs(jac_sig_aoa_p_cubic , aoa_expr, aoa);
jac_sig_aoa_m_cubic = subs(jac_sig_aoa_m_cubic , aoa_expr, aoa);

%% get input arguments (used variables)
input_arg_p = sym2cell(symvar(jac_sig_aoa_p_cubic));
input_arg_m = sym2cell(symvar(jac_sig_aoa_m_cubic));

if 0
%% export to m code
matlabFunction(jac_sig_aoa_p_cubic,'File','jac_sig_aoa_p_cubic.m','Vars',input_arg_p,'Outputs',{'out'});
matlabFunction(jac_sig_aoa_m_cubic,'File','jac_sig_aoa_m_cubic.m','Vars',input_arg_m,'Outputs',{'out'});
end

if 0
%% export to c code
ccode(jac_sig_aoa_p_cubic,'file','jac_sig_aoa_p_cubic_ccode.c');
ccode(jac_sig_aoa_m_cubic,'file','jac_sig_aoa_m_cubic_ccode.c');
end

if 0
%% prep c code for mpc model functions
fid = fopen('jac_sig_aoa_p_cubic_ccode.c');
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
fid = fopen('jac_sig_aoa_p_cubic_ccode.c','w');
for k = 1:length(txt)
    fprintf(fid,[char(txt{k}),' \n']);
end
fclose(fid);
fid = fopen('jac_sig_aoa_m_cubic_ccode.c');
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
fid = fopen('jac_sig_aoa_m_cubic_ccode.c','w');
for k = 1:length(txt)
    fprintf(fid,[char(txt{k}),' \n']);
end
fclose(fid);
end

