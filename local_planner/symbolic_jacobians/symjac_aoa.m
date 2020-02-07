% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% symbolic jacobian for angle of attack
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

clear; clc;

gamma = sym('gamma','real');
theta = sym('theta','real');
aoa_p = sym('aoa_p','real');
aoa_m = sym('aoa_m','real');
delta_aoa = sym('delta_aoa','real');
log_sqrt_w_over_sig1_aoa = sym('log_sqrt_w_over_sig1_aoa','real');

aoa_expr = theta - gamma;

% jacobian
jac_aoa = jacobian(aoa_expr, [gamma; theta]);

%% substitute calculations we'll have already done
% aoa = sym('aoa','real');
% jac_aoa = subs(jac_aoa , aoa_expr, aoa);

%% get input arguments (used variables)
doit = true;

strs = {'aoa'};
ss = 1;

input_arg = sym2cell(symvar(jac_aoa));

if doit
%% export to m code
matlabFunction(jac_aoa,'File',['jac_',strs{ss},'.m'],'Vars',input_arg,'Outputs',{'out'});
end

if doit
%% export to c code
ccode(jac_aoa,'file',['jac_',strs{ss},'_ccode.c']);
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
if length(input_arg) == 0
    txt_top{1} = ['void jacobian_',strs{ss},'(double *jac)'];
    txt_top{2} = '{\n';
    header_len = 2;
else
    txt_top{1} = ['void jacobian_',strs{ss},'(double *jac,'];
    txt_top{2} = ['const double ', char(input_arg{1})];
    for k = 2:length(input_arg)
        txt_top{2} = [txt_top{2}, ', const double ', char(input_arg{k})];
    end
    txt_top{2} = [txt_top{2}, ')\n'];
    txt_top{3} = '{\n';
    header_len = 3;
end
txt_top{header_len+1} = '/* w.r.t.:';
txt_top{header_len+2} = '    theta';
txt_top{header_len+3} = '    gamma';
txt_top{header_len+4} = '*/\n';
fid = fopen(['jac_',strs{ss},'_ccode.c'],'w');
for k = 1:length(txt_top)
    fprintf(fid,[char(txt_top{k}),'\n']);
end
for k = 1:length(txt)
    fprintf(fid,[char(txt{k}),'\n']);
end
fprintf(fid,[char('}'),'\n']);
fclose(fid);
end

