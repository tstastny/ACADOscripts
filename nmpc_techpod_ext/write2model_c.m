% /////////////////////////////////////////////////////////////////////////
% INSTRUCTIONS FOR USE ////////////////////////////////////////////////////
%
% run sym2matfun.m in cell 1 to generate f_ and J_ c files
% select all in each resp. c file and remove indent (SHIFT+TAB)
% run modifytxt.m
% /////////////////////////////////////////////////////////////////////////
clear;
clc;

% generate f and J c files
sym2matfun

k = 1;

% print includes
f_lines{k} = '#include "acado_common.h"';
k = k + 1;
f_lines{k} = '';
k = k + 1;

% ////////// rhs //////////////////////////////////////////////////////////

% print rhs definition
f_lines{k} = 'void rhs( const real_t *in, real_t *out ){';
k = k + 1;
f_lines{k} = '';
k = k + 1;

% print states
f_lines{k} = '/* states */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

for i = 1:n_X
    state_i = char(states(i));
    if strcmp(state_i,'theta') || strcmp(state_i,'psi')
        state_i = [state_i,'_Var'];
    end
    f_lines{k} = ['const double ',state_i,' = in[',int2str(i-1),'];'];
    k = k + 1;
end
f_lines{k} = '';
k = k + 1;

% print controls
f_lines{k} = '/* controls */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

for i = 1:n_U
    ctrl_i = char(ctrls(i));
    if strcmp(ctrl_i,'theta') || strcmp(ctrl_i,'psi')
        ctrl_i = [ctrl_i,'_Var'];
    end
    f_lines{k} = ['double ',ctrl_i,' = in[',int2str(n_X+i-1),'];'];
    k = k + 1;
end
f_lines{k} = '';
k = k + 1;

% print control constraints
f_lines{k} = '/* control constraints */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

f_lines{k} = 'const double d2r = 3.14159265359/180.0;';
k = k + 1;
f_lines{k} = '';
k = k + 1;

f_lines{k} = 'if (uT > 1.0) uT = 1.0;';
k = k + 1;
f_lines{k} = 'if (uT < 0.1) uT = 0.1;';
k = k + 1;
f_lines{k} = 'if (uE > 20.0 * d2r) uE = 20.0 * d2r;';
k = k + 1;
f_lines{k} = 'if (uE < -20.0 * d2r) uE = -20.0 * d2r;';
k = k + 1;
f_lines{k} = 'if (uA > 20.0 * d2r) uA = 20.0 * d2r;';
k = k + 1;
f_lines{k} = 'if (uA < -20.0 * d2r) uA = -20.0 * d2r;';
k = k + 1;
f_lines{k} = 'if (uR > 20.0 * d2r) uR = 20.0 * d2r;';
k = k + 1;
f_lines{k} = 'if (uR < -20.0 * d2r) uR = -20.0 * d2r;';
k = k + 1;
f_lines{k} = '';
k = k + 1;

% print parameters
f_lines{k} = '/* parameters */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

for i = 1:n_P
    params_i = char(params(i));
    if strcmp(params_i,'theta') || strcmp(params_i,'psi')
        params_i = [params_i,'_Var'];
    end
    f_lines{k} = ['const double ',params_i,' = in[',int2str(n_X+n_U+i-1),'];'];
    k = k + 1;
end
f_lines{k} = '';
k = k + 1;

% open f_ c file
fid = fopen('f_.c');
tline = fgets(fid);

% print optimized intermediate calculations
f_lines{k} = '/* optimized intermediate calculations */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

processing_ts = true;
while processing_ts
    if ischar(tline)
        c0 = textscan(tline,'%1s');
        if strcmp(char(c0{1}(1)),'t')
            c1 = textscan(tline,'%s','Delimiter','\n');
            f_lines{k} = ['const double ',char(c1{1})];
            k = k + 1;
            tline = fgets(fid);
        else
            processing_ts = false;
        end
    else
        processing_ts = false;
    end
end     
f_lines{k} = '';
k = k + 1;

% print rhs
f_lines{k} = '/* rhs */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

kk = 1;
for i = 0:(n_X-1)

    if i<10, ii = 1;
    else ii = 2;
    end

    if ischar(tline)
        formatSpec0 = ['%*6s %',int2str(ii),'d'];
        c0 = textscan(tline,formatSpec0);

        if c0{1} ~= i
            f_lines{k} = ['out[',int2str(kk-1),'] = 0.0;'];
            k = k + 1;
            kk = kk + 1;
        else
            formatSpec1 = ['%*s %s %s %s %s'];
            c1 = textscan(tline,formatSpec1,'Delimiter',' ');
            f_lines{k} = ['out[',int2str(kk-1),'] = ',char(c1{4})];
            tline = fgets(fid);
            k = k + 1;
            kk = kk + 1;
        end
    else
        f_lines{k} = ['out[',int2str(kk-1),'] = 0.0;'];
        k = k + 1;
        kk = kk + 1;
    end

end
fclose(fid);

f_lines{k} = '';
k = k + 1;
f_lines{k} = '}';
k = k + 1;
f_lines{k} = '';
k = k + 1;

% ////////// rhs_jac //////////////////////////////////////////////////////

% print rhs_jac definition
f_lines{k} = 'void rhs_jac( const real_t *in, real_t *out ){';
k = k + 1;
f_lines{k} = '';
k = k + 1;

% print states
f_lines{k} = '/* states */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

for i = 1:n_X
    state_i = char(states(i));
    if strcmp(state_i,'theta') || strcmp(state_i,'psi')
        state_i = [state_i,'_Var'];
    end
    f_lines{k} = ['const double ',state_i,' = in[',int2str(i-1),'];'];
    k = k + 1;
end
f_lines{k} = '';
k = k + 1;

% print controls
f_lines{k} = '/* controls */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

for i = 1:n_U
    ctrl_i = char(ctrls(i));
    if strcmp(ctrl_i,'theta') || strcmp(ctrl_i,'psi')
        ctrl_i = [ctrl_i,'_Var'];
    end
    f_lines{k} = ['double ',ctrl_i,' = in[',int2str(n_X+i-1),'];'];
    k = k + 1;
end
f_lines{k} = '';
k = k + 1;

% print control constraints
f_lines{k} = '/* control constraints */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

f_lines{k} = 'const double d2r = 3.14159265359/180.0;';
k = k + 1;
f_lines{k} = '';
k = k + 1;

f_lines{k} = 'if (uT > 1.0) uT = 1.0;';
k = k + 1;
f_lines{k} = 'if (uT < 0.1) uT = 0.1;';
k = k + 1;
f_lines{k} = 'if (uE > 20.0 * d2r) uE = 20.0 * d2r;';
k = k + 1;
f_lines{k} = 'if (uE < -20.0 * d2r) uE = -20.0 * d2r;';
k = k + 1;
f_lines{k} = 'if (uA > 20.0 * d2r) uA = 20.0 * d2r;';
k = k + 1;
f_lines{k} = 'if (uA < -20.0 * d2r) uA = -20.0 * d2r;';
k = k + 1;
f_lines{k} = 'if (uR > 20.0 * d2r) uR = 20.0 * d2r;';
k = k + 1;
f_lines{k} = 'if (uR < -20.0 * d2r) uR = -20.0 * d2r;';
k = k + 1;
f_lines{k} = '';
k = k + 1;

% print parameters
f_lines{k} = '/* parameters */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

for i = 1:n_P
    params_i = char(params(i));
    if strcmp(params_i,'theta') || strcmp(params_i,'psi')
        params_i = [params_i,'_Var'];
    end
    f_lines{k} = ['const double ',params_i,' = in[',int2str(n_X+n_U+i-1),'];'];
    k = k + 1;
end
f_lines{k} = '';
k = k + 1;

% open J_ c file
fid = fopen('J_.c');
tline = fgets(fid);

% print optimized intermediate calculations
f_lines{k} = '/* optimized intermediate calculations */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

processing_ts = true;
while processing_ts
    if ischar(tline)
        c0 = textscan(tline,'%1s');
        if strcmp(char(c0{1}(1)),'t')
            c1 = textscan(tline,'%s','Delimiter','\n');
            f_lines{k} = ['const double ',char(c1{1})];
            k = k + 1;
            tline = fgets(fid);
        else
            processing_ts = false;
        end
    else
        processing_ts = false;
    end
end     
f_lines{k} = '';
k = k + 1;

% print jacobian
f_lines{k} = '/* jacobian */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

kk = 1;
for i = 0:(n_X-1)
    for j = 0:(n_X+n_U-1)
        
        if i==14
            stoppp=1;
        end
        
        if i<10, ii = 1;
        else ii = 2;
        end
        if j<10, jj = 1;
        else jj = 2;
        end
       
        if ischar(tline)
            formatSpec0 = ['%s %d %d %s'];
            c0 = textscan(tline,formatSpec0,'Delimiter',{']','][','['});

            if c0{2} ~= i || c0{3} ~= j
                f_lines{k} = ['out[',int2str(kk-1),'] = 0.0;'];
                k = k + 1;
                kk = kk + 1;
            else
                formatSpec1 = ['%*s %s %s %s %s'];
                c1 = textscan(tline,formatSpec1,'Delimiter',' ');
                f_lines{k} = ['out[',int2str(kk-1),'] = ',char(c1{4})];
                tline = fgets(fid);
                k = k + 1;
                kk = kk + 1;
            end
        else
            f_lines{k} = ['out[',int2str(kk-1),'] = 0.0;'];
            k = k + 1;
            kk = kk + 1;
        end
        
    end
end
fclose(fid);

f_lines{k} = '';
k = k + 1;
f_lines{k} = '}';
k = k + 1;

% ////////// write to file ////////////////////////////////////////////////

fid = fopen('export_nmpc_ext/model.c','w');
for k = 1:length(f_lines)
    fprintf(fid,[char(f_lines{k}),'\n']);
end
fclose(fid);


