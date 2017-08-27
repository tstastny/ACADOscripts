% /////////////////////////////////////////////////////////////////////////
% INSTRUCTIONS FOR USE
%
% WRITE SOMETHING HERE.
% /////////////////////////////////////////////////////////////////////////
close all;
clear all;
clc;

% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% setup
% /////

ext_mod = true;
ext_obj = true;

num_jac = true;                                                             % numerical jacobian
use_atan2 = true;
use_bool = true;

% generate optimized c code for each external function
sym2ccode

% define original/replacement strings
for i = 1:(n_X+n_U+n_OD)
    if i>10                                                                % double digits
        char_ins{i} = ['in',int2str(i-1)];
        char_ins1{i} = ['in[',int2str(i-1),']'];
    else                                                                   % single digits
        char_ins{i} = ['in0',int2str(i-1)];
        char_ins1{i} = ['in[',int2str(i-1),']'];
    end
end

k = 1;

% print includes
f_lines{k} = '#include "acado_common.h"';
k = k + 1;
if use_atan2
    f_lines{k} = '#include <math.h>';
    k = k + 1;
end
if num_jac
    f_lines{k} = '#include <string.h>';
    k = k + 1;
end
if use_bool
    f_lines{k} = '#include <stdbool.h>';
    k = k + 1;
end
f_lines{k} = '';
k = k + 1;

% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% functions
% /////////

if ext_mod
    write_rhs
    write_rhs_jac
end

if ext_obj
    write_obj
    write_objN
end

% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% write to file
% /////////////

fid = fopen('export_nmpc_ext/model.c','w');
for k = 1:length(f_lines)
    fprintf(fid,[char(f_lines{k}),'\n']);
end
fclose(fid);

disp('Done.');
