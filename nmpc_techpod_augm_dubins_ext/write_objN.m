% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% obj
% ///

% /////////////////////////////////////////////////////////////////////////
% objective evaluation function
% ///

% print rhs evaluation definition
f_lines{k} = 'void lsq_objN_eval( real_t *in, real_t *out ){';
k = k + 1;
f_lines{k} = '';
k = k + 1;

% open obj c file
fid = fopen('ext_lsq_obj_N_.c');
tline = fgets(fid);

% store starting index
lsq_obj_eval_st = k;

% print optimized intermediate calculations
f_lines{k} = '/* optimized intermediate calculations */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

processing_ts = true;
while processing_ts
    if ischar(tline)
        c0 = textscan(tline,'%1s');
        if strcmp(char(c0{1}(1)),'t')                                      % check if first character is a "t"
            c1 = textscan(tline,'%s','Delimiter','\n');                    % read in line
            c2 = char(c1{1});                                              % take first string
            c3 = regexprep(c2,char_ins,char_ins1);                         % fix ins
            f_lines{k} = ['const double ',c3];                             % define as const double
            k = k + 1;
            tline = fgets(fid);                                            % get next line
        else
            processing_ts = false;
        end
    else
        processing_ts = false;
    end
end     
f_lines{k} = '';
k = k + 1;

% print tracked expressions
if idx_tracked_expr > 0

f_lines{k} = '/* tracked expressions */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

i = 0;
while i < idx_tracked_expr

    % look for 1, 2, 3 digit indicies
    if i<10, ii = 1;
    elseif i<100, ii = 2;
    else ii = 3;
    end

    if ischar(tline)
        formatSpec0 = ['%*3s %',int2str(ii),'d'];                          % skip characters before first index
        c0 = textscan(tline,formatSpec0);                                  % read the index

        if c0{1} ~= i
            % if the index does not match
            f_lines{k} = 'ERROR: tracked expr index does not match';
            k = k + 1;
            disp('PRINT TRACKED EXPR: index does not match, something is wrong.')
            break;
        else
            str_expr = char(tracked_expr(i+1,1));
            if strcmp(str_expr(1:5), 'atan2')                               % tracked expression is an atan2
                % print the expression
                formatSpec1 = '%s';
                c1 = textscan(tline,formatSpec1,'Delimiter',' ');              % delimit with empty space
                c2 = char(c1{1}(end));                                         % take last string
                c3_1 = regexprep(c2,char_ins,char_ins1);                         % fix ins

                tline = fgets(fid);                                         % get next line
                formatSpec1 = '%s';
                c1 = textscan(tline,formatSpec1,'Delimiter',' ');              % delimit with empty space
                c2 = char(c1{1}(end));                                         % take last string
                c3_2 = regexprep(c2,char_ins,char_ins1);                         % fix ins

                f_lines{k} = ['const double ', str_expr, ...
                    ' = atan2(', c3_1(1:end-1), ', ', c3_2(1:end-1), ');'];
                k = k + 1;
                i = i + 2;
                tline = fgets(fid);  
            else
                % print the expression
                formatSpec1 = '%s';
                c1 = textscan(tline,formatSpec1,'Delimiter',' ');              % delimit with empty space
                c2 = char(c1{1}(end));                                         % take last string
                c3 = regexprep(c2,char_ins,char_ins1);                         % fix ins
                f_lines{k} = ['const double ', str_expr, ' = ', c3];
                k = k + 1;
                i = i + 1;
                tline = fgets(fid);  
            end
        end
    end

end
f_lines{k} = '';
k = k + 1;
end

% print outputs
f_lines{k} = '/* outputs */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

kk = 1;
for i = idx_tracked_expr:idx_tracked_expr+n_Y-1

    % look for 1, 2, 3 digit indicies
    if i<10, ii = 1;
    elseif i<100, ii = 2;
    else ii = 3;
    end

    if ischar(tline)
        formatSpec0 = ['%*3s %',int2str(ii),'d'];                          % skip characters before first index
        c0 = textscan(tline,formatSpec0);                                  % read the index

        if c0{1} ~= i
            % if the index does not match, mark as zero, and continue
            f_lines{k} = ['out[',int2str(kk-1),'] = 0.0;'];
            k = k + 1;
            kk = kk + 1;
        else
            % print the expression
            formatSpec1 = '%s';
            c1 = textscan(tline,formatSpec1,'Delimiter',' ');              % delimit with empty space
            c2 = char(c1{1}(end));                                         % take last string
            c3 = regexprep(c2,char_ins,char_ins1);                         % fix ins
            f_lines{k} = ['out[',int2str(kk-1),'] = ',c3];   
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

% store ending index
lsq_obj_eval_end = k;

f_lines{k} = '';
k = k + 1;
f_lines{k} = '}';
k = k + 1;
f_lines{k} = '';
k = k + 1;

% /////////////////////////////////////////////////////////////////////////
% obj with numerical jacobians
% ///

% print lsq definition
f_lines{k} = 'void evaluateLSQEndTerm( const real_t *in, real_t *out ){';
k = k + 1;
f_lines{k} = '';
k = k + 1;

% evaluate objective outputs
eval_obj_lines = {'double in_Delta[ACADO_NX+ACADO_NU+ACADO_NOD];', ...
'memcpy(in_Delta, in, sizeof(in_Delta));', ...
'lsq_objN_eval( in_Delta, out );', ...
' ', ...
'/* lsq_objN jacobians */', ...
' ', ...
'double f_Delta_m[ACADO_NYN];', ...
'double f_Delta_p[ACADO_NYN];', ...
'const double Delta = 0.00001;', ...
'const double Delta2 = 2.0 * Delta;', ...
' ', ...
'int i;', ...
'int j;', ...
'for (i = 0; i < ACADO_NX; i=i+1) {', ...
' ', ...
'    in_Delta[i] = in[i] - Delta;', ...
'    lsq_objN_eval( in_Delta, f_Delta_m );', ...
'    in_Delta[i] = in[i] + Delta;', ...
'    lsq_objN_eval( in_Delta, f_Delta_p );', ...
'    in_Delta[i] = in[i];', ...
' ', ...
'    for (j = 0; j < ACADO_NYN; j=j+1) {', ...
'        out[ACADO_NYN+j*ACADO_NX+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;', ...
'    }', ...
' ', ...
'}'};

f_lines = {f_lines{:}, eval_obj_lines{:}};
k = length(f_lines) + 1;

f_lines{k} = '';
k = k + 1;
f_lines{k} = '}';
k = k + 1;
f_lines{k} = '';
k = k + 1;
