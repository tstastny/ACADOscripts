% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////
% rhs_jac
% ///////

% print rhs definition
f_lines{k} = 'void rhs_jac( const real_t *in, real_t *out ){';
k = k + 1;
f_lines{k} = '';
k = k + 1;

% open rhs c file
fid = fopen('ext_Jrhs_.c');
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

% print rhs
f_lines{k} = '/* rhs_jac */';
k = k + 1;
f_lines{k} = '';
k = k + 1;

kk = 1;
for i = 0:( (n_X+n_U)*n_X - 1 )

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

f_lines{k} = '';
k = k + 1;
f_lines{k} = '}';
k = k + 1;
f_lines{k} = '';
k = k + 1;
