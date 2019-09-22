% clear; clc;

% help speed up the c -> m process for the eval_obj.m function
% >> call this from the root folder

n_in = 31; % set this to IDX_TERR_DATA
n_out = 11;

fid = fopen('misc/c_snippet.m');
txt = textscan(fid,'%s','delimiter','\n'); 
fclose(fid);
txt = txt{1};

for i = 1:length(txt)
    
    str1 = txt{i};
    
    % comments        
    idx = strfind(txt{i},'//');
    if ~isempty(idx)
        str1 = [txt{i}(1:idx-1),'%%',txt{i}(idx+2:end)];
        txt{i} = str1;
    end
    
    % more comments        
    idx = strfind(txt{i},'/*');
    if ~isempty(idx)
        str1 = [txt{i}(1:idx-1),'%%',txt{i}(idx+2:end)];
        txt{i} = str1;
    end
    
    % const double        
    idx = strfind(txt{i},'const double ');
    if ~isempty(idx)
        str1 = [txt{i}(1:idx-1),txt{i}(idx+13:end)];
        txt{i} = str1;
    end
    
    % double        
    idx = strfind(txt{i},'double ');
    if ~isempty(idx)
        str1 = [txt{i}(1:idx-1),txt{i}(idx+7:end)];
        txt{i} = str1;
    end
       
    % int        
    idx = strfind(txt{i},'int ');
    if ~isempty(idx)
        str1 = [txt{i}(1:idx-1),txt{i}(idx+4:end)];
        txt{i} = str1;
    end
    
    % in's
    for i_in = n_in-1:-1:0
        in_str = ['in[',int2str(i_in),']'];
        idx = strfind(txt{i},in_str);
        if ~isempty(idx)
            if i_out>9
                ii=2;
            else
                ii=1;
            end
            str1 = [txt{i}(1:idx-1),'in(',int2str(i_in+1),')',txt{i}(idx+length(in_str):end)];
            txt{i} = str1;
        end
    end
    
    % out's
    for i_out = n_out-1:-1:0
        out_str = ['out[',int2str(i_out),']'];
        idx = strfind(txt{i},out_str);
        if ~isempty(idx)
            if i_out>9
                ii=2;
            else
                ii=1;
            end
            str1 = [txt{i}(1:idx-1),'out(',int2str(i_out+1),')',txt{i}(idx+length(out_str):end)];
            txt{i} = str1;
        end
    end
    
end

fid = fopen('misc/m_snippet.m','w');
for k = 1:length(txt)
    fprintf(fid,[char(txt{k}),' \n']);
end
fclose(fid);
