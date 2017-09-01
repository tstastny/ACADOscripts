
% NOTE: does NOT account for atan2's inside of atan2's.

for idx_FaR = 1:length(FaR_expr)

str = char(FaR_expr(idx_FaR));
lstr = length(str);
i = 1;
while i < lstr-10                                                           % length(char(atan2(y,x))) = 10 -- min length
    if strcmp(str(i:i+4), 'atan2')                                          % check for atan2 str match
        idx_atan2_st = i;                                                   % record start index of atan2
        pcount = 1;                                                         % initalize parenthesis count
        j = idx_atan2_st+6;                                                 % initialize atan2 scan
        failed = true;
        while pcount > 0 && j <= lstr
            if strcmp(str(j), '(')
                pcount = pcount + 1;
            end
            if strcmp(str(j), ')')
                pcount = pcount - 1;
            end
            if strcmp(str(j), ',')
                idx_comma = j;
            end
            if pcount == 0
                idx_atan2_end = j;
                failed = false;
            end
            j = j + 1;
        end
        if failed
            disp('FAIL: found no end to atan2')
            break
        end
        % store atan2 arguments in symbolic form
        atan2_args(idx_atan2s, 1) = eval(str(idx_atan2_st+6:idx_comma-1));
        atan2_args(idx_atan2s, 2) = eval(str(idx_comma+1:idx_atan2_end-1));
        % replace atan2s with dummy placeholder in frhs
        if idx_atan2s < 10
            str_atan2 = ['atan2_0',int2str(idx_atan2s)];
        else
            str_atan2 = ['atan2_',int2str(idx_atan2s)];
        end
        str = [str(1:idx_atan2_st-1),str_atan2,str(idx_atan2_end+1:end)];
        syms(str_atan2);
        atan2s(idx_atan2s) = sym(str_atan2);
        % find new length of str and continue search
        lstr = length(str);
        i = idx_atan2_st + 8;
        idx_atan2s = idx_atan2s + 1;
    else
        i = i + 1;
    end
end

FaR_expr(idx_FaR) = eval(str);
end
