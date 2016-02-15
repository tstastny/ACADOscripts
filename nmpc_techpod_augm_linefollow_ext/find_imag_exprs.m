
expr = jjj;

imag_exprs = sym('imag_exprs');
idx_imag = 1;
str = char(expr);
lstr = length(str);
i = 1;
while i<lstr-4;
    if strcmp(str(i:i+3), 'imag')
        pcount = 1;
        j = i+6;
        while pcount>0 || j<=lstr
            if strcmp(str(j), '(')
                pcount = pcount + 1;
            elseif strcmp(str(j), ')')
                pcount = pcount - 1;
            end
            if pcount == 0
                imag_end = j;
                imag_exprs(idx_imag) = eval(str(i+6:j-1));
                idx_imag = idx_imag + 1;
                i = j;
            end
        end
    end
    i = i + 1;
end
                