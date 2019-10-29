function [x_occ,y_occ,occ_found] = bresenham(x0, y0, x1, y1, h, terr_mat, output_everything)

len_x = int32(x1 - x0);
len_y = int32(y1 - y0);
x = int32(x0);
y = int32(y0);

dx0 = int32(0);
dy0 = int32(0);
dx1 = int32(0);
dy1 = int32(0);

if (len_x<0)
    dx0 = -1;
elseif (len_x>0)
    dx0 = 1;
end

if (len_y<0)
    dy0 = -1;
elseif (len_y>0)
    dy0 = 1;
end

if (len_x<0)
    dx1 = -1;
elseif (len_x>0)
    dx1 = 1;
end

longest = abs(len_x);
shortest = abs(len_y);
if (~(longest > shortest))
    
    longest = abs(len_y);
    shortest = abs(len_x);
    
    if (len_y<0)
        dy1 = -1;
    elseif (len_y>0)
        dy1 = 1;
    end
    
    dx1 = 0;
end

kk = 1;
occ_found = false;

numerator = bitshift(longest,1);
for i=0:longest
    
    if (terr_mat(x,y) > h)
        x_occ(kk) = x;
        y_occ(kk) = y;
        occ_found = true;
        return;
    elseif output_everything
        x_occ(kk) = x;
        y_occ(kk) = y;
        kk = kk + 1;
    end
    
    numerator = numerator + shortest;
    if (~(numerator < longest))
        numerator = numerator - longest;
        x = x + dx0;
        y = y + dy0;
    else
        x = x + dx1;
        y = y + dy1;
    end

end