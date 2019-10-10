function [x_occ,y_occ,occ_detected] = castray(x0, y0, x1, y1, h, terr_mat, output_everything, check_occ)

% total x/y distance
dx = abs(x1 - x0);
dy = abs(y1 - y0);

% initialize ray trace
x = x0;
y = y0;
n = 1 + dx + dy;
if (x1 > x0)
    x_inc =  1;
else
    x_inc = -1;
end
if (y1 > y0)
    y_inc =  1;
else
    y_inc = -1;
end
error = dx - dy;
dx = dx * 2;
dy = dy * 2;

kk = 1;
occ_detected = false;
for ii = 1:n
    
    if x<0
        x_check = 0;
    elseif x>size(terr_mat,2)-1
        x_check = size(terr_mat,2)-1;
    else
        x_check = x;
    end
    if y<0
        y_check = 0;
    elseif y>size(terr_mat,1)-1
        y_check = size(terr_mat,1)-1;
    else
        y_check = y;
    end;

    if (terr_mat(y_check+1,x_check+1) > h) && kk>1 && check_occ % we assume we are not already in an invalid cell.. otherwise we're dead anyway
        x_occ(kk) = x;
        y_occ(kk) = y;
        occ_detected = true;
        return;
    elseif output_everything
        x_occ(kk) = x;
        y_occ(kk) = y;
        kk = kk + 1;
    end
    
    if (error > 0)
        x = x + x_inc;
        error = error - dy;
    else
        y = y + y_inc;
        error = error + dx;
    end
end
