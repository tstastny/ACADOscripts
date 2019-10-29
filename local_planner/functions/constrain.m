function x_cstr = constrain(x, xmin, xmax)

if (x < xmin)
    x_cstr = xmin;
elseif (x > xmax)
    x_cstr = xmax;
else
    x_cstr = x;
end