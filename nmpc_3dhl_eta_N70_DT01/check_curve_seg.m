function ret = check_curve_seg( pos, vel, params )

% tB
tB_n = cos(params(6+1))*cos(params(5+1));
tB_e = cos(params(6+1))*sin(params(5+1));
tB_d = -sin(params(6+1));

% r - b
if (params(4+1)<0.0)
    rot90 = -1.570796326794897;
else
    rot90 = 1.570796326794897;
end
br_n = pos(0+1) - (params(1+1) + fabs(params(4+1)) * cos(params(5+1) - rot90));
br_e = pos(1+1) - (params(2+1) + fabs(params(4+1)) * sin(params(5+1) - rot90));
br_d = pos(2+1) - params(3+1);

% bearing : dot( v , tB ) (lat)
dot_vtB = vel(0+1) * tB_n + vel(1+1) * tB_e;

% travel : dot( (r-b) , tB ) (lat)
dot_brtB = br_n * tB_n + br_e * tB_e;

% proximity : norm( r-b ) (lat)
norm_br = sqrt( br_n*br_n + br_e*br_e );

% proximity : norm( r-b ) (lon)
norm_br_d = abs(br_d);

% check (1) proximity, (2) bearing, (3) travel 
ret = ( norm_br < params(14+1) && norm_br_d < 10.0 && dot_vtB > params(15+1)*sqrt(vel(0+1)*vel(0+1)+vel(1+1)*vel(1+1)) && dot_brtB > 0.0 );