function bool = check_curve_seg( pos, vel, params )

% % Tb
% Tb_n = cos(params(7))*cos(params(6));
% Tb_e = cos(params(7))*sin(params(6));
% Tb_d = -sin(params(7));
% 
% % p - b
% if params(5)<0.0
%     rot90 = -pi/2;
% else
%     rot90 = pi/2;
% end
% bp_n = pos(1) - (params(2) + abs(params(5)) * cos(params(6) - rot90));
% bp_e = pos(2) - (params(3) + abs(params(5)) * sin(params(6) - rot90));
% bp_d = pos(3) - params(4);
% 
% % dot( v , Tb )
% dot_vTb = vel(1) * Tb_n + vel(2) * Tb_e + vel(3) * Tb_d;
% 
% % dot( (p-b) , Tb )
% dot_bpTb = bp_n * Tb_n + bp_e * Tb_e + bp_d * Tb_d;
% 
% % norm( p-b )
% norm_bp = sqrt( bp_n*bp_n + bp_e*bp_e + bp_d*bp_d );
% 
% % check (1) proximity, (2) bearing, (3) travel 
% bool = ( norm_bp < params(15) && dot_vTb > params(16) && dot_bpTb > 0.0 );

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
br_n = pos(0+1) - (params(1+1) + abs(params(4+1)) * cos(params(5+1) - rot90));
br_e = pos(1+1) - (params(2+1) + abs(params(4+1)) * sin(params(5+1) - rot90));
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
bool = ( norm_br < params(14+1) && norm_br_d < 10.0 && dot_vtB > params(15+1)*sqrt(vel(0+1)*vel(0+1)+vel(1+1)*vel(1+1)) && dot_brtB > 0.0 );