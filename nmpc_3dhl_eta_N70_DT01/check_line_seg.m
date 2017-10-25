function ret = check_line_seg( pos, vel, params )
 
% tB
 tB_n = cos(params(6+1))*cos(params(5+1));
 tB_e = cos(params(6+1))*sin(params(5+1));
 tB_d = -sin(params(6+1));

% r - b
 br_n = pos(0+1) - params(1+1);
 br_e = pos(1+1) - params(2+1);
 br_d = pos(2+1) - params(3+1);

% dot( (r-b) , tB )
 dot_brtB = br_n * tB_n + br_e * tB_e + br_d * tB_d;

% check travel 
ret = ( dot_brtB > 0.0 );
