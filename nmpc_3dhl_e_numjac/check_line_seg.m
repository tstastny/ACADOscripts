function bool = check_line_seg( pos, vel, params )
 
% waypoint a to b
ab_n = params(4) - params(1);
ab_e = params(5) - params(2);
ab_d = params(6) - params(3);

norm_ab = sqrt( ab_n*ab_n + ab_e*ab_e + ab_d*ab_d );

% Tb
if (norm_ab > 0.1)
    Tb_n =  ab_n / norm_ab;
    Tb_e = ab_e / norm_ab;
    Tb_d = ab_d / norm_ab;
else
    Tb_n = 1;
    Tb_e = 0;
    Tb_d = 0;
end

% p - b
bp_n = pos(1) - params(4);
bp_e = pos(2) - params(5);
bp_d = pos(3) - params(6);

% % dot( v , Tb )
% dot_vTb = vel(1) * Tb_n + vel(2) * Tb_e + vel(3) * Tb_d;

% dot( (p-b) , Tb )
dot_bpTb = bp_n * Tb_n + bp_e * Tb_e + bp_d * Tb_d;

% % norm( p-b )
% norm_bp = sqrt( bp_n*bp_n + bp_e*bp_e + bp_d*bp_d );

% check (1) proximity, (2) bearing, (3) travel 
% bool = ( (norm_bp < params(18) && dot_vTb > params(19)) || dot_bpTb > 0.0 );
bool = ( dot_bpTb > 0.0 );

