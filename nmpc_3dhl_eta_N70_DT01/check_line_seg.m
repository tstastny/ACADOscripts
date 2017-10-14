function bool = check_line_seg( pos, vel, params )
 
% Tb
Tb_n = cos(params(7))*cos(params(6));
Tb_e = cos(params(7))*sin(params(6));
Tb_d = -sin(params(7));

% p - b
bp_n = pos(1) - params(2);
bp_e = pos(2) - params(3);
bp_d = pos(3) - params(4);

% % dot( v , Tb )
% dot_vTb = vel(1) * Tb_n + vel(2) * Tb_e + vel(3) * Tb_d;

% dot( (p-b) , Tb )
dot_bpTb = bp_n * Tb_n + bp_e * Tb_e + bp_d * Tb_d;

% % norm( p-b )
% norm_bp = sqrt( bp_n*bp_n + bp_e*bp_e + bp_d*bp_d );

% check (1) proximity, (2) bearing, (3) travel 
% bool = ( (norm_bp < params(18) && dot_vTb > params(19)) || dot_bpTb > 0.0 );
bool = ( dot_bpTb > 0.0 );

