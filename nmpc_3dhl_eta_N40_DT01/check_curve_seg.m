function bool = check_curve_seg( pos, vel, params )
    
% chi_b
chi_b = params(7) + params(5) * (params(8) + 1.570796326794897);

% Tb
Tb_n = cos(chi_b)*cos(params(6));
Tb_e = sin(chi_b)*cos(params(6));
Tb_d = -sin(params(6));

% p - b
bp_n = pos(1) - ( params(1) + params(4) * ( cos(params(7) + params(5) * params(8)) ) );
bp_e = pos(2) - ( params(2) + params(4) * ( sin(params(7) + params(5) * params(8)) ) );
bp_d = pos(3) - ( params(3) - params(4) * tan(params(6)) * params(8) );

% dot( v , Tb )
dot_vTb = vel(1) * Tb_n + vel(2) * Tb_e + vel(3) * Tb_d;

% dot( (p-b) , Tb )
dot_bpTb = bp_n * Tb_n + bp_e * Tb_e + bp_d * Tb_d;

% norm( p-b )
norm_bp = sqrt( bp_n*bp_n + bp_e*bp_e + bp_d*bp_d );

% check (1) proximity, (2) bearing, (3) travel 
bool = ( norm_bp < params(18) && dot_vTb > params(19) && dot_bpTb > 0.0 );
