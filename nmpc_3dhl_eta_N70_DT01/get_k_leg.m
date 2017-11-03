function k_leg = get_k_leg(in,Ns)

ACADO_NX = Ns(1);
ACADO_NU = Ns(2);
minus_NU = 0;
idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
pparam_sel=0;

% variable definitions
if (in(idx_OD_0+pparam_sel+4+1)<0.0)
    pparam_ldir = -1.0;
else
    pparam_ldir = 1.0;
end
Gam_temp = in(idx_OD_0+pparam_sel+6+1);

% calculate closest poon loiter circle
cr_n = in(0+1) - in(idx_OD_0+pparam_sel+1+1);
cr_e = in(1+1) - in(idx_OD_0+pparam_sel+2+1);
norm_cr = sqrt( cr_n*cr_n + cr_e*cr_e );
if (norm_cr>0.1) 
    cr_n_unit = cr_n / norm_cr;
    cr_e_unit = cr_e / norm_cr;
else 
    cr_n_unit = 0.0;
    cr_e_unit = 0.0;
end
p_n = abs(in(idx_OD_0+pparam_sel+4+1)) * cr_n_unit + in(idx_OD_0+pparam_sel+1+1);
p_e = abs(in(idx_OD_0+pparam_sel+4+1)) * cr_e_unit + in(idx_OD_0+pparam_sel+2+1);

% calculate tangent
tP_n = pparam_ldir * -cr_e_unit;
tP_e = pparam_ldir * cr_n_unit;

% angular position
xi_pos = atan2(cr_e_unit, cr_n_unit);

% angular exit
xi_exit = in(idx_OD_0+pparam_sel+5+1) - pparam_ldir * 1.570796326794897;
if (xi_exit>3.141592653589793) 
    xi_exit = xi_exit - 6.283185307179586;
elseif (xi_exit<-3.141592653589793) 
    xi_exit = xi_exit + 6.283185307179586;
end

% angular travel (back calculated) from exit (0,2pi)
delta_xi = pparam_ldir * (xi_exit - xi_pos);
if (delta_xi >= 6.28318530718), delta_xi = 0.0; end;
if (delta_xi < 0.0), delta_xi = delta_xi + 6.28318530718; end;

% closest poon nearest spiral leg and tangent down component
if (abs(in(idx_OD_0+pparam_sel+6+1)) < 0.001) 

    p_d = in(idx_OD_0+pparam_sel+3+1);
    tP_d = 0.0;
    
    k_leg = 0;

else 

    RtanGam = abs(in(idx_OD_0+pparam_sel+4+1)) * tan(in(idx_OD_0+pparam_sel+6+1));

    % height down from exit
    delta_d_xi = delta_xi * RtanGam;

    % nearest spiral leg
    k_leg = round( (in(2+1) - (in(idx_OD_0+pparam_sel+3+1) + delta_d_xi)) / abs(6.28318530718*RtanGam) );
    
end