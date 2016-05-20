function bool = check_curve_seg( pos, pparams )
    
    b_n = pparams(1) + pparams(4) * ( cos(pparams(7) + pparams(5) * pparams(8)) );
    b_e = pparams(2) + pparams(4) * ( sin(pparams(7) + pparams(5) * pparams(8)) );
    b_d = pparams(3) - pparams(4) * tan(pparams(6)) * pparams(8);

    Gamma_b = pparams(7) + pparams(5) * (pparams(8) + 1.570796326794897);

    Tb_n = cos(Gamma_b);
    Tb_e = sin(Gamma_b);
    Tb_d = -sin(pparams(6));

    % 1-D track position %TODO: not hard-coded acceptance radius
    pb_t = 1.0 - ( Tb_n*(pos(1)-b_n+Tb_n) + Tb_e*(pos(2)-b_e+Tb_e) + Tb_d*(pos(3)-b_d+Tb_d) );

    % check
    bool  = ( (pb_t - 25.0) < 0.0 && abs(b_d - pos(3)) < 10.0 );