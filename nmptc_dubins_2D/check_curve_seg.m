function bool = check_curve_seg( pos, pparams, n_dot, e_dot )
    
    % end point
    b_n = pparams(1) + pparams(4) * ( cos(pparams(7) + pparams(5) * pparams(8)) );
    b_e = pparams(2) + pparams(4) * ( sin(pparams(7) + pparams(5) * pparams(8)) );

    % end point tangent
    chi_b = pparams(7) + pparams(5) * (pparams(8) + 1.570796326794897);

    Tb_n = cos(chi_b);
    Tb_e = sin(chi_b);

    % dot product
    V_g = sqrt(n_dot*n_dot + e_dot*e_dot);
    if ( V_g < 0.01 ), V_g = 0.01; end
    
    dot_T_V = (Tb_n*n_dot + Tb_e*e_dot)/V_g;

    % 1-D track position
    bp_n = pos(1)-b_n;
    bp_e = pos(2)-b_e;
    pb_t = 1.0 - ( Tb_n*(bp_n+Tb_n) + Tb_e*(bp_e+Tb_e) );
        
    % check //TODO: not hard-coded acceptance radius
    bool = ( pb_t < 0.0 && dot_T_V > 0.8 && sqrt(bp_n*bp_n+bp_e*bp_e) < 25.0 );