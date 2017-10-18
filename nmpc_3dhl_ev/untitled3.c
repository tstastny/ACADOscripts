    // variable definitions
//     const double pparam_cc_n = in[idx_OD_0+pparam_sel+1];
//     const double pparam_cc_e = in[idx_OD_0+pparam_sel+2];
//     const double pparam_cc_d = in[idx_OD_0+pparam_sel+3];
//     const double pparam_R = fabs(in[idx_OD_0+pparam_sel+4]);
    const double pparam_ldir = (in[idx_OD_0+pparam_sel+4]<0.0) ? -1.0 : 1.0;
//     const double pparam_Chi = in[idx_OD_0+pparam_sel+5];
//     const double pparam_Gam = in[idx_OD_0+pparam_sel+6];
    double Gam_temp = in[idx_OD_0+pparam_sel+6];

    // calculate closest point on loiter circle
    const double cr_n = in[0] - in[idx_OD_0+pparam_sel+1];
    const double cr_e = in[1] - in[idx_OD_0+pparam_sel+2];
    const double norm_cr = sqrt( cr_n*cr_n + cr_e*cr_e );
    double cr_n_unit;
    double cr_e_unit;
    if (norm_cr>0.1) {
        cr_n_unit = cr_n / norm_cr;
        cr_e_unit = cr_e / norm_cr;
    }
    else {
        cr_n_unit = 0.0;
        cr_e_unit = 0.0;
    }
    p_n = fabs(in[idx_OD_0+pparam_sel+4]) * cr_n_unit + in[idx_OD_0+pparam_sel+1];
    p_e = fabs(in[idx_OD_0+pparam_sel+4]) * cr_e_unit + in[idx_OD_0+pparam_sel+2];

    // calculate tangent
    tP_n = pparam_ldir * -cr_e_unit;
    tP_e = pparam_ldir * cr_n_unit;
    
    // angular position
    const double xi_pos = atan2(cr_e_unit, cr_n_unit);
    
    // angular exit
    double xi_exit = in[idx_OD_0+pparam_sel+5] - pparam_ldir * 1.570796326794897;
    if (xi_exit>3.141592653589793) {
        xi_exit = xi_exit - 6.283185307179586;
    }
    else if (xi_exit<-3.141592653589793) {
        xi_exit = xi_exit + 6.283185307179586;
    }
    
    // angular travel (back calculated) from exit [0,2pi)
    double delta_xi = pparam_ldir * (xi_exit - xi_pos);
    if (delta_xi >= 6.28318530718) delta_xi = 0.0;
    if (delta_xi < 0.0) delta_xi =+ 6.28318530718;

    // closest point on nearest spiral leg and tangent down component
    if (fabs(in[idx_OD_0+pparam_sel+6]) < 0.001) {

        p_d = in[idx_OD_0+pparam_sel+3];
        tP_d = 0.0;

    } else {

        const double RtanGam = fabs(in[idx_OD_0+pparam_sel+4]) * tan(in[idx_OD_0+pparam_sel+6]);

        // height down from exit
        const double delta_d_xi = delta_xi * RtanGam;

        // nearest spiral leg
        const double delta_d_k = round( (in[2] - (in[idx_OD_0+pparam_sel+3] + delta_d_xi)) / (6.28318530718*RtanGam) ) * 6.28318530718*RtanGam;

        // closest point on nearest spiral leg
        p_d = in[idx_OD_0+pparam_sel+3] + delta_d_k + delta_d_xi;
        
//         // cap end point
//         if ((p_d - in[idx_OD_0+pparam_sel+3]) * in[idx_OD_0+pparam_sel+6] < 0.0) {
//             p_d = in[idx_OD_0+pparam_sel+3];
//             tP_d = 0.0;
//             Gam_temp = 0.0;
//         }
//         else {
//             tP_d = -sin(in[idx_OD_0+pparam_sel+6]);
//         }
        
    }
    
    if (fabs(tP_n)<0.01 && fabs(tP_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        tP_n=1.0;
        tP_e=0.0;
    }
    
    // Normalize tP
    tP_n = tP_n * cos(Gam_temp);
    tP_e = tP_e * cos(Gam_temp);