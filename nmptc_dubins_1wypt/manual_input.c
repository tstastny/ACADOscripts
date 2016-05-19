/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

double d_n;
double d_e;
double d_d;
double Td_n;
double Td_e;
double Td_d;
double Gamma_d;

const double pparam_type = in[11];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[12];
    const double pparam_aa_e = in[13];
    const double pparam_aa_d = in[14];
    const double pparam_bb_n = in[15];
    const double pparam_bb_e = in[16];
    const double pparam_bb_d = in[17];

    // calculate vector from waypoint a to b
    const double abn = pparam_bb_n - pparam_aa_n;
    const double abe = pparam_bb_e - pparam_aa_e;
    const double abd = pparam_bb_d - pparam_aa_d;
    const double norm_ab = sqrt(abn*abn + abe*abe + abd*abd);

    // calculate tangent
    Td_n = abn / (norm_ab + 0.01);
    Td_e = abe / (norm_ab + 0.01);
    Td_d = abd / (norm_ab + 0.01);

    // point on track
    d_n = pparam_aa_n;
    d_e = pparam_aa_e;
    d_d = pparam_aa_d;
    
    // desired ground-relative flight path angle
    if ( Td_d > 1.0 ) Td_d = 1.0;
    if ( Td_d < -1.0 ) Td_d = -1.0;
    Gamma_d = -asin(Td_d);

// CURVE SEGMENT
} else if ( pparam_type < 1.5 ) {

    // variable definitions
    const double pparam_cc_n = in[12];
    const double pparam_cc_e = in[13];
    const double pparam_cc_d = in[14];
    const double pparam_R = in[15];
    const double pparam_ldir = in[16];
    const double pparam_gam_sp = in[17];
    const double pparam_xi0 = in[18];
    const double pparam_dxi = in[19];

    // calculate closest point on loiter circle
    const double cp_n = in[0] - pparam_cc_n;
    const double cp_e = in[1] - pparam_cc_e;
    const double norm_cp = sqrt( cp_n*cp_n + cp_e*cp_e );
    const double cp_n_unit = cp_n / (norm_cp + 0.01);
    const double cp_e_unit = cp_e / (norm_cp + 0.01);
    d_n = pparam_R * cp_n_unit + pparam_cc_n;
    d_e = pparam_R * cp_e_unit + pparam_cc_e;

    // calculate tangent
    Td_n = pparam_ldir * -cp_e_unit;
    Td_e = pparam_ldir * cp_n_unit;

    // spiral angular position: [0,2*pi)
    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
    double delta_xi = xi_sp-pparam_xi0;

    // closest point on nearest spiral leg and tangent down component
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

        delta_xi = delta_xi + 6.28318530718;

    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

        delta_xi = delta_xi - 6.28318530718;

    }

    if (fabs(pparam_gam_sp) < 0.001) {

        d_d = pparam_cc_d;
        Td_d = 0.0;
        Gamma_d = 0.0;

    } else {

        const double Rtangam = pparam_R * tan(pparam_gam_sp);

        // spiral height delta for current angle
        const double delta_d_xi = -delta_xi * pparam_ldir * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end = -pparam_dxi * Rtangam;

        // nearest spiral leg
        double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
        const double delta_d_end_k  = round( (delta_d_sp_end - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        // check
        if (delta_d_k * pparam_gam_sp > 0.0) { //NOTE: gam is actually being used for its sign, but writing a sign operator doesnt make a difference here

            delta_d_k = 0.0;

        } else if (fabs(delta_d_k) > fabs(delta_d_end_k) ) {

            delta_d_k = (delta_d_k < 0.0) ? -fabs(delta_d_end_k) : fabs(delta_d_end_k);

        }

        // closest point on nearest spiral leg
        const double delta_d_sp = delta_d_k + delta_d_xi;
        d_d = pparam_cc_d + delta_d_sp;
        Td_d = -sin(pparam_gam_sp);
        Gamma_d = pparam_gam_sp;
    }
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
