bool check_line_seg( double *pos, double *pparams );
bool check_curve_seg( double *pos, double *pparams );


/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
// bool b_switch_segment = false; 
int pparam_sel = 0;
// if ( in[16] < 0.5 ) {
//     b_switch_segment = check_line_seg( &in[0], &in[16] );
// } else if (in[16] < 1.5 ) {
//     b_switch_segment = check_curve_seg( &in[0], &in[16] );
// }
// if (b_switch_segment) pparam_sel = 9;

double d_n = 0.0;
double d_e = 0.0;
double d_d = 0.0;
double Td_n = 1.0;
double Td_e = 0.0;
double Td_d = 0.0;
double Gamma_d = 0.0;

const double pparam_type = in[16+pparam_sel];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[17+pparam_sel];
    const double pparam_aa_e = in[18+pparam_sel];
    const double pparam_aa_d = in[19+pparam_sel];
    const double pparam_bb_n = in[20+pparam_sel];
    const double pparam_bb_e = in[21+pparam_sel];
    const double pparam_bb_d = in[22+pparam_sel];

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
    const double pparam_cc_n = in[17+pparam_sel];
    const double pparam_cc_e = in[18+pparam_sel];
    const double pparam_cc_d = in[19+pparam_sel];
    const double pparam_R = in[20+pparam_sel];
    const double pparam_ldir = in[21+pparam_sel];
    const double pparam_gam_sp = in[22+pparam_sel];
    const double pparam_xi0 = in[23+pparam_sel];
    const double pparam_dxi = in[24+pparam_sel];

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

/* begin inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

bool check_line_seg( double *pos, double *pparams ) {
    
    // calculate vector from waypoint a to b
    const double ab_n = pparams[3] - pparams[0];
    const double ab_e = pparams[4] - pparams[1];
    const double ab_d = pparams[5] - pparams[2];
    
    // 1-D track position
    const double pb_t = sqrt( ab_n*ab_n + ab_e*ab_e + ab_d*ab_d ) * ( 1.0 - (ab_n*(pos[0]-pparams[0]) + ab_e*(pos[1]-pparams[1]) + ab_d*(pos[2]-pparams[2])) );
    
    // check
    return (pb_t < 0.0);
}

bool check_curve_seg( double *pos, double *pparams ) {
    
    const double b_n = pparams[0] + pparams[3] * ( cos(pparams[6] + pparams[4] * pparams[7]) );
    const double b_e = pparams[1] + pparams[3] * ( sin(pparams[6] + pparams[4] * pparams[7]) );
    const double b_d = pparams[2] - pparams[3] * tan(pparams[5]) * pparams[7];
        
    const double Gamma_b = pparams[6] + pparams[4] * (pparams[7] + 1.570796326794897);
        
    const double Tb_n = cos(Gamma_b);
    const double Tb_e = sin(Gamma_b);
    const double Tb_d = -sin(pparams[5]);

    // 1-D track position
    const double pb_t = 1.0 - ( Tb_n*(pos[0]-b_n+Tb_n) + Tb_e*(pos[1]-b_e+Tb_e) + Tb_d*(pos[2]-b_d+Tb_d) );
        
    // check
    return ( pb_t < 0.0 && fabs(b_d - pos[2]) < 10.0 );
}

/* end inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */