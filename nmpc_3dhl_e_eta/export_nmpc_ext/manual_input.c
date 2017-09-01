bool check_line_seg( const double *pos, const double *vel, const double *params );
bool check_curve_seg( const double *pos, const double *vel, const double *params );

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
const int idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
bool b_switch_segment = false;
int pparam_sel = 0;
double sw_dot = 0.0;
if ( in[ACADO_NX-1] < 0.05 ) { // check x_sw
    const double vel[3] = {n_dot,e_dot,d_dot};
    if ( in[idx_OD_0] < 0.5 ) { // path type
        b_switch_segment = check_line_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    } else if (in[ACADO_NX+ACADO_NU-minus_NU] < 1.5 ) {
        b_switch_segment = check_curve_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    }
} else {
    b_switch_segment = true;
}
if (b_switch_segment) {
    pparam_sel = 9;
    sw_dot = 1.0;
} 

double p_n = 0.0;
double p_e = 0.0;
double p_d = 0.0;
double tP_n = 1.0;
double tP_e = 0.0;
double tP_d = 0.0;

const double pparam_type = in[idx_OD_0+pparam_sel];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[idx_OD_0+pparam_sel+1];
    const double pparam_aa_e = in[idx_OD_0+pparam_sel+2];
    const double pparam_aa_d = in[idx_OD_0+pparam_sel+3];
    const double pparam_bb_n = in[idx_OD_0+pparam_sel+4];
    const double pparam_bb_e = in[idx_OD_0+pparam_sel+5];
    const double pparam_bb_d = in[idx_OD_0+pparam_sel+6];

    // calculate vector from waypoint a to b
    const double abn = pparam_bb_n - pparam_aa_n;
    const double abe = pparam_bb_e - pparam_aa_e;
    const double abd = pparam_bb_d - pparam_aa_d;
    const double norm_ab = sqrt(abn*abn + abe*abe + abd*abd);

    // calculate tangent
    if (norm_ab>0.1) {
        tP_n = abn / norm_ab;
        tP_e = abe / norm_ab;
        tP_d = abd / norm_ab;
    }
    
    // dot product
    const double dot_abunit_ap = tP_n*(in[0] - pparam_aa_n) + tP_e*(in[1] - pparam_aa_e) + tP_d*(in[2] - pparam_aa_d);
    
    // point on track
    p_n = pparam_aa_n + dot_abunit_ap * tP_n;
    p_e = pparam_aa_e + dot_abunit_ap * tP_e;
    p_d = pparam_aa_d + dot_abunit_ap * tP_d;
    
// CURVE SEGMENT
} else if ( pparam_type < 1.5 ) {

    // variable definitions
    const double pparam_cc_n = in[idx_OD_0+pparam_sel+1];
    const double pparam_cc_e = in[idx_OD_0+pparam_sel+2];
    const double pparam_cc_d = in[idx_OD_0+pparam_sel+3];
    const double pparam_R = in[idx_OD_0+pparam_sel+4];
    const double pparam_ldir = in[idx_OD_0+pparam_sel+5];
    const double pparam_gam_sp = in[idx_OD_0+pparam_sel+6];
    const double pparam_xi0 = in[idx_OD_0+pparam_sel+7];
    const double pparam_dxi = in[idx_OD_0+pparam_sel+8];

    // calculate closest point on loiter circle
    const double cp_n = in[0] - pparam_cc_n;
    const double cp_e = in[1] - pparam_cc_e;
    const double norm_cr = sqrt( cp_n*cp_n + cp_e*cp_e );
    double cp_n_unit;
    double cp_e_unit;
    if (norm_cr>0.1) {
        cp_n_unit = cp_n / norm_cr;
        cp_e_unit = cp_e / norm_cr;
    }
    else {
        cp_n_unit = 0.0;
        cp_e_unit = 0.0;
    }
    p_n = pparam_R * cp_n_unit + pparam_cc_n;
    p_e = pparam_R * cp_e_unit + pparam_cc_e;

    // calculate tangent
    tP_n = pparam_ldir * -cp_e_unit;
    tP_e = pparam_ldir * cp_n_unit;
    
    // spiral angular position: [0,2*pi)
    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
    double delta_xi_p = xi_sp-pparam_xi0;
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

        delta_xi_p = delta_xi_p + 6.28318530718;

    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

        delta_xi_p = delta_xi_p - 6.28318530718;

    }

    // closest point on nearest spiral leg and tangent down component
    if (fabs(pparam_gam_sp) < 0.001) {

        p_d = pparam_cc_d;
        tP_d = 0.0;

    } else {

        const double Rtangam = pparam_R * tan(pparam_gam_sp);

        // spiral height delta for current angle
        const double delta_d_xi = -delta_xi_p * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end = -pparam_dxi * Rtangam;

        // nearest spiral leg
        const double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        // closest point on nearest spiral leg
        p_d = pparam_cc_d + delta_d_k + delta_d_xi;

        /* p (on spiral) = (start height) + (revolution height increment) +
         * (lateral-direcitonal angular position increment)
         */
        
        // cap end point
        if ((p_d - (delta_d_sp_end + pparam_cc_d)) * pparam_gam_sp < 0.0) {
            // we (or more correctly, the closest point on the nearest spiral leg) are beyond the end point
            p_d = pparam_cc_d + delta_d_sp_end;
            tP_d = 0.0;
        }
        else {
            tP_d = -sin(pparam_gam_sp);
        }
        
    }
    
    if (fabs(tP_n)<0.01 && fabs(tP_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        tP_n=1.0;
    }
    
    // Renormalize tP
    const double normtP = sqrt(tP_n*tP_n+tP_e*tP_e+tP_d*tP_d);
    tP_n = tP_n / normtP;
    tP_e = tP_e / normtP;
    tP_d = tP_d / normtP;
        
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

/* begin inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

bool check_line_seg( const double *pos, const double *vel, const double *params ) {
    
    // waypoint a to b
    const double ab_n = params[3] - params[0];
    const double ab_e = params[4] - params[1];
    const double ab_d = params[5] - params[2];
    
    const double norm_ab = sqrt( ab_n*ab_n + ab_e*ab_e + ab_d*ab_d );
    
    // tB
    double tB_n = 1.0;
    double tB_e = 0.0;
    double tB_d = 0.0;
    if (norm_ab > 0.1) {
        tB_n = ab_n / norm_ab;
        tB_e = ab_e / norm_ab;
        tB_d = ab_d / norm_ab;
    }

    // p - b
    const double bp_n = pos[0] - params[3];
    const double bp_e = pos[1] - params[4];
    const double bp_d = pos[2] - params[5];
    
    // dot( (p-r) , tB )
    const double dot_brtB = bp_n * tB_n + bp_e * tB_e + bp_d * tB_d;
    
    // check travel 
    return ( dot_brtB > 0.0 );
    
}

bool check_curve_seg( const double *pos, const double *vel, const double *params ) {
    
    // chi_B
    const double chi_B = params[6] + params[4] * (params[7] + 1.570796326794897);
        
    // tB
    const double tB_n = cos(chi_B)*cos(params[5]);
    const double tB_e = sin(chi_B)*cos(params[5]);
    const double tB_d = -sin(params[5]);

    // p - b
    const double bp_n = pos[0] - ( params[0] + params[3] * ( cos(params[6] + params[4] * params[7]) ) );
    const double bp_e = pos[1] - ( params[1] + params[3] * ( sin(params[6] + params[4] * params[7]) ) );
    const double bp_d = pos[2] - ( params[2] - params[3] * tan(params[5]) * params[7] );
    
    // dot( v , tB )
    const double dot_vtB = vel[0] * tB_n + vel[1] * tB_e + vel[2] * tB_d;
    
    // dot( (p-r) , tB )
    const double dot_brtB = bp_n * tB_n + bp_e * tB_e + bp_d * tB_d;
    
    // norm( p-r )
    const double norm_br = sqrt( bp_n*bp_n + bp_e*bp_e + bp_d*bp_d );
    
    // check (1) proximity, (2) bearing, (3) travel 
    return ( norm_br < params[17] && dot_vtB > params[18] && dot_brtB > 0.0 );
}

/* end inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
