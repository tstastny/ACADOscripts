bool check_line_seg( const double *pos, const double *pparams );
bool check_curve_seg( const double *pos, const double *pparams, const double n_dot, const double e_dot );

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
bool b_switch_segment = false;
int pparam_sel = 0;
double sw_dot = 0.0;
if ( in[5] < 0.05 ) {
    if ( in[8] < 0.5 ) {
        b_switch_segment = check_line_seg( &in[0], &in[9] );
    } else if (in[8] < 1.5 ) {
        b_switch_segment = check_curve_seg( &in[0], &in[9], n_dot, e_dot );
    }
} else {
    b_switch_segment = true;
}
if (b_switch_segment) {
    pparam_sel = 9;
    sw_dot = 1.0;
} 

double d_n = 0.0;
double d_e = 0.0;
double Td_n = 1.0;
double Td_e = 0.0;

const double pparam_type = in[8+pparam_sel];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[9+pparam_sel];
    const double pparam_aa_e = in[10+pparam_sel];
    const double pparam_bb_n = in[12+pparam_sel];
    const double pparam_bb_e = in[13+pparam_sel];

    // calculate vector from waypoint a to b
    const double abn = pparam_bb_n - pparam_aa_n;
    const double abe = pparam_bb_e - pparam_aa_e;
    const double norm_ab = sqrt(abn*abn + abe*abe);

    // calculate tangent
    Td_n = abn / (norm_ab + 0.01);
    Td_e = abe / (norm_ab + 0.01);

    // point on track
    d_n = pparam_aa_n;
    d_e = pparam_aa_e;
    
// CURVE SEGMENT
} else if ( pparam_type < 1.5 ) {

    // variable definitions
    const double pparam_cc_n = in[9+pparam_sel];
    const double pparam_cc_e = in[10+pparam_sel];
    const double pparam_R = in[12+pparam_sel];
    const double pparam_ldir = in[13+pparam_sel];
    const double pparam_gam_sp = in[14+pparam_sel];
    const double pparam_xi0 = in[15+pparam_sel];
    const double pparam_dxi = in[16+pparam_sel];

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
}
/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

/* begin inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

bool check_line_seg( const double *pos, const double *pparams ) {
    
    // calculate vector from waypoint a to b
    const double ab_n = pparams[3] - pparams[0];
    const double ab_e = pparams[4] - pparams[1];
    
    const double norm_ab = sqrt( ab_n*ab_n + ab_e*ab_e );
    
    // 1-D track position
    const double pb_t = norm_ab - ( ab_n*(pos[0]-pparams[0]) + ab_e*(pos[1]-pparams[1]) ) / norm_ab;
    
    // check
    return ( pb_t < 0.0 );
}

bool check_curve_seg( const double *pos, const double *pparams, const double n_dot, const double e_dot ) {
    
    // end point
    const double b_n = pparams[0] + pparams[3] * ( cos(pparams[6] + pparams[4] * pparams[7]) );
    const double b_e = pparams[1] + pparams[3] * ( sin(pparams[6] + pparams[4] * pparams[7]) );
        
    // end point tangent
    const double chi_b = pparams[6] + pparams[4] * (pparams[7] + 1.570796326794897);
        
    const double Tb_n = cos(chi_b);
    const double Tb_e = sin(chi_b);
    
    // dot product
    double V_g = sqrt(n_dot*n_dot + e_dot*e_dot);
    if ( V_g < 0.01 ) V_g = 0.01;
    
    const double dot_T_V = (Tb_n*n_dot + Tb_e*e_dot)/V_g;

    // 1-D track position
    const double bp_n = pos[0]-b_n;
    const double bp_e = pos[1]-b_e;
    const double pb_t = 1.0 - ( Tb_n*(bp_n+Tb_n) + Tb_e*(bp_e+Tb_e) );
        
    // check //TODO: not hard-coded acceptance radius
    return ( pb_t < 0.0 && dot_T_V > 0.8 );
}

/* end inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */