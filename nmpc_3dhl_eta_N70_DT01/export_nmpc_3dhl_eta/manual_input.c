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
        b_switch_segment = check_line_seg( &in[0], &vel[0], &in[idx_OD_0] );
    } else if (in[idx_OD_0] < 1.5 ) {
        b_switch_segment = check_curve_seg( &in[0], &vel[0], &in[idx_OD_0] );
    }
} else {
    b_switch_segment = true;
}
if (b_switch_segment) {
    pparam_sel = 7;
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

    // calculate tangent
    double tP_n = cos(in[idx_OD_0+pparam_sel+6])*cos(in[idx_OD_0+pparam_sel+5]);
    double tP_e = cos(in[idx_OD_0+pparam_sel+6])*sin(in[idx_OD_0+pparam_sel+5]);
    double tP_d = -sin(in[idx_OD_0+pparam_sel+6]);
    
    // dot product
    const double dot_tP_bp = tP_n*(in[0] - in[idx_OD_0+pparam_sel+1]) + tP_e*(in[1] - in[idx_OD_0+pparam_sel+2]) + tP_d*(in[2] - in[idx_OD_0+pparam_sel+3]);
    
    // point on track
    p_n = in[idx_OD_0+pparam_sel+1] + dot_tP_bp * tP_n;
    p_e = in[idx_OD_0+pparam_sel+2] + dot_tP_bp * tP_e;
    p_d = in[idx_OD_0+pparam_sel+3] + dot_tP_bp * tP_d;
    
// ARC SEGMENT
} else if ( pparam_type < 1.5 ) {

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
    if (delta_xi < 0.0) delta_xi = delta_xi + 6.28318530718;

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
        
        // cap end point
        if ((p_d - in[idx_OD_0+pparam_sel+3]) * in[idx_OD_0+pparam_sel+6] < 0.0) {
            p_d = in[idx_OD_0+pparam_sel+3];
            tP_d = 0.0;
            Gam_temp = 0.0;
        }
        else {
            tP_d = -sin(in[idx_OD_0+pparam_sel+6]);
        }
        
    }
    
    if (fabs(tP_n)<0.01 && fabs(tP_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        tP_n=1.0;
        tP_e=0.0;
    }
    
    // Normalize tP
    tP_n = tP_n * cos(Gam_temp);
    tP_e = tP_e * cos(Gam_temp);
    
// LOITER UNLIM
} else if ( pparam_type < 2.5 ) {

    const double pparam_ldir = (in[idx_OD_0+pparam_sel+4]<0.0) ? -1.0 : 1.0;

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
    
    p_d = in[idx_OD_0+pparam_sel+3];
    tP_d = 0.0;
    
    if (fabs(tP_n)<0.01 && fabs(tP_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        tP_n=1.0;
        tP_e=0.0;
    }
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

/* begin inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

/* begin inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

bool check_line_seg( const double *pos, const double *vel, const double *params ) {
    
    // tB
    const double tB_n = cos(params[6])*cos(params[5]);
    const double tB_e = cos(params[6])*sin(params[5]);
    const double tB_d = -sin(params[6]);

    // r - b
    const double br_n = pos[0] - params[1];
    const double br_e = pos[1] - params[2];
    const double br_d = pos[2] - params[3];
    
    // dot( (r-b) , tB )
    const double dot_brtB = br_n * tB_n + br_e * tB_e + br_d * tB_d;
    
    // check travel 
    return ( dot_brtB > 0.0 );
    
}

bool check_curve_seg( const double *pos, const double *vel, const double *params ) {
    
    // tB
    const double tB_n = cos(params[6])*cos(params[5]);
    const double tB_e = cos(params[6])*sin(params[5]);
    const double tB_d = -sin(params[6]);

    // r - b
    const double rot90 = (params[4]<0.0) ? -1.570796326794897 : 1.570796326794897;
    const double br_n = pos[0] - (params[1] + fabs(params[4]) * cos(params[5] - rot90));
    const double br_e = pos[1] - (params[2] + fabs(params[4]) * sin(params[5] - rot90));
    const double br_d = pos[2] - params[3];
    
    // bearing : dot( v , tB ) (lat)
    const double dot_vtB = vel[0] * tB_n + vel[1] * tB_e;
    
    // travel : dot( (r-b) , tB ) (lat)
    const double dot_brtB = br_n * tB_n + br_e * tB_e;
    
    // proximity : norm( r-b ) (lat)
    const double norm_br = sqrt( br_n*br_n + br_e*br_e );
    
    // proximity : norm( r-b ) (lon)
    const double norm_br_d = fabs(br_d);
    
    // check (1) proximity, (2) bearing, (3) travel 
    return ( norm_br < params[14] && norm_br_d < 10.0 && dot_vtB > params[15]*sqrt(vel[0]*vel[0]+vel[1]*vel[1]) && dot_brtB > 0.0 );
}

/* end inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
