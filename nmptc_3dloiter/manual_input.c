// variable definitions
const double pparam_cc_d = in[12];
const double pparam_R = in[13];
const double pparam_ldir = in[14];
const double pparam_gam_sp = in[15];
const double pparam_xi0 = in[16];
const double pparam_dxi = in[17];

// spiral angular position: [0,2*pi)
const double xi_sp = atan2(cp_e_unit, cp_n_unit);
double delta_xi = xi_sp-pparam_xi0;

if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {
    
    delta_xi = delta_xi + 6.28318530718;

} else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {
    
    delta_xi = delta_xi - 6.28318530718;

}

// closest point on nearest spiral leg and tangent down component
double d_d;
double Td_d;
double Gamma_d;

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
    double delta_d_k = round( (in[11] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
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
    Td_d = -asin(pparam_gam_sp);
    Gamma_d = pparam_gam_sp;
}