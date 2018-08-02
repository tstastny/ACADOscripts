/* PATH CALCULATIONS */
    
    // path tangent unit vector 
    const double tP_n_bar = cos(in[16]);
    const double tP_e_bar = sin(in[16]);
    
    // "closest" point on track
    const double tp_dot_br = tP_n_bar*(in[0]-in[12]) + tP_e_bar*(in[1]-in[13]);
    const double tp_dot_br_n = tp_dot_br*tP_n_bar;
    const double tp_dot_br_e = tp_dot_br*tP_e_bar;
    const double p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar;
    const double p_d = in[14] - p_lat*tan(in[15]);
    
    /* DIRECTIONAL GUIDANCE */
    
    // lateral-directional error
    const double e_lat = ((in[0]-in[12])*tP_e_bar - (in[1]-in[13])*tP_n_bar);
    
    // ground speed
    const double v_c_gamma = in[8]*cos(in[3]);
    const double vG_n = v_c_gamma*cos(in[4]) + in[9];
    const double vG_e = v_c_gamma*sin(in[4]) + in[10];
    const double vG_d = -in[8]*sin(in[3]) + in[11];
    const double norm_vg_lat2 = vG_n*vG_n + vG_e*vG_e;
    const double norm_vg_lat = sqrt(norm_vg_lat2);
    
    // lateral-directional track-error boundary
    double e_b_lat;
    if (norm_vg_lat > 1.0) {
        e_b_lat = norm_vg_lat*in[17];                               
    } else {
        e_b_lat = 0.5*in[17]*(norm_vg_lat2 + 1.0);
    }
    
    // lateral-directional setpoint
    double normalized_e_lat = e_lat/e_b_lat;
    const double chi_sp = in[16] + atan(M_2_PI * normalized_e_lat);
    
    // lateral-directional error
    double chi_err = chi_sp - atan2(vG_e,vG_n);
    if (chi_err > M_PI) chi_err = chi_err - M_2_PI;
    if (chi_err < -M_PI) chi_err = chi_err + M_2_PI;
    
    /* LONGITUDINAL GUIDANCE */

    // normalized track-error
    normalized_e_lat = fabs(normalized_e_lat);
    if (normalized_e_lat > 1.0) normalized_e_lat = 1.0;
    
    // smooth track proximity factor
    double track_prox = cos(M_PI_2 * normalized_e_lat);
    track_prox = track_prox * track_prox;
    
    // path down velocity setpoint
    vP_d = in[15] * sqrt(norm_vg_lat2 + vg_d*vg_d) * track_prox  - in[11];
    
    // longitudinal velocity increment
    double delta_vd;
    if (in[2] < 0.0) {
        delta_vd = VD_SINK + VD_EPS - vP_d;
    }
    else {
        delta_vd = VD_CLMB - VD_EPS - vP_d;
    }
    
    // longitudinal track-error boundary
    const double e_b_lon = in[18] * delta_vd;
    const double nomralized_e_lon = fabs(in[2]/e_b_lon);
    
    // longitudinal approach velocity
    const double vsp_d_app = TWO_OVER_PI * atan(M_2_PI * nomralized_e_lon) * delta_vd + vP_d;
    
    // down velocity setpoint (air-mass relative)
    const double vsp_d = vP_d + vsp_d_app;
    
    // flight path angle setpoint
    double vsp_d_over_v = vsp_d/in[8];
    if (vsp_d_over_v > 1.0) vsp_d_over_v = 1.0;
    if (vsp_d_over_v < -1.0) vsp_d_over_v = -1.0;
    const double gamma_sp = asin(vsp_d_over_v);
       
    /* TERRAIN */
    
    // lookup 2.5d grid
    int idx_q[4];
    double dh[2];
    lookup_terrain_idx(in[0], in[1], in[20], in[21], idx_q, dh);
    
    // bi-linear interpolation
    const double h12 = (1-dh[0])*in[22+idx_q[0]] + dh[0]*in[22+idx_q[1]];
    const double h34 = (1-dh[0])*in[22+idx_q[2]] + dh[0]*in[22+idx_q[3]];
    const double h_terr = (1-dh[1])*h12 + dh[1]*h34;
    
    // soft constraint formulation
    double one_minus_h_normalized = 1.0 + (in[2] + h_terr)/in[19];
    if (one_minus_h_normalized <= 0.0) one_minus_h_normalized = 0.0;
    
    // constraint priority
    const double sig_h = (one_minus_h_normalized > 1.0) ? 1.0 : cos(M_PI*one_minus_h_normalized)*0.5+0.5;
    
    // state output
    out[0] = sig_h*chi_err;
    out[1] = one_minus_h_normalized*one_minus_h_normalized;
    
    // control output
    out[2] = sig_h*gamma_sp - in[6];    // gamma ref
    out[3] = in[7];                     // phi ref
    out[4] = (in[6] - in[3])/1;         // gamma dot
    out[5] = (in[7] - in[5])/0.5;       // phi dot