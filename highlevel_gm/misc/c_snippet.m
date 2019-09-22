/* DEFINE INPUTS -- this is just simply easier to read.. */
    
/* states */
const double r_n = in[0];
const double r_e = in[1];
const double r_d = in[2];
const double v = in[3];
const double gamma = in[4];
const double xi = in[5];
const double phi = in[6];
const double theta = in[7];
//const double n_p = in[8];

/* controls */
const double u_T = in[9];
const double phi_ref = in[10];
const double theta_ref = in[11];

/* online data */

// disturbances
const double w_n = in[12];
const double w_e = in[13];
const double w_d = in[14];

// path reference
const double b_n = in[15];
const double b_e = in[16];
const double b_d = in[17];
const double Gamma_p = in[18];
const double chi_p = in[19];

//control augmented attitude time constants and gains
//const double tau_phi = in[20];
//const double tau_theta = in[21];
//const double k_phi = in[22];
//const double k_theta = in[23];

// soft angle of attack constraints
const double k_aoa = in[24];
const double aoa_m = in[25];
const double aoa_p = in[26];

// terrain
const double k_h = in[27];
const double delta_h = in[28];
const double terr_local_origin_n = in[29];
const double terr_local_origin_e = in[30];
//const double terrain_data = in[31];
int IDX_TERR_DATA = 31;

/* INTERMEDIATE CALCULATIONS */

// ground speed
double v_cos_gamma = v*cos(gamma);
const double vG_n = v_cos_gamma*cos(xi) + w_n;
const double vG_e = v_cos_gamma*sin(xi) + w_e;
const double vG_d = -v*sin(gamma) + w_d;

/* PATH FOLLOWING */

// path tangent unit vector 
const double tP_n_bar = cos(chi_p);
const double tP_e_bar = sin(chi_p);

// "closest" point on track
const double tp_dot_br = tP_n_bar*(r_n-b_n) + tP_e_bar*(r_e-b_e);
const double tp_dot_br_n = tp_dot_br*tP_n_bar;
const double tp_dot_br_e = tp_dot_br*tP_e_bar;
const double p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar;
const double p_d = b_d - p_lat*tan(Gamma_p);

// position error
const double e_lat = ((r_n-b_n)*tP_e_bar - (r_e-b_e)*tP_n_bar);
const double e_lon = p_d - r_d;

// velocity error
v_cos_gamma = v*cos(Gamma_p);
const double vP_n = v_cos_gamma*cos(chi_p);
const double vP_e = v_cos_gamma*sin(chi_p);
const double vP_d = -v*sin(Gamma_p);
const double e_v_n = vP_n - vG_n;
const double e_v_e = vP_e - vG_e;
const double e_v_d = vP_d - vG_d;

/* SOFT CONSTRAINTS */

// angle of attack
const double aoa_mid = (aoa_p - aoa_m) / 2.0;
double aoa = theta - gamma;
if (aoa > aoa_mid) aoa = 2.0*aoa_mid - aoa;
const double sig_aoa = 1.0/(1.0 + exp(k_aoa*(aoa - aoa_m)));

/* TERRAIN */

// lookup 2.5d grid
int idx_q[4];
double dh[2];
lookup_terrain_idx(r_n, r_e, terr_local_origin_n, terr_local_origin_e, idx_q, dh);

// bi-linear interpolation
const double h12 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[0]] + dh[0]*in[IDX_TERR_DATA+idx_q[1]];
const double h34 = (1-dh[0])*in[IDX_TERR_DATA+idx_q[2]] + dh[0]*in[IDX_TERR_DATA+idx_q[3]];
const double h_terr = (1-dh[1])*h12 + dh[1]*h34;

// soft constraint
const double sig_h = 1.0 / (1 + exp(k_h*((-r_d - h_terr) - delta_h)));

// state output
out[0] = e_lat;
out[1] = e_lon;
out[2] = e_v_n;
out[3] = e_v_e;
out[4] = e_v_d;
out[5] = v;
    out[6] = sig_aoa;
    out[7] = sig_h;
    
    // control output
    out[8] = u_T;
    out[9] = phi_ref;
    out[10] = theta_ref;