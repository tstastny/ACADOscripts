// path tangent unit vector 
const double tP_n_bar = cos(in[16]);
const double tP_e_bar = sin(in[16]);

// "closest" point on track
const double tp_dot_br = tP_n_bar*(in[0]-in[12]) + tP_e_bar*(in[1]-in[13]);
const double tp_dot_br_n = tp_dot_br*tP_n_bar;
const double tp_dot_br_e = tp_dot_br*tP_e_bar;
const double p_n = in[12] + tp_dot_br_n;
const double p_e = in[13] + tp_dot_br_e;
const double p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar;
const double p_d = in[14] - p_lat*tan(in[15]);

// directional error
const double v_c_gamma = in[8]*cos(in[3]);
const double v_n = v_c_gamma*cos(in[4]);
const double v_e = v_c_gamma*sin(in[4]);
const double tp_dot_vg = tP_n_bar*(v_n+in[9]) + tP_e_bar*(v_e+in[10]);

// terrain
const double h_ter = 10.0;
double h_ter_normalized = (in[2] + h_ter)/in[17];
h_ter_normalized = (h_ter_normalized > 0.0) ? h_ter_normalized : 0.0;

// state output
out[0] = (in[1]-p_e)*cos(in[16]) - (in[0]-p_n)*sin(in[16]);
out[1] = p_d - in[2];
out[2] = tp_dot_vg*0.5+0.5;
out[3] = h_ter_normalized*h_ter_normalized;

// control output
out[4] = in[6]; // gamma ref
out[5] = in[7]; // mu ref
out[6] = (in[6] - in[3])/1; // gamma dot
out[7] = (in[7] - in[5])/0.7; // mu dot