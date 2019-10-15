function input_y = get_unit_velocity_setpoint( input_x, input_od )
%GET_UNIT_VELOCITY_SETPOINT Summary of this function goes here
%   Detailed explanation goes here

% states
r_n = input_x(:,1);
r_e = input_x(:,2);
r_d = input_x(:,3);
v = input_x(:,4);
gamma = input_x(:,5);
xi= input_x(:,6);

% online data
w_n = input_od(1);
w_e = input_od(2);
w_d = input_od(3);
b_n = input_od(4);
b_e = input_od(5);
b_d = input_od(6);
Gamma_p = input_od(7);
chi_p = input_od(8);
T_b_lat = input_od(9);
T_b_lon = input_od(10);
gamma_app_max = input_od(11);

% ground speed */
v_cos_gamma = v.*cos(gamma);
cos_xi = cos(xi);
sin_xi = sin(xi);
vG_n = v_cos_gamma.*cos_xi + w_n;
vG_e = v_cos_gamma.*sin_xi + w_e;
vG_d = -v.*sin(gamma) + w_d;

% PATH FOLLOWING */

% path tangent unit vector  */
tP_n_bar = cos(chi_p);
tP_e_bar = sin(chi_p);

% "closest" point on track */
tp_dot_br = tP_n_bar.*(r_n-b_n) + tP_e_bar.*(r_e-b_e);
tp_dot_br_n = tp_dot_br.*tP_n_bar;
tp_dot_br_e = tp_dot_br.*tP_e_bar;
p_lat = tp_dot_br_n.*tP_n_bar + tp_dot_br_e.*tP_e_bar;
p_d = b_d - p_lat.*tan(Gamma_p);

% position error */
e_lat = (r_n-b_n).*tP_e_bar - (r_e-b_e).*tP_n_bar;
e_lon = p_d - r_d;

% lateral-directional error boundary */
e_b_lat = T_b_lat .* sqrt(vG_n.*vG_n + vG_e.*vG_e);

% course approach angle */
chi_app = atan(pi/2.*e_lat./e_b_lat);

% longitudinal error boundary */
e_b_lon = T_b_lon * abs(vG_d);
e_b_lon(abs(vG_d) < 1.0) = T_b_lon .* 0.5 .* (1.0 + vG_d(abs(vG_d) < 1.0).*vG_d(abs(vG_d) < 1.0)); % vG_d may be zero */
    
% flight path approach angle */
Gamma_app = -gamma_app_max .* atan(pi/2.*e_lon./e_b_lon);

% normalized ground velocity setpoint */
v_cos_gamma = cos(Gamma_p + Gamma_app);
vP_n_unit = v_cos_gamma.*cos(chi_p + chi_app);
vP_e_unit = v_cos_gamma.*sin(chi_p + chi_app);
vP_d_unit = -sin(Gamma_p + Gamma_app);

input_y = [vP_n_unit, vP_e_unit, vP_d_unit];

end

