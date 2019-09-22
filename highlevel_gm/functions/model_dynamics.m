function [d_states, output] = model_dynamics(time, states, controls, onlinedata, model_params, sysid_config)
%
% MODEL DYNAMICS
%
%--------------------------------------------------------------------------

% STATES - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% r_n = states(1);
% r_e = states(2);
% r_d = states(3);
v = states(4);
gamma = states(5);
xi = states(6);
phi = states(7);
theta = states(8);
n_p = states(9);

% CONTROLS - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
u_T = controls(1);
phi_ref = controls(2);
theta_ref = controls(3);

% ONLINE DATA - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

% disturbances
w_n = onlinedata(1);
w_e = onlinedata(2);
w_d = onlinedata(3);

% path reference
% b_n = onlinedata(4);
% b_e = onlinedata(5);
% b_d = onlinedata(6);
% Gamma_p = onlinedata(7);
% chi_p = onlinedata(8);

% guidance
% T_b_lat = onlinedata(9);
% T_b_lon = onlinedata(10);

% control augmented attitude time constants and gains
tau_phi = onlinedata(11);
tau_theta = onlinedata(12);
k_phi = onlinedata(13);
k_theta = onlinedata(14);

% soft constraints
% k_aoa = onlinedata(15);
% aoa_m = onlinedata(16);
% aoa_p = onlinedata(17);

% terrain
% k_h = onlinedata(18);
% delta_h = onlinedata(19);
% terr_local_origin_n = onlinedata(20);
% terr_local_origin_e = onlinedata(21);
% % terrain_data(19881);
% LEN_TERR_DATA = 225;
% terrain_data(LEN_TERR_DATA);

% MODEL -------------------------------------------------------------------

% environmental parameters
g = 9.81;       % acceleration of gravity [m/s2]
rho = 1.15;     % air density [kg/m^3]

% model parameters

% salient characteristics
mass = sysid_config.mass;   % mass [kg]

% aerodynamic and thrusting parameters
for i = 1:length(model_params)
    eval([model_params(i).Name,' = ',num2str(model_params(i).Value),';']);
end

% prop speed time constant
tau_n = 0.2;

% prop advance ratio scaling
vmin = 10;                                                                  % minimum stabilizable airspeed on trim map
vmax = 25;                                                                  % maximum stabilizable airspeed on trim map
n_T0_vmin = (-cT_1*vmin/sysid_config.d_prop + ...                           % zero-thrust prop speed at minimum airspeed
    sqrt((cT_1*vmin/sysid_config.d_prop)^2 - cT_0*4))/2/cT_0;
n_T0_vmax = (-cT_1*vmax/sysid_config.d_prop + ...                           % zero-thrust prop speed at maximum airspeed
    sqrt((cT_1*vmax/sysid_config.d_prop)^2 - cT_0*4))/2/cT_0;

% static modeling
qbarS = 1/2 * rho * v^2 * sysid_config.S_w;                                 % dynamic pressure
aoa = theta - gamma;                                                        % angle of attack
L = qbarS * (cL_0 + cL_aoa*aoa);                                            % lift
D = qbarS * (cD_0 + cD_aoa*aoa + cD_aoa2*aoa^2);                            % drag
vp = v * cos(aoa - sysid_config.epsilon_T);                                 % inflow at propeller
sig_vp = (vp - vmin)/(vmax - vmin);                                         % prop inflow linear interpolater
u_n = ...                                                                   % prop speed input (converted from throttle input considering inflow and zero thrusting conditions)
    (n_T0_vmin + u_T * (sysid_config.rpm_max/60 - n_T0_vmin)) * (1 - sig_vp) + ...          
    (n_T0_vmax + u_T * (sysid_config.rpm_max/60 - n_T0_vmax)) * sig_vp;                       
T = rho * n_p^2 * sysid_config.d_prop^4 * ...                               % thrust (assuming n_p min > 0! -- should not be possible if control constraints are respected)
    (cT_0 + cT_1 * vp / n_p / sysid_config.d_prop); 

% state differentials
d_states(1) = v * cos(gamma) * cos(xi) + w_n;
d_states(2) = v * cos(gamma) * sin(xi) + w_e;
d_states(3) = -v * sin(gamma) + w_d;
d_states(4) = 1/mass * (T * cos(aoa) - D) - g * sin(gamma);
d_states(5) = 1/mass/v * ( (T * sin(aoa) + L) * cos(phi) - mass * g * cos(gamma) ); 
d_states(6) = sin(phi)/mass/v/cos(gamma) * (T * sin(aoa) + L);
d_states(7) = (k_phi * phi_ref - phi) / tau_phi;
d_states(8) = (k_theta * theta_ref - theta) / tau_theta;
d_states(9) = (u_n - n_p) / tau_n;

% output
output = states;

