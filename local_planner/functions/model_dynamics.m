function [d_states, output] = model_dynamics(time, states, controls, onlinedata, model_params, sysid_config)
%
% MODEL DYNAMICS
%
%--------------------------------------------------------------------------

nX=9;
nU=3;
idx_shift = 0;
% if (endterm_eval) idx_shift = -ACADO_NU; % this is an end term evaluation - dont consider the controls

% STATES - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

% r_n = states(1+0);
% r_e = states(1+1);
% r_d = states(1+2);
v = states(1+3);
gamma = states(1+4);
xi = states(1+5);
phi = states(1+6);
theta = states(1+7);
n_p = states(1+8);

% CONTROLS - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

u_T = controls(1-nX+9);
phi_ref = controls(1-nX+10);
theta_ref = controls(1-nX+11);

% ONLINE DATA - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

% environment
rho = onlinedata(1-nX-nU+12-idx_shift);
% disturbances
w_n = onlinedata(1-nX-nU+13-idx_shift);
w_e = onlinedata(1-nX-nU+14-idx_shift);
w_d = onlinedata(1-nX-nU+15-idx_shift);
% path reference
% b_n = onlinedata(1-nX-nU+16-idx_shift);
% b_e = onlinedata(1-nX-nU+17-idx_shift);
% b_d = onlinedata(1-nX-nU+18-idx_shift);
% Gamma_p = onlinedata(1-nX-nU+19-idx_shift);
% chi_p = onlinedata(1-nX-nU+20-idx_shift);
% guidance
% T_b_lat = onlinedata(1-nX-nU+21-idx_shift);
% T_b_lon = onlinedata(1-nX-nU+22-idx_shift);
% gamma_app_max = onlinedata(1-nX-nU+23-idx_shift);
% control augmented attitude time constants and gains
tau_phi = onlinedata(1-nX-nU+24-idx_shift);
tau_theta = onlinedata(1-nX-nU+25-idx_shift);
k_phi = onlinedata(1-nX-nU+26-idx_shift);
k_theta = onlinedata(1-nX-nU+27-idx_shift);
% angle of attack soft constraint
% delta_aoa = onlinedata(1-nX-nU+28-idx_shift);
% aoa_m = onlinedata(1-nX-nU+29-idx_shift);
% aoa_p = onlinedata(1-nX-nU+30-idx_shift);
% log_sqrt_w_over_sig1_aoa = onlinedata(1-nX-nU+31-idx_shift);
% one_over_sqrt_w_aoa = onlinedata(1-nX-nU+32-idx_shift);
% height soft constraint 
% h_offset = onlinedata(1-nX-nU+33-idx_shift);
% delta_h = onlinedata(1-nX-nU+34-idx_shift);
% delta_y = onlinedata(1-nX-nU+35-idx_shift);
% log_sqrt_w_over_sig1_h = onlinedata(1-nX-nU+36-idx_shift);
% one_over_sqrt_w_h = onlinedata(1-nX-nU+37-idx_shift);
% radial soft constraint
% r_offset = onlinedata(1-nX-nU+38-idx_shift);
% delta_r0 = onlinedata(1-nX-nU+39-idx_shift);
% k_r = onlinedata(1-nX-nU+40-idx_shift);
% log_sqrt_w_over_sig1_r = onlinedata(1-nX-nU+41-idx_shift);
% one_over_sqrt_w_r = onlinedata(1-nX-nU+42-idx_shift);
% terrain lookup
% terr_local_origin_n = onlinedata(1-nX-nU+43-idx_shift);
% terr_local_origin_e = onlinedata(1-nX-nU+44-idx_shift);
% terr_dis = onlinedata(1-nX-nU+45-idx_shift);
% %terrain_data = onlinedata(1-nX-nU+46-idx_shift);
% IDX_TERR_DATA = 1+46-idx_shift;

% MODEL -------------------------------------------------------------------

% environmental parameters
g = 9.81;       % acceleration of gravity [m/s2]

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

