function [d_states, output] = model_dynamics(time, states, controls)
%
% MODEL DYNAMICS
%
%--------------------------------------------------------------------------

nX=4;
nU=3;
idx_shift = 0;
% if (endterm_eval) idx_shift = -ACADO_NU; % this is an end term evaluation - dont consider the controls

% STATES - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

v = states(1);
gamma = states(2);
theta = states(3);
zeta_w = states(4);

% CONTROLS - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

delta_w = controls(1);
T_w = controls(2);
theta_ref = controls(3);

% MODEL -------------------------------------------------------------------

% environmental parameters
g = 9.81;       % gravity [m/s^2]
rho = 1.225;    % density of air [kg/m^3]

% model parameters
m = 1.85;       % mass of the plane [kg]
S_w = 0.15;   % surface area of wing [m^2]
k_T2L = 0.3;    % control augmented slipstream gain
k = 1;          % control augmented atitude gain
tau_0 = 0.4;    % control augmente time constant 
c_w = pi/10;    % slew rate of the wing [rad/s]

% intermediate states
alpha = theta - gamma; % angle of attack at zeta_w == 0
v_x = v * cos(alpha);
v_z = v * sin(alpha);


% additional differentials
d_v_x = 1/m*(T_w*cos(zeta_w)-m*g*sin(theta)-k_T2L*T_w*sin(zeta_w)...
    +0.5*rho*v^2*S_w*(C_Ltotal(alpha+zeta_w)*sin(alpha)...
    -C_Dtotal(alpha+zeta_w)*cos(alpha)))-k*(theta_ref - theta)/tau_0*v_z; % v_x dynamics
d_v_z =1/m*(-T_w*sin(zeta_w)+m*g*cos(theta)-k_T2L*T_w*cos(zeta_w)...
    -0.5*rho*v^2*S_w*(C_Ltotal(alpha+zeta_w)*cos(alpha)...
    -C_Dtotal(alpha+zeta_w)*sin(alpha)))+k*(theta_ref - theta)/tau_0*v_x; % v_z dynamics

% differential equations

d_v = (v_x*d_v_x + v_z*d_v_z)/(v); % velocity danamics
d_theta = (k*theta_ref - theta)/tau_0; % theta dynamics
d_gamma = d_theta - (v_x*d_v_z + v_z*d_v_x)/(v^2); % flight path angle dynamics
d_zeta = c_w*delta_w; % zeta_w dynamics

d_states(1) = d_v;
d_states(2) = d_gamma;
d_states(3) = d_theta;
d_states(4) = d_zeta;

% output
output = states;

