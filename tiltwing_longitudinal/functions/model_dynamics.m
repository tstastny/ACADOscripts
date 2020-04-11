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

v_x = states(1);
v_z = states(2);
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
alpha = atan(v_z/v_x); % angle of attack at zeta_w == 0


% state differentials
d_states(1) = 1/m*(T_w*cos(zeta_w)-m*g*sin(theta)-k_T2L*T_w*sin(zeta_w)...
    +0.5*rho*(v_x^2+v_z^2)*S_w*(C_Ltotal(alpha+zeta_w)*sin(alpha)...
    -C_Dtotal(alpha+zeta_w)*cos(alpha)))+k*(theta_ref - theta)/tau_0*v_z; % v_x dynamics
d_states(2) =1/m*(-T_w*sin(zeta_w)+m*g*cos(theta)-k_T2L*T_w*cos(zeta_w)...
    -0.5*rho*(v_x^2+v_z^2)*S_w*(C_Ltotal(alpha+zeta_w)*cos(alpha)...
    -C_Dtotal(alpha+zeta_w)*sin(alpha)))-k*(theta_ref - theta)/tau_0*v_x; % v_z dynamics
d_states(3) = (k*theta_ref - theta)/tau_0; % theta dynamics
d_states(4) = c_w*delta_w; % zeta_w dynamics

% output
output = states;

