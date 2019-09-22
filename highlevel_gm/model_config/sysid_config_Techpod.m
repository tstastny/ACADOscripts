% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% platform specific SYSID config / model parameters
%
% config: Techpod
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

% / environment / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

sysid_config.g = 9.81;          % gravity [m/s^2]
sysid_config.R_air = 287.1; 	% dry air constant [J/kg/K]

% / mass/inertia / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 

sysid_config.mass = 3.124;      % mass [kg]

sysid_config.Ixx = 0.6688;      % x-axis moment of inertia [kg*m^2]
sysid_config.Iyy = 0.16235;     % y-axis moment of inertia [kg*m^2]
sysid_config.Izz = 0.69364;     % z-axis moment of inertia [kg*m^2]
sysid_config.Ixz = 0.0281;      % xz-axis product of inertia [kg*m^2] TODO: add as optimization parameter

% / planform geometry / / / / / / / / / / / / / / / / / / / / / / / / / / / 

% full wing
sysid_config.b_w = 2.598;    % span [m]
sysid_config.S_w = 0.418;    % surface area [m]
sysid_config.c_w = 0.165;    % mean geometric chord [m]
sysid_config.A_w = 16.147;   % aspect ratio [~]

% left outer wing section (1)
sysid_config.S_w1 = 0.080;   % surface area [m^2]
sysid_config.c_w1 = 0.139;   % mean geometric chord [m]

% left inner wing section (2)
sysid_config.S_w2 = 0.129;   % surface area [m^2]
sysid_config.c_w2 = 0.182;   % mean geometric chord [m]

% right inner wing section (3)
sysid_config.S_w3 = 0.129;   % surface area [m^2]
sysid_config.c_w3 = 0.182;   % mean geometric chord [m]

% right outer wing section (4)
sysid_config.S_w4 = 0.080;   % surface area [m^2]
sysid_config.c_w4 = 0.139;   % mean geometric chord [m]

% horizontal tail
sysid_config.S_ht = 0.095;   % surface area [m^2]
sysid_config.c_ht = 0.147;   % mean geometric chord [m]
sysid_config.b_ht = 0.66;    % span [m]
sysid_config.lambda_LE_ht = 0.2148; % leading edge sweep angle [rad]
sysid_config.cr_ht = 0.180; % root chord [m]
sysid_config.ct_ht = 0.108; % tip chord [m]

% vertical tail
sysid_config.S_vt = 0.044;   % surface area [m^2]
sysid_config.c_vt = 0.153;   % mean geometric chord [m]
sysid_config.b_vt = 0.290;   % span [m]
sysid_config.lambda_LE_vt = 0.2563; % leading edge sweep angle [rad]
sysid_config.cr_vt = 0.175; % root chord [m]
sysid_config.ct_vt = 0.130; % tip chord [m]

% / lever arms / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 
% body coordinates (from cg)

% full wing
sysid_config.x_w = 0.013;    % [m]
sysid_config.y_w = 0.000;    % [m]
sysid_config.z_w = -0.065;   % [m]

% left outer wing section (1)
sysid_config.x_w1 = 0.007;   % [m]
sysid_config.y_w1 = -0.993;  % [m]
sysid_config.z_w1 = -0.065;  % [m]

% left inner wing section (2)
sysid_config.x_w2 = 0.017;   % [m]
sysid_config.y_w2 = -0.342;  % [m]
sysid_config.z_w2 = -0.065;  % [m]

% right inner wing section (3)
sysid_config.x_w3 = 0.017;   % [m]
sysid_config.y_w3 = 0.342;   % [m]
sysid_config.z_w3 = -0.065;  % [m]

% right outer wing section (4)
sysid_config.x_w4 = 0.007;   % [m]
sysid_config.y_w4 = 0.993;   % [m]
sysid_config.z_w4 = -0.065;  % [m]

% horizontal tail
sysid_config.x_ht = -0.644;  % [m]
sysid_config.y_ht = 0.000;   % [m]
sysid_config.z_ht = -0.220;  % [m]

% vertical tail
sysid_config.x_vt = -0.573;  % [m]
sysid_config.y_vt = 0.000;   % [m]
sysid_config.z_vt = -0.067;  % [m]

% propeller
sysid_config.x_prop = -0.16; % [m]
sysid_config.y_prop = 0;     % [m]
sysid_config.z_prop = -0.1; % [m]
sysid_config.epsilon_T = deg2rad(2); % [rad]

% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 

% full wing section characteristics for calculations in 'planform_geometry.m'
% - - - - - - - - - -
% unit: m
% half_span = 1.300;
% root_chord = 0.205;
% tip_chord = 0.119;
% y_root = 0.000;
% x_le_root = 0.075;
% x_le_tip = 0.032;
% - - - - - - - - - -
% inner wing section characteristics for calculations in 'planform_geometry.m'
% - - - - - - - - - -
% unit: m
% half_span = 0.715;
% root_chord = 0.205;
% tip_chord = 0.158;
% y_root = 0.000;
% x_le_root = 0.075;
% x_le_tip = 0.051;
% - - - - - - - - - -
% outer wing section characteristics for calculations in 'planform_geometry.m'
% - - - - - - - - - -
% unit: m
% half_span = 0.584;
% root_chord = 0.158;
% tip_chord = 0.119;
% y_root = 0.715;
% x_le_root = 0.051;
% x_le_tip = 0.032;
% - - - - - - - - - -
% horizontal tail characteristics for calculations in 'planform_geometry.m'
% - - - - - - - - - -
% unit: m
% half_span = 0.330;
% root_chord = 0.180;
% tip_chord = 0.108;
% y_root = 0.000;
% x_le_root = -0.575;
% x_le_tip = -0.647;
% - - - - - - - - - -
% vertical tail characteristics for calculations in 'planform_geometry.m'
% - - - - - - - - - -
% unit: m
% half_span = 0.290;
% root_chord = 0.175;
% tip_chord = 0.130;
% y_root = -0.070;
% x_le_root = -0.499;
% x_le_tip = -0.575;
% - - - - - - - - - -

% / actuator mapping / / / / / / / / / / / / / / / / / / / / / / / / / / / 

% conversion slope [deg/pwm]
sysid_config.deg_pwm = [0;               % mot (not used)
                        0.0554053104;    % r ail
                        0.0700360645;    % ele
                        0.0589654743;    % rud
                        0.0564822437;    % l ail
                        0.0470898462;    % r flp
                        0.0488636107];   % l flp

% XXX: revisit this calibration formulation at some point..

% actuator offset [deg]
% d_off = (d_max + d_min) / 2
sysid_config.deg_off = [0; 0; 4; 0; 0; 0; 0];

% actuator limits [deg]
% +/- about actuator offset, d_lim = (d_max - d_min) / 2
sysid_config.deg_lim = [0; 20; 15; 20; 20; 13; 13]; % XXX: this probably should be defined with mins and maxs .. 

% d_max = 2*d_lim + d_min
% d_max = 2*d_off - d_min
% d_max = (2*d_lim + d_min + 2*d_off - d_min)/2
% d_max = d_lim + d_off
% d_min = d_max - 2*d_lim

% actuator sign convention (up/right is positive, down/left is negative)
sysid_config.sign_convention = [1; 1; -1; 1; 1; -1; -1];

% static (v=0) throttle input to rpm mapping (fit) and deadzones
sysid_config.rpm_0      = 143.5585;
sysid_config.rpm_input  = 11052.92;
sysid_config.input_dead = 0.16;
sysid_config.rpm_max    = 9820;
sysid_config.rpm_min    = 1912; % max(0, rpm_input * input_dead + rpm_0)

% prop dimensions
sysid_config.d_prop = 0.28; % [m]

% prop spinning direction (used for spiral slipstream effects): 1=cw;
% -1=ccw (cw = positive roll direction about x-body axis)
sysid_config.dir_rot_prop = 1;

% empirical induced velocity correction in near field - this value would be
% k_w=2.0 in the ideal case (i.e. two times the induced velocity at the
% propeller disk in the settled near field flow beyond the disk)
% typical k_w values:
% - in cruise: 1.8
% - in hover: 0.8
sysid_config.k_w = 1.8; % NOTE: currently no hover regime implemented in model

