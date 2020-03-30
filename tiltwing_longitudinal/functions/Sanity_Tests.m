% Definition of parameters
m = 1.85;
g = 9.81;
rho = 1.225; 
S_w = 0.15;
k_T2L = 0.3;
k = 1;
tau_0 = 0.4;
c_w = pi/10;



% Test values for sanity tests
% Initial States
v_x = 10;
v_z = 2;
theta = 0;
zeta_w =  deg2rad(45);

alpha = atan(v_z/v_x)

% Controls
delta_w = 0;
T_w = 15;
theta_ref = 0;
c_D = C_Dtotal(alpha+zeta_w)
c_L = C_Ltotal(alpha+zeta_w)



dot_states(1,1) = 1/m*(T_w*cos(zeta_w)-m*g*sin(theta)-k_T2L*T_w*sin(zeta_w)...
    +0.5*rho*(v_x^2+v_z^2)*S_w*(C_Ltotal(alpha+zeta_w)*sin(alpha)...
    -C_Dtotal(alpha+zeta_w)*cos(alpha)))+k*(theta_ref - theta)/tau_0*v_z; % v_x dynamics
dot_states(2,1) =1/m*(-T_w*sin(zeta_w)+m*g*cos(theta)-k_T2L*T_w*cos(zeta_w)...
    -0.5*rho*(v_x^2+v_z^2)*S_w*(C_Ltotal(alpha+zeta_w)*cos(alpha)...
    -C_Dtotal(alpha+zeta_w)*sin(alpha)))-k*(theta_ref - theta)/tau_0*v_x; % v_z dynamics
dot_states(3,1) = k*(theta_ref - theta)/tau_0; % theta dynamics
dot_states(4,1) = c_w*delta_w; % zeta_w dynamics




dot_states(1,2) = 1/m*(T_w*cos(zeta_w)-m*g*sin(theta)-k_T2L*T_w*sin(zeta_w)...
    +0.5*rho*(v_x^2+v_z^2)*S_w*(C_Ltotal(alpha+zeta_w)*sin(alpha)...
    -C_Dtotal(alpha+zeta_w)*cos(alpha)))+k*(theta_ref - theta)/tau_0*v_z;
dot_states(2,2) = 1/m*(-T_w*sin(zeta_w)+m*g*cos(theta)-k_T2L*T_w*cos(zeta_w)...
    -0.5*rho*(v_x^2+v_z^2)*S_w*(C_Ltotal(alpha+zeta_w)*cos(alpha)...
    -C_Dtotal(alpha+zeta_w)*sin(alpha)))-k*(theta_ref - theta)/tau_0*v_x;
dot_states(3,2) = k*(theta_ref - theta)/tau_0;
dot_states(4,2) = c_w*delta_w;

dot_states