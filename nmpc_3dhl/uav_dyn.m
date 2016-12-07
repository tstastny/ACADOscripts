function [d_states, output] = uav_dyn(time, states, ctrls, dstbs, params)
%
% UAV DYNAMICS
%
% last update 19.10.16, Thomas Stastny
%--------------------------------------------------------------------------

% parameters
for i = 1:length(params)
    eval([params(i).Name,'=',num2str(params(i).Value),';']);
end

% states
n = states(1);          % northing
e = states(2);          % easting
d = states(3);          % down

V = states(4);          % airspeed
gamma = states(5);      % flight path angle
xi = states(6);         % heading angle

phi = states(7);        % roll angle
theta = states(8);      % pitch angle

p = states(9);          % roll rate
q = states(10);         % pitch angle rate
r = states(11);         % yaw rate

delta_T = states(12);   % effective throttle setting
if delta_T>1, delta_T=1; end;
if delta_T<0, delta_T=0; end;

mu = phi;

% controls
uT = ctrls(1);          % throttle
if uT>1, uT=1; end;
if uT<0, uT=0; end;
phi_r = ctrls(2);       % roll reference
theta_r = ctrls(3);     % pitch reference

% disturbances
wn = dstbs(1);
we = dstbs(2);
wd = dstbs(3);

% environment/
g = 9.81;
m = 2.65;
rho = 1.225;
S = 0.39;
qbarS = 0.5*rho*V^2*S;

% aero coefficients
alpha = (theta - gamma);
cD = cD0 + cDa * alpha + cDa2 * alpha^2;
cL = cL0 + cLa * alpha + cLa2 * alpha^2;

% forces
T = (cT1*delta_T+cT2*delta_T^2+cT3*delta_T^3)/V/cos(alpha); 
D = qbarS*cD;
L = qbarS*cL;

% angular accelerations
Lm = Lp * p + Lr * r + LeR * (phi_r-phi);
Mm = V^2 * (M0 + Ma * alpha + Mq * q + MeP * (theta_r-theta));
Nm = Nr * r + NR * phi + NRR * phi_r;

% d_states
d_states(1) = V*cos(gamma)*cos(xi) + wn;
d_states(2) = V*cos(gamma)*sin(xi) + we;
d_states(3) = -V*sin(gamma) + wd;
d_states(4) = (T*cos(alpha)-D)/m-g*sin(gamma);
d_states(5) = ((T*sin(alpha)+L)*cos(mu) - m*g*cos(gamma))/m/V;
d_states(6) = (T*sin(alpha)+L)*sin(mu)/m/V/cos(gamma);
d_states(7) = p;
d_states(8) = q*cos(phi)-r*sin(phi);
d_states(9) = Lm;
d_states(10) = Mm;
d_states(11) = Nm;
d_states(12) = (uT - delta_T)/tauT;

% output
output = states;