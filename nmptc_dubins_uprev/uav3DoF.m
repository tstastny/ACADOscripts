function [d_states, simout, cost] = uav3DoF(time, states, ctrls, wn, we, wd, V, dyn, pparams)

% state defines
n = states(1);
e = states(2);
d = states(3);
mu = states(4);
gamma = states(5);
xi = states(6);
mu_dot = states(7);
gamma_dot = states(8);

% control defines
mu_r = ctrls(1);
gamma_r = ctrls(2);

omega_n_mu = dyn(1);
zeta_mu = dyn(2);
omega_n_gamma = dyn(3);
zeta_gamma = dyn(4);

% differentials
d_states(1) = V * cos(gamma) * cos(xi) + wn;
d_states(2) = V* cos(gamma) * sin(xi) + we;
d_states(3) = -V * sin(gamma) + wd;
d_states(4) = mu_dot;
d_states(5) = gamma_dot;
d_states(6) = 9.81 * tan(mu) / V;
d_states(7) = omega_n_mu * (omega_n_mu * (mu_r - mu) - 2 * zeta_mu * mu_dot);
d_states(8) = omega_n_gamma * (omega_n_gamma * (gamma_r - gamma) - 2 * zeta_gamma * gamma_dot);

% output
simout(1) = n;
simout(2) = e;
simout(3) = d;
simout(4) = mu;
simout(5) = gamma;
simout(6) = xi;
simout(7) = mu_dot;
simout(8) = gamma_dot;

% cost
pparam1 = pparams(1);
pparam2 = pparams(2);
pparam3 = pparams(3);
pparam4 = pparams(4);
pparam5 = pparams(5);
pparam6 = pparams(6);
pparam7 = pparams(7);
pparam8 = pparams(8);
pparam9 = pparams(9);

% calculate closest point on loiter circle
cp_n = n - pparam2;
cp_e = e - pparam3;
norm_cp = sqrt( cp_n^2 + cp_e^2 );
cp_n_unit = cp_n / (norm_cp + 0.01);
cp_e_unit = cp_e / (norm_cp + 0.01);
cd_n = pparam5 * cp_n_unit;
cd_e = pparam5 * cp_e_unit;
d_n = n + cd_n - cp_n;
d_e = e + cd_e - cp_e;

% variable definitions
pparam_cc_d = pparam4;
pparam_R = pparam5;
pparam_ldir = pparam6;
pparam_gam_sp = pparam7;
pparam_xi0 = pparam8;
pparam_dxi = pparam9;

% spiral angular position: [0,2*pi)
xi_sp = atan2(cp_e_unit, cp_n_unit);
delta_xi = xi_sp-pparam_xi0;

if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp)
    
    delta_xi = delta_xi + 6.28318530718;

elseif (pparam_ldir<0.0 && xi_sp>pparam_xi0)
    
    delta_xi = delta_xi - 6.28318530718;

end

% closest point on nearest spiral leg and tangent down component
if (abs(pparam_gam_sp) < 0.001)

    d_d = pparam_cc_d;
    Td_d = 0.0;
    Gamma_d = 0.0;

else

    Rtangam = pparam_R * tan(pparam_gam_sp);

    % spiral height delta for current angle
    delta_d_xi = -delta_xi * pparam_ldir * Rtangam;

    % end spiral altitude change
    delta_d_sp_end = -pparam_dxi * Rtangam;

    % nearest spiral leg
    delta_d_k = round( (d - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
    delta_d_end_k  = round( (delta_d_sp_end - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

    % check
    if (delta_d_k * pparam_gam_sp > 0.0) %NOTE: gam is actually being used for its sign, but writing a sign operator doesnt make a difference here

        delta_d_k = 0.0;
    
    elseif (abs(delta_d_k) > abs(delta_d_end_k) )
    
        if (delta_d_k < 0.0) 
            delta_d_k = -abs(delta_d_end_k);
        else
            delta_d_k =  abs(delta_d_end_k);
        end
    
    end

    % closest point on nearest spiral leg
    delta_d_sp = delta_d_k + delta_d_xi;
    d_d = pparam_cc_d + delta_d_sp;
    Td_d = -asin(pparam_gam_sp);
    Gamma_d = pparam_gam_sp;
end

% calculate tangent
Td_n = pparam6*-cp_e_unit;
Td_e = pparam6*cp_n_unit;

% track position error
pd_n = d_n - n;
pd_e = d_e - e;
pd_d = d_d - d;
cx = Td_e * pd_d - pd_e * Td_d;
cy = -(Td_n * pd_d - pd_n * Td_d);
cz = Td_n * pd_e - pd_n * Td_e;
et = sqrt( cx^2 + cy^2 + cz^2 );

% % ground speed
% V_g = sqrt(n_dot^2 + e_dot^2 + d_dot^2);
% 
% if (V_g < 0.01), V_g = 0.01; end
% 
% sin_d_dot_V_g = d_dot/V_g;
% 
% if (sin_d_dot_V_g > 1.0)
%     sin_d_dot_V_g = 1.0;
% elseif (sin_d_dot_V_g < -1.0)
%     sin_d_dot_V_g = -1.0;
% end
% 
% Gamma = -asin(sin_d_dot_V_g);
% 
% % Gamma error
% e_Gamma = Gamma_d - Gamma;
% 
% % chi error
% e_chi = atan2(Td_e,Td_n) - atan2(e_dot,n_dot);
% if (e_chi>3.14159265359), e_chi = e_chi - 6.28318530718; end
% if (e_chi<-3.14159265359), e_chi = e_chi + 6.28318530718; end

cost(1) = et;
