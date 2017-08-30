function J = calculate_cost(in,Ns)

ACADO_NX = Ns(1);
ACADO_NU = Ns(2);
minus_NU = 0;

% optimized intermediate calculations */

t2 = cos(in(4+1));

Vsafe = in(3+1);
if (Vsafe<1.0), Vsafe = 1.0; end;

n_dot = in(36+1)+Vsafe*t2*cos(in(5+1));
e_dot = in(37+1)+Vsafe*t2*sin(in(5+1));
d_dot = in(38+1)-Vsafe*sin(in(4+1));

% begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

% CHECK SEGMENT SWITCHING CONDITIONS %TODO: put this in a function!
idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
pparam_sel = 0;

p_n = 0.0;
p_e = 0.0;
p_d = 0.0;
tP_n = 1.0;
tP_e = 0.0;
tP_d = 0.0;

pparam_type = in(idx_OD_0+pparam_sel+1);

% LINE SEGMENT
if ( pparam_type < 0.5 )

    % variable definitions
    pparam_aa_n = in(idx_OD_0+pparam_sel+1+1);
    pparam_aa_e = in(idx_OD_0+pparam_sel+2+1);
    pparam_aa_d = in(idx_OD_0+pparam_sel+3+1);
    pparam_bb_n = in(idx_OD_0+pparam_sel+4+1);
    pparam_bb_e = in(idx_OD_0+pparam_sel+5+1);
    pparam_bb_d = in(idx_OD_0+pparam_sel+6+1);

    % calculate vector from waypoa to b
    abn = pparam_bb_n - pparam_aa_n;
    abe = pparam_bb_e - pparam_aa_e;
    abd = pparam_bb_d - pparam_aa_d;
    norm_ab = sqrt(abn*abn + abe*abe + abd*abd);

    % calculate tangent
    if (norm_ab>0.1)
        tP_n = abn / norm_ab;
        tP_e = abe / norm_ab;
        tP_d = abd / norm_ab;
    end
    
    % dot product
    dot_abunit_ap = tP_n*(in(0+1) - pparam_aa_n) + tP_e*(in(1+1) - pparam_aa_e) + tP_d*(in(2+1) - pparam_aa_d);
    
    % poon track
    p_n = pparam_aa_n + dot_abunit_ap * tP_n;
    p_e = pparam_aa_e + dot_abunit_ap * tP_e;
    p_d = pparam_aa_d + dot_abunit_ap * tP_d;
    
% CURVE SEGMENT
elseif ( pparam_type < 1.5 )

    % variable definitions
    pparam_cc_n = in(idx_OD_0+pparam_sel+1+1);
    pparam_cc_e = in(idx_OD_0+pparam_sel+2+1);
    pparam_cc_d = in(idx_OD_0+pparam_sel+3+1);
    pparam_R = in(idx_OD_0+pparam_sel+4+1);
    pparam_ldir = in(idx_OD_0+pparam_sel+5+1);
    pparam_gam_sp = in(idx_OD_0+pparam_sel+6+1);
    pparam_xi0 = in(idx_OD_0+pparam_sel+7+1);
    pparam_dxi = in(idx_OD_0+pparam_sel+8+1);

    % calculate closest poon loiter circle
    cp_n = in(0+1) - pparam_cc_n;
    cp_e = in(1+1) - pparam_cc_e;
    norm_cr = sqrt( cp_n*cp_n + cp_e*cp_e );
    if (norm_cr>0.1)
        cp_n_unit = cp_n / norm_cr;
        cp_e_unit = cp_e / norm_cr;
    else
        cp_n_unit = 0.0;
        cp_e_unit = 0.0;
    end
    p_n = pparam_R * cp_n_unit + pparam_cc_n;
    p_e = pparam_R * cp_e_unit + pparam_cc_e;

    % calculate tangent
    tP_n = pparam_ldir * -cp_e_unit;
    tP_e = pparam_ldir * cp_n_unit;
    
    % spiral angular position: (0,2*pi)
    xi_sp = atan2(cp_e_unit, cp_n_unit);
    delta_xi_p = xi_sp-pparam_xi0;
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp)

        delta_xi_p = delta_xi_p + 6.28318530718;

    elseif (pparam_ldir<0.0 && xi_sp>pparam_xi0)

        delta_xi_p = delta_xi_p - 6.28318530718;

    end

    % closest poon nearest spiral leg and tangent down component
    if (abs(pparam_gam_sp) < 0.001)

        p_d = pparam_cc_d;
        tP_d = 0.0;

    else

        Rtangam = pparam_R * tan(pparam_gam_sp);

        % spiral height delta for current angle
        delta_d_xi = -delta_xi_p * Rtangam;

        % end spiral altitude change
        delta_d_sp_end = -pparam_dxi * Rtangam;

        % nearest spiral leg
        delta_d_k = round( (in(2+1) - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        % closest poon nearest spiral leg
        p_d = pparam_cc_d + delta_d_k + delta_d_xi;

        % p (on spiral) = (start height) + (revolution height increment) +
         % (lateral-direcitonal angular position increment)
         %
        
        % cap end point
        if ((p_d - (delta_d_sp_end + pparam_cc_d)) * pparam_gam_sp < 0.0)
            % we (or more correctly, the closest poon the nearest spiral leg) are beyond the end point
            p_d = pparam_cc_d + delta_d_sp_end;
            tP_d = 0.0;
        else
            tP_d = -sin(pparam_gam_sp);
        end
        
    end
    
    if (abs(tP_n)<0.01 && abs(tP_e)<0.01) % should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        tP_n=1.0;
    end
    
    % Renormalize tP
    normtP = sqrt(tP_n*tP_n+tP_e*tP_e+tP_d*tP_d);
    tP_n = tP_n / normtP;
    tP_e = tP_e / normtP;
    tP_d = tP_d / normtP;
        
end

% end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

alpha = -in(4+1)+in(7+1);

t3 = in(1+1)-p_e;
t4 = in(0+1)-p_n;
norm_rp_ne = sqrt(t3*t3+t4*t4);

t5 = 1.0/in(44+1);
if (norm_rp_ne>0.0001)
    t6 = 1.0/norm_rp_ne;
else
    t6 = 0.0;
end
t7 = -in(2+1)+p_d;

sgn_rp = 0.0;
if (t7>0.0)
    sgn_rp = 1.0;
elseif (t7<0.0)
    sgn_rp = -1.0;
end

t17 = e_dot*e_dot;
t18 = n_dot*n_dot;
t19 = t17+t18;

e_ne = t4*tP_e-t3*tP_n;

if (t19>in(44+1))
    e_b_ne = sqrt(t19)*in(42+1);                               
else
    e_b_ne = in(42+1)*in(44+1)*(1.0/2.0)+in(42+1)*t5*t19*(1.0/2.0);
end
sat_e_ne = abs(e_ne)/e_b_ne;
if (sat_e_ne>1.0), sat_e_ne = 1.0; end;

t8 = sat_e_ne-1.0;
t9 = t8*t8;
t10 = 3.141592653589793*t9*(1.0/2.0);
t11 = cos(t10);
t12 = sin(t10);

e_d = t7;

if (abs(d_dot)>in(44+1))
    e_b_d = abs(d_dot)*in(43+1);                               
else
    e_b_d = in(43+1)*in(44+1)*(1.0/2.0)+in(43+1)*t5*abs(d_dot)*(1.0/2.0);
end
sat_e_d = abs(e_d)/e_b_d;
if (sat_e_d>1.0), sat_e_d = 1.0; end;

t13 = sat_e_d-1.0;
t14 = t13*t13;
t15 = 3.141592653589793*t14*(1.0/2.0);
t16 = sin(t15);

t20 = alpha-in(39+1)+in(41+1);
t21 = 1.0/(in(41+1)*in(41+1));
t22 = -alpha+in(40+1)+in(41+1);

rp_n_unit = -t4*t6;
rp_e_unit = -t3*t6;

atan2_01 = atan2(rp_e_unit*t11+t12*tP_e, rp_n_unit*t11+t12*tP_n);
atan2_02 = atan2(e_dot, n_dot);
eta_lat = atan2_01-atan2_02;
if (eta_lat>3.141592653589793)
    eta_lat = eta_lat - 6.283185307179586;
elseif (eta_lat<-3.141592653589793)
    eta_lat = eta_lat + 6.283185307179586;
end

atan2_03 = atan2(-t16*tP_d-sgn_rp*cos(t15), t16*sqrt(tP_e*tP_e+tP_n*tP_n));
atan2_04 = atan2(-d_dot, sqrt(t19));
eta_lon = atan2_03-atan2_04;
if (eta_lon>3.141592653589793)
    eta_lon = eta_lon - 6.283185307179586;
elseif (eta_lon<-3.141592653589793)
    eta_lon = eta_lon + 6.283185307179586;
end

if (alpha>(in(39+1)-in(41+1)))
    a_soft=(t20*t20)*t21;
elseif (alpha>(in(40+1)+in(41+1)))
    a_soft=0.0;
else
    a_soft=t21*(t22*t22);
end

% outputs */

% out(0+1) = eta_lat;
% out(1+1) = eta_lon;
% out(2+1) = Vsafe;
% out(3+1) = in(8+1);
% out(4+1) = in(9+1);
% out(5+1) = in(10+1);
% out(6+1) = a_soft;
% out(7+1) = in(11+1)*(-4.143016944939305)+in(13+1)*4.143016944939305;
% out(8+1) = in(13+1);
% out(9+1) = in(14+1);
% out(10+1) = in(15+1);
% out(11+1) = in(13+1);
% out(12+1) = in(14+1);
% out(13+1) = in(15+1);

thetal_lat = t10;
thetal_lon = t15;

J = [e_ne,e_d,p_n,p_e,p_d,tP_n,tP_e,tP_d, eta_lat,eta_lon, e_ne/e_b_ne,e_d/e_b_d, a_soft];

