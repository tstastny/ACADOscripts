function J = calculate_cost(in,Ns)

ACADO_NX = Ns(1);
ACADO_NU = Ns(2);
minus_NU = 0;

% optimized intermediate calculations */

t2 = cos(in(1+4));

alpha = -in(1+4)+in(1+7);

Vsafe = in(1+3);
if (Vsafe<1.0), Vsafe = 1.0; end;

n_dot = in(1+38)+Vsafe*t2*cos(in(1+5));
e_dot = in(1+39)+Vsafe*t2*sin(in(1+5));
d_dot = in(1+40)-Vsafe*sin(in(1+4));

% begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

% CHECK SEGMENT SWITCHING CONDITIONS %TODO: put this in a function!
idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU+1;
pparam_sel = 0;

d_n = 0.0;
d_e = 0.0;
d_d = 0.0;
Td_n = 1.0;
Td_e = 0.0;
Td_d = 0.0;

pparam_type = in(idx_OD_0+pparam_sel);

% LINE SEGMENT
if ( pparam_type < 0.5 )

    % variable definitions
    pparam_aa_n = in(idx_OD_0+pparam_sel+1);
    pparam_aa_e = in(idx_OD_0+pparam_sel+2);
    pparam_aa_d = in(idx_OD_0+pparam_sel+3);
    pparam_bb_n = in(idx_OD_0+pparam_sel+4);
    pparam_bb_e = in(idx_OD_0+pparam_sel+5);
    pparam_bb_d = in(idx_OD_0+pparam_sel+6);

    % calculate vector from waypoa to b
    abn = pparam_bb_n - pparam_aa_n;
    abe = pparam_bb_e - pparam_aa_e;
    abd = pparam_bb_d - pparam_aa_d;
    norm_ab = sqrt(abn*abn + abe*abe + abd*abd);

    % calculate tangent
    if (norm_ab>0.1)
        Td_n = abn / norm_ab;
        Td_e = abe / norm_ab;
        Td_d = abd / norm_ab;
    end
    
    % dot product
    dot_abunit_ap = Td_n*(in(1) - pparam_aa_n) + Td_e*(in(2) - pparam_aa_e) + Td_d*(in(3) - pparam_aa_d);
    
    % poon track
    d_n = pparam_aa_n + dot_abunit_ap * Td_n;
    d_e = pparam_aa_e + dot_abunit_ap * Td_e;
    d_d = pparam_aa_d + dot_abunit_ap * Td_d;
    
% CURVE SEGMENT
elseif ( pparam_type < 1.5 )

    % variable definitions
    pparam_cc_n = in(idx_OD_0+pparam_sel+1);
    pparam_cc_e = in(idx_OD_0+pparam_sel+2);
    pparam_cc_d = in(idx_OD_0+pparam_sel+3);
    pparam_R = in(idx_OD_0+pparam_sel+4);
    pparam_ldir = in(idx_OD_0+pparam_sel+5);
    pparam_gam_sp = in(idx_OD_0+pparam_sel+6);
    pparam_xi0 = in(idx_OD_0+pparam_sel+7);
    pparam_dxi = in(idx_OD_0+pparam_sel+8);

    % calculate closest poon loiter circle
    cp_n = in(1) - pparam_cc_n;
    cp_e = in(2) - pparam_cc_e;
    norm_cp = sqrt( cp_n*cp_n + cp_e*cp_e );
    if (norm_cp>0.1)
        cp_n_unit = cp_n / norm_cp;
        cp_e_unit = cp_e / norm_cp;
    else
        cp_n_unit = 0.0;
        cp_e_unit = 0.0;
    end
    d_n = pparam_R * cp_n_unit + pparam_cc_n;
    d_e = pparam_R * cp_e_unit + pparam_cc_e;

    % calculate tangent
    Td_n = pparam_ldir * -cp_e_unit;
    Td_e = pparam_ldir * cp_n_unit;
    
    % spiral angular position: [0,2*pi)
    xi_sp = atan2(cp_e_unit, cp_n_unit);
    delta_xi_p = xi_sp-pparam_xi0;
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp)

        delta_xi_p = delta_xi_p + 6.28318530718;

    elseif (pparam_ldir<0.0 && xi_sp>pparam_xi0)

        delta_xi_p = delta_xi_p - 6.28318530718;

    end

    % closest poon nearest spiral leg and tangent down component
    if (abs(pparam_gam_sp) < 0.001)

        d_d = pparam_cc_d;
        Td_d = 0.0;

    else

        Rtangam = pparam_R * tan(pparam_gam_sp);

        % spiral height delta for current angle
        delta_d_xi = -delta_xi_p * Rtangam;

        % end spiral altitude change
        delta_d_sp_end = -pparam_dxi * Rtangam;

        % nearest spiral leg
        delta_d_k = round( (in(3) - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        % closest poon nearest spiral leg
        d_d = pparam_cc_d + delta_d_k + delta_d_xi;

        % d (on spiral) = (start height) + (revolution height increment) +
        % (lateral-direcitonal angular position increment)
        
        % cap end point
        if ((d_d - (delta_d_sp_end + pparam_cc_d)) * pparam_gam_sp < 0.0)
            % we (or more correctly, the closest poon the nearest spiral leg) are beyond the end point
            d_d = pparam_cc_d + delta_d_sp_end;
            Td_d = 0.0;
        else
            Td_d = -sin(pparam_gam_sp);
        end
        
    end
    
    if (abs(Td_n)<0.01 && abs(Td_e)<0.01) % should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        Td_n=1.0;
    end
    
    % Renormalize Td
    normTd = sqrt(Td_n*Td_n+Td_e*Td_e+Td_d*Td_d);
    Td_n = Td_n / normTd;
    Td_e = Td_e / normTd;
    Td_d = Td_d / normTd;
        
end

% end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

t3 = d_e-in(1+1);
t4 = d_n-in(1+0);
t6 = d_d-in(1+2);

norm_pd_ne = sqrt(t3*t3+t4*t4);
e_t_ne = -Td_e*t4+Td_n*t3;
e_t_d = t6;

pd_n_unit = 0.0; 
pd_e_unit = 0.0;
if (norm_pd_ne>0.001)
    pd_n_unit = t4/norm_pd_ne;
    pd_e_unit = t3/norm_pd_ne;
end

% take sign of height difference
sgn_pd = 0.0;
if (t6<0.0)
    sgn_pd = -1.0;
elseif (t6>0.0)
    sgn_pd = 1.0;
end

% saturated track error ratios
sat_e_ne = abs(e_t_ne)/in(1+44);
if ( sat_e_ne>1.0 ), sat_e_ne = 1.0; end;

sat_e_d = abs(e_t_d)/in(1+45);
if( sat_e_d>1.0 ), sat_e_d = 1.0; end;

t7 = sat_e_ne-1.0;
t8 = t7*t7;
% t9 = t8*3.141592653589793*(1.0/2.0);
% t10 = sin(t9);
% t11 = cos(t9);
t9 = t8;
t10 = t9;
t11 = 1-t9;
t12 = sat_e_d-1.0;
t13 = t12*t12;
% t14 = t13*3.141592653589793*(1.0/2.0);
% t15 = sin(t14);
t14 = t13;
t15 = t14;
t16 = alpha-in(1+41)+in(1+43);
t17 = 1.0/(in(1+43)*in(1+43));
t18 = -alpha+in(1+42)+in(1+43);

% soft constraints
if (alpha>(in(1+41)-in(1+43)))
    a_soft=(t16*t16)*t17;
elseif (alpha>(in(1+42)+in(1+43)))
    a_soft=0.0;
else
    a_soft=t17*(t18*t18);
end

% error angles
atan2_01 = atan2(Td_e*t10+pd_e_unit*t11, Td_n*t10+pd_n_unit*t11);
atan2_02 = atan2(e_dot, n_dot);

eta_lat = atan2_01-atan2_02;
if (eta_lat>3.141592653589793)
    eta_lat = eta_lat - 6.283185307179586;
elseif (eta_lat<-3.141592653589793)
    eta_lat = eta_lat + 6.283185307179586;
end

% atan2_03 = atan2(-Td_d*t15-sgn_pd*cos(t14), t15*sqrt(Td_e*Td_e+Td_n*Td_n));
atan2_03 = atan2(-Td_d*t15-sgn_pd*(1-t14), t15*sqrt(Td_e*Td_e+Td_n*Td_n));
atan2_04 = atan2(-d_dot, sqrt(e_dot*e_dot+n_dot*n_dot));

eta_lon = atan2_03-atan2_04;
if (eta_lon>3.141592653589793)
    eta_lon = eta_lon - 6.283185307179586;
elseif (eta_lon<-3.141592653589793)
    eta_lon = eta_lon + 6.283185307179586;
end

% outputs */

out(1,1+0) = eta_lat;
out(1,1+1) = eta_lon;
out(1,1+2) = in(1+12);
out(1,1+3) = in(1+13);
out(1,1+4) = Vsafe;
out(1,1+5) = in(1+8);
out(1,1+6) = in(1+9);
out(1,1+7) = in(1+10);
out(1,1+8) = a_soft;
out(1,1+9) = in(1+11)*(-4.143016944939305)+in(1+15)*4.143016944939305;
out(1,1+10) = in(1+15);
out(1,1+11) = in(1+16);
out(1,1+12) = in(1+17);
out(1,1+13) = in(1+15);
out(1,1+14) = in(1+16);
out(1,1+15) = in(1+17);

J = [out,d_n,d_e,d_d,Td_n,Td_e,Td_d,e_t_ne,e_t_d];


