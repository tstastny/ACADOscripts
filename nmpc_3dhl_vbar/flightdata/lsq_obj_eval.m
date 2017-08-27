function [out,aux] = lsq_obj_eval( in )


ACADO_NX=13;
ACADO_NU=3;

% for manual input indexing ... */

minus_NU = 0;

% optimized intermediate calculations */

t2 = cos(in(5));

alpha = -in(5)+in(8);

t3 = alpha-in(45)+in(47);
t4 = 1.0/(in(47)*in(47));
t5 = -alpha+in(46)+in(47);

Vsafe = in(4);
if (Vsafe<1.0), Vsafe = 1.0; end;

n_dot = in(37)+Vsafe*t2*cos(in(6));
e_dot = in(38)+Vsafe*t2*sin(in(6));
d_dot = in(39)-Vsafe*sin(in(5));

% begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

% CHECK SEGMENT SWITCHING CONDITIONS %TODO: put this in a function!
idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU+1;
b_switch_segment = false;
pparam_sel = 0;
sw_dot = 0.0;
if ( in(13) < 0.05 ) % check x_sw
    vel = [n_dot,e_dot,d_dot];
    if ( in(idx_OD_0) < 0.5 ) % path type
        b_switch_segment = check_line_seg( in, vel, in(idx_OD_0+1:end) );
    elseif (in(ACADO_NX+ACADO_NU-minus_NU+1) < 1.5 )
        b_switch_segment = check_curve_seg( in, vel, in(idx_OD_0+1:end) );
    end
else
    b_switch_segment = true;
end
if (b_switch_segment)
    pparam_sel = 9;
    sw_dot = 1.0;
end 

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

    % calculate vector from waypoint a to b
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
    
    % point on track
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

    % calculate closest point on loiter circle
    cp_n = in(1) - pparam_cc_n;
    cp_e = in(2) - pparam_cc_e;
    norm_cp = sqrt( cp_n*cp_n + cp_e*cp_e );
    if (norm_cp>0.1)
        cp_n_unit = cp_n / norm_cp;
        cp_e_unit = cp_e / norm_cp;
    else
        cp_n_unit = 0;
        cp_e_unit = 0;
    end
    d_n = pparam_R * cp_n_unit + pparam_cc_n;
    d_e = pparam_R * cp_e_unit + pparam_cc_e;

    % calculate tangent
    Td_n = pparam_ldir * -cp_e_unit;
    Td_e = pparam_ldir * cp_n_unit;

    % spiral angular position: [0,2*pi)
    xi_sp = atan2(cp_e_unit, cp_n_unit);
    delta_xi = xi_sp-pparam_xi0;

    % closest point on nearest spiral leg and tangent down component
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp)

        delta_xi = delta_xi + 6.28318530718;

    elseif (pparam_ldir<0.0 && xi_sp>pparam_xi0)

        delta_xi = delta_xi - 6.28318530718;

    end

    if (abs(pparam_gam_sp) < 0.001)

        d_d = pparam_cc_d;
        Td_d = 0.0;

    else

        Rtangam = pparam_R * tan(pparam_gam_sp);

        % spiral height delta for current angle
        delta_d_xi = -delta_xi * pparam_ldir * Rtangam;

        % end spiral altitude change
        delta_d_sp_end = -pparam_dxi * Rtangam;

        % nearest spiral leg
        delta_d_k = round( (in(3) - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
        delta_d_end_k  = round( (delta_d_sp_end - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        % check
        if (delta_d_k * pparam_gam_sp > 0.0) %NOTE: gam is actually being used for its sign, but writing a sign operator doesnt make a difference here

            delta_d_k = 0.0;

        elseif (abs(delta_d_k) > abs(delta_d_end_k) )

            if (delta_d_k < 0.0)
                delta_d_k =  -abs(delta_d_end_k);
            else
                delta_d_k =  abs(delta_d_end_k);
            end
        end

        % closest point on nearest spiral leg
        delta_d_sp = delta_d_k + delta_d_xi;
        d_d = pparam_cc_d + delta_d_sp;
        Td_d = -sin(pparam_gam_sp);
    end
    
    if (abs(Td_n)<0.01 && abs(Td_e)<0.01) % should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        Td_n=1.0;
    end
end

% end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

pd_n = d_n-in(1);
pd_e = d_e-in(2);
pd_d = d_d-in(3);

e_t_ne = -Td_e*pd_n+Td_n*pd_e;
e_t_d = pd_d;

if (e_t_ne>in(43))
    e_t_1_ne = 1.0;
elseif (e_t_ne>-in(43))
    e_t_1_ne = sin((3.141592653589793*e_t_ne*(1.0/2.0))/in(43));
else
    e_t_1_ne = -1.0;
end

if (e_t_d>in(41))
    e_t_1_d = 1.0;
elseif (e_t_d>-in(41))
    e_t_1_d = sin((3.141592653589793*e_t_d*(1.0/2.0))/in(41));
else
    e_t_1_d = -1.0;
end

% negative unit normal vector
norm_pd = sqrt(e_t_ne * e_t_ne + e_t_d * e_t_d);
if (norm_pd<1.0)
    Tpd_n = 0.0;
    Tpd_e = 0.0;
    Tpd_d = 0.0;
else
    Tpd_n = pd_n/norm_pd;
    Tpd_e = pd_e/norm_pd;
    Tpd_d = pd_d/norm_pd;
end

% unit velocity vector
norm_v = sqrt(d_dot*d_dot+e_dot*e_dot+n_dot*n_dot);
if (abs(norm_v)<0.05)
    vbar_n=0.0;
    vbar_e=0.0;
    vbar_d=0.0;
elseif (abs(norm_v)<in(44))
    vbar_n=((sin(2*norm_v/in(44)/3.141592653589793)-1)+1)*n_dot;
    vbar_e=((sin(2*norm_v/in(44)/3.141592653589793)-1)+1)*e_dot;
    vbar_d=((sin(2*norm_v/in(44)/3.141592653589793)-1)+1)*d_dot;
else
    vbar_n=n_dot/norm_v;
    vbar_e=e_dot/norm_v;
    vbar_d=d_dot/norm_v;
end

t6 = e_t_ne+in(43);
t7 = in(42)*t6;
t8 = exp(t7);
t9 = t8+1.0;
t10 = 1.0/t9;
t11 = e_t_ne-in(43);
t15 = in(42)*t11;
t12 = exp(-t15);
t13 = t12+1.0;
t14 = 1.0/t13;
t16 = t10+t14-1.0;
t17 = t10+t14;
t18 = e_t_d+in(41);
t19 = in(40)*t18;
t20 = exp(t19);
t21 = t20+1.0;
t22 = 1.0/t21;
t23 = e_t_d-in(41);
t24 = exp(-in(40)*t23);
t25 = t24+1.0;
t26 = 1.0/t25;

% soft constraints
if (alpha>(in(45)-in(47)))
    a_soft=(t3*t3)*t4;
elseif (alpha>(in(46)+in(47)))
    a_soft=0.0;
else
    a_soft=t4*(t5*t5);
end

% outputs */

out(1,1) = e_t_1_ne;
out(1,2) = e_t_1_d;
out(1,3) = -vbar_n-Td_n*t16+Tpd_n*t17;
out(1,4) = -vbar_e-Td_e*t16+Tpd_e*t17;
out(1,5) = -vbar_d-Td_d*(t22+t26-1.0)+Tpd_d*(t22+t26);
out(1,6) = Vsafe;
out(1,7) = in(9);
out(1,8) = in(10);
out(1,9) = in(11);
out(1,10) = a_soft;
out(1,11) = in(12);
out(1,12) = in(15);
out(1,13) = in(16);
out(1,14) = in(12);
out(1,15) = in(15);
out(1,16) = in(16);

aux(1,1) = e_t_ne;
aux(1,2) = e_t_d;


