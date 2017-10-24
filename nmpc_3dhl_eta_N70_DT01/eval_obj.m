function [out,aux] = eval_obj(in,Ns)

ACADO_NX = Ns(1);
ACADO_NU = Ns(2);
minus_NU = 0;

% optimized intermediate calculations */

t2 = cos(in(4+1));

alpha = -in(4+1)+in(7+1);

Vsafe = in(3+1);
if (Vsafe<1.0), Vsafe = 1.0; end;

n_dot = in(32+1)+Vsafe*t2*cos(in(5+1));
e_dot = in(33+1)+Vsafe*t2*sin(in(5+1));
d_dot = in(34+1)-Vsafe*sin(in(4+1));

% begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

% CHECK SEGMENT SWITCHING CONDITIONS %TODO: put this in a function!
idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
pparam_sel = 0;
sw_dot = 0.0;

p_n = 0.0;
p_e = 0.0;
p_d = 0.0;
tP_n = 1.0;
tP_e = 0.0;
tP_d = 0.0;

pparam_type = in(idx_OD_0+pparam_sel+1);

phi_ff = 0.0;

% LINE SEGMENT
if ( pparam_type < 0.5 ) 

    % calculate tangent
    tP_n = cos(in(idx_OD_0+pparam_sel+6+1))*cos(in(idx_OD_0+pparam_sel+5+1));
    tP_e = cos(in(idx_OD_0+pparam_sel+6+1))*sin(in(idx_OD_0+pparam_sel+5+1));
    tP_d = -sin(in(idx_OD_0+pparam_sel+6+1));
    
    % dot product
    dot_tP_bp = tP_n*(in(0+1) - in(idx_OD_0+pparam_sel+1+1)) + tP_e*(in(1+1) - in(idx_OD_0+pparam_sel+2+1)) + tP_d*(in(2+1) - in(idx_OD_0+pparam_sel+3+1));
    
    % poon track
    p_n = in(idx_OD_0+pparam_sel+1+1) + dot_tP_bp * tP_n;
    p_e = in(idx_OD_0+pparam_sel+2+1) + dot_tP_bp * tP_e;
    p_d = in(idx_OD_0+pparam_sel+3+1) + dot_tP_bp * tP_d;
    
    % feed forward
    phi_ff = 0.0;
    
% ARC SEGMENT
elseif ( pparam_type < 1.5 ) 

% variable definitions
%     pparam_cc_n = in(idx_OD_0+pparam_sel+1+1);
%     pparam_cc_e = in(idx_OD_0+pparam_sel+2+1);
%     pparam_cc_d = in(idx_OD_0+pparam_sel+3+1);
%     pparam_R = abs(in(idx_OD_0+pparam_sel+4+1));
    if (in(idx_OD_0+pparam_sel+4+1)<0.0)
        pparam_ldir = -1.0;
    else
        pparam_ldir = 1.0;
    end
%     pparam_Chi = in(idx_OD_0+pparam_sel+5+1);
%     pparam_Gam = in(idx_OD_0+pparam_sel+6+1);
    Gam_temp = in(idx_OD_0+pparam_sel+6+1);

    % calculate closest poon loiter circle
    cr_n = in(0+1) - in(idx_OD_0+pparam_sel+1+1);
    cr_e = in(1+1) - in(idx_OD_0+pparam_sel+2+1);
    norm_cr = sqrt( cr_n*cr_n + cr_e*cr_e );
    if (norm_cr>0.1) 
        cr_n_unit = cr_n / norm_cr;
        cr_e_unit = cr_e / norm_cr;
    
    else 
        cr_n_unit = 0.0;
        cr_e_unit = 0.0;
    end
    p_n = abs(in(idx_OD_0+pparam_sel+4+1)) * cr_n_unit + in(idx_OD_0+pparam_sel+1+1);
    p_e = abs(in(idx_OD_0+pparam_sel+4+1)) * cr_e_unit + in(idx_OD_0+pparam_sel+2+1);

    % calculate tangent
    tP_n = pparam_ldir * -cr_e_unit;
    tP_e = pparam_ldir * cr_n_unit;
    
    % angular position
    xi_pos = atan2(cr_e_unit, cr_n_unit);
    
    % angular exit
    xi_exit = in(idx_OD_0+pparam_sel+5+1) - pparam_ldir * 1.570796326794897;
    if (xi_exit>3.141592653589793) 
        xi_exit = xi_exit - 6.283185307179586;
    
    elseif (xi_exit<-3.141592653589793) 
        xi_exit = xi_exit + 6.283185307179586;
    end
    
    % angular travel (back calculated) from exit (0,2pi)
    delta_xi = pparam_ldir * (xi_exit - xi_pos);
    if (delta_xi >= 6.28318530718), delta_xi = 0.0; end;
    if (delta_xi < 0.0), delta_xi = delta_xi + 6.28318530718; end;

    % closest poon nearest spiral leg and tangent down component
    if (abs(in(idx_OD_0+pparam_sel+6+1)) < 0.001) 

        p_d = in(idx_OD_0+pparam_sel+3+1);
        tP_d = 0.0;

    else 

        RtanGam = abs(in(idx_OD_0+pparam_sel+4+1)) * tan(in(idx_OD_0+pparam_sel+6+1));

        % height down from exit
        delta_d_xi = delta_xi * RtanGam;

        % nearest spiral leg
        delta_d_k = round( (in(2+1) - (in(idx_OD_0+pparam_sel+3+1) + delta_d_xi)) / (6.28318530718*RtanGam) ) * 6.28318530718*RtanGam;
        
        % closest poon nearest spiral leg
        p_d = in(idx_OD_0+pparam_sel+3+1) + delta_d_k + delta_d_xi;
        
        % cap end point
        if ((p_d - in(idx_OD_0+pparam_sel+3+1)) * in(idx_OD_0+pparam_sel+6+1) < 0.0) 
            p_d = in(idx_OD_0+pparam_sel+3+1);
            tP_d = 0.0;
            Gam_temp = 0.0;
        
        else 
            tP_d = -sin(in(idx_OD_0+pparam_sel+6+1));
        end
        
    end
    
    if (abs(tP_n)<0.01 && abs(tP_e)<0.01)  % should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        tP_n=1.0;
        tP_e=0.0;
    end
    
    % Normalize tP
    tP_n = tP_n * cos(Gam_temp);
    tP_e = tP_e * cos(Gam_temp);
    
    % feed forward
    phi_ff = atan((n_dot*n_dot+e_dot*e_dot)/in(idx_OD_0+pparam_sel+4+1)/9.81);
    
% LOITER UNLIM
elseif ( pparam_type < 2.5 ) 

    if (in(idx_OD_0+pparam_sel+4+1)<0.0)
        pparam_ldir = -1.0;
    else
        pparam_ldir = 1.0;
    end

    % calculate closest poon loiter circle
    cr_n = in(0+1) - in(idx_OD_0+pparam_sel+1+1);
    cr_e = in(1+1) - in(idx_OD_0+pparam_sel+2+1);
    norm_cr = sqrt( cr_n*cr_n + cr_e*cr_e );
    if (norm_cr>0.1) 
        cr_n_unit = cr_n / norm_cr;
        cr_e_unit = cr_e / norm_cr;
    
    else 
        cr_n_unit = 0.0;
        cr_e_unit = 0.0;
    end
    p_n = abs(in(idx_OD_0+pparam_sel+4+1)) * cr_n_unit + in(idx_OD_0+pparam_sel+1+1);
    p_e = abs(in(idx_OD_0+pparam_sel+4+1)) * cr_e_unit + in(idx_OD_0+pparam_sel+2+1);

    % calculate tangent
    tP_n = pparam_ldir * -cr_e_unit;
    tP_e = pparam_ldir * cr_n_unit;
    
    p_d = in(idx_OD_0+pparam_sel+3+1);
    tP_d = 0.0;
    
    if (abs(tP_n)<0.01 && abs(tP_e)<0.01)  % should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        tP_n=1.0;
        tP_e=0.0;
    end
    
    % feed forward
    phi_ff = atan((n_dot*n_dot+e_dot*e_dot)/in(idx_OD_0+pparam_sel+4+1)/9.81);
end

% end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

t3 = in(1+1)-p_e;
t4 = in(0+1)-p_n;
norm_rp_ne = sqrt(t3*t3+t4*t4);

if norm_rp_ne<0.00001
    rp_n_unit=0;
    rp_e_unit=0;
else
    rp_n_unit = -t4/norm_rp_ne;
    rp_e_unit = -t3/norm_rp_ne;
end

t6 = -in(2+1)+p_d;

sgn_rp = 0.0;
if (t6>0.0) 
    sgn_rp = 1.0;
elseif (t6<0.0) 
    sgn_rp = -1.0;
end

t16 = e_dot*e_dot;
t17 = n_dot*n_dot;
t18 = t16+t17;

e_lat = t4*tP_e-t3*tP_n;

if (t18>1.0) 
    e_b_lat = sqrt(t18)*in(38+1);                               
else 
    e_b_lat = in(38+1)*(1.0/2.0)+in(38+1)*t18*(1.0/2.0);
end
sat_e_lat = abs(e_lat)/e_b_lat;
if (sat_e_lat>1.0), sat_e_lat = 1.0; end;

% t7 = sat_e_lat-1.0;
% t8 = t7*t7;
% t9 = 3.141592653589793*t8*(1.0/2.0);
% t10 = cos(t9);
% t11 = sin(t9);

e_lon = t6;

% if (abs(d_dot)>1.0) 
%     e_b_lon = abs(d_dot)*in(39+1);                               
% else 
%     e_b_lon = in(39+1)*(1.0/2.0)+in(39+1)*abs(d_dot)*(1.0/2.0);
% end
% sat_e_lon = abs(e_lon)/e_b_lon;
% if (sat_e_lon>1.0), sat_e_lon = 1.0; end;

e_b_lon = in(39+1);
% sat_e_lon = abs(e_lon)/e_b_lon;
% if (sat_e_lon>1.0), sat_e_lon = 1.0; end;
% 
% t12 = sat_e_lon-1.0;
% t13 = t12*t12;
% t14 = 3.141592653589793*t13*(1.0/2.0);
% t15 = sin(t14);

% atan2_01 = atan2(rp_e_unit*t10+t11*tP_e, rp_n_unit*t10+t11*tP_n);
% atan2_02 = atan2(e_dot, n_dot);
% eta_lat = atan2_01-atan2_02;
% if (eta_lat>3.141592653589793) {
%     eta_lat = eta_lat - 6.283185307179586;
% }
% elseif (eta_lat<-3.141592653589793) {
%     eta_lat = eta_lat + 6.283185307179586;
% }

% atan2_03 = atan2(-t15*tP_d-sgn_rp*cos(t14), t15*sqrt(tP_e*tP_e+tP_n*tP_n));
% atan2_04 = atan2(-d_dot, sqrt(t18));
% eta_lon = atan2_03-atan2_04;
% if (eta_lon>3.141592653589793) {
%     eta_lon = eta_lon - 6.283185307179586;
% }
% elseif (eta_lon<-3.141592653589793) {
%     eta_lon = eta_lon + 6.283185307179586;
% }

norm_vG_lat = sqrt(t18);
% if (norm_vG_lat<1.0) 
%     eta_lat = sqrt(-(norm_vG_lat-2.0)*(norm_vG_lat-n_dot*(rp_n_unit*t10+t11*tP_n)-e_dot*(rp_e_unit*t10+t11*tP_e))/2.0);
% else 
%     eta_lat = sqrt((norm_vG_lat*(1.0/2.0)-n_dot*(rp_n_unit*t10+t11*tP_n)*(1.0/2.0)-e_dot*(rp_e_unit*t10+t11*tP_e)*(1.0/2.0))/norm_vG_lat);
% end
% % tt3 = (rp_n_unit*t10+t11*tP_n)*e_dot-(rp_e_unit*t10+t11*tP_e)*n_dot;
% % if (tt3<0.0)
% %     tt3=-1.0;
% % else
% %     tt3=1.0;
% % end
% % if (norm_vG_lat<0.00001) 
% %     eta_lat = 0.0;
% % elseif (norm_vG_lat<1.0) 
% %     VG1 = 0.5-0.5*cos(3.141592653589793*norm_vG_lat);
% %     VG1_VG = VG1/norm_vG_lat;
% %     
% %     tt1 = VG1*(rp_e_unit*t10+t11*tP_e)-e_dot*VG1_VG;
% %     tt2 = VG1*(rp_n_unit*t10+t11*tP_n)-n_dot*VG1_VG;
% % 
% %     eta_lat = (tt3*sqrt(tt1*tt1+tt2*tt2)/2.0);
% % else
% %     tt1 = norm_vG_lat*(-rp_e_unit*t10+t11*tP_e)-e_dot;
% %     tt2 = norm_vG_lat*(rp_n_unit*t10+t11*tP_n)-n_dot;
% %     
% %     eta_lat = (tt3*sqrt(tt1*tt1+tt2*tt2)/norm_vG_lat/2.0);
% % end

thetal=-sat_e_lat*(sat_e_lat-2.0);
ln=(1.0-thetal)*tP_n/sqrt(tP_e*tP_e+tP_n*tP_n)+thetal*rp_n_unit;
le=(1.0-thetal)*tP_e/sqrt(tP_e*tP_e+tP_n*tP_n)+thetal*rp_e_unit;
atan2_01 = atan2(le, ln);
atan2_02 = atan2(e_dot, n_dot);
eta_lat = atan2_01-atan2_02;
if (eta_lat>3.141592653589793)
    eta_lat = eta_lat - 6.283185307179586;
elseif (eta_lat<-3.141592653589793)
    eta_lat = eta_lat + 6.283185307179586;
end



norm_vG_lon = sqrt(t18+d_dot*d_dot);
% if (norm_vG_lon<1.0) 
%     eta_lon = sqrt(-(norm_vG_lon-2.0)*(norm_vG_lon-t15*sqrt(tP_e*tP_e+tP_n*tP_n)*norm_vG_lat-d_dot*(-t15*tP_d-sgn_rp*cos(t14)))/2.0);
% else 
%     eta_lon = sqrt((norm_vG_lon*(1.0/2.0)-norm_vG_lat*t15*sqrt(tP_e*tP_e+tP_n*tP_n)*(1.0/2.0)-d_dot*(-t15*tP_d-sgn_rp*cos(t14))*(1.0/2.0))/norm_vG_lon);
% end

% d_dot_max = 3.0;
% sat_e_lon = e_lon/in(39+1);
% if (sat_e_lon>1.0), sat_e_lon = 1.0; end;
% if (sat_e_lon<-1.0), sat_e_lon = -1.0; end;
% eta_lon = sqrt((sat_e_lon*d_dot_max - d_dot)/d_dot_max);

% ddot_clmb = 3.5;
% ddot_sink = 1.5;
% if (e_lon<0.0)
%     sat_e_lon = e_lon/in(39+1)/-ddot_clmb;
%     if (sat_e_lon>1.0), sat_e_lon = 1.0; end;
%     thetal = -sat_e_lon*(sat_e_lon-2.0);
%     eta_lon = (-ddot_clmb*thetal-d_dot)/(ddot_clmb+ddot_sink);
% else
%     sat_e_lon = e_lon/in(39+1)/ddot_sink;
%     if (sat_e_lon>1.0), sat_e_lon = 1.0; end;
%     thetal = -sat_e_lon*(sat_e_lon-2.0);
%     eta_lon = (ddot_sink*thetal-d_dot)/(ddot_clmb+ddot_sink);
% end

ddot_sp = norm_vG_lon*tP_d/sqrt(tP_e*tP_e+tP_n*tP_n);
ddot_clmb = 3.5;
ddot_sink = 1.5;
if (ddot_sp>1.4), ddot_sp=1.4; end;
if (ddot_sp<-3.4), ddot_sp=-3.4; end;
if (e_lon<0.0)
    sat_e_lon = abs(e_lon/in(39+1)/(-ddot_clmb-ddot_sp));
    if (sat_e_lon>1.0), sat_e_lon=1.0; end;
    thetal = -sat_e_lon*(sat_e_lon-2.0);
    eta_lon = ((-ddot_clmb-ddot_sp)*thetal + ddot_sp - d_dot)/(ddot_clmb+ddot_sink);
else
    sat_e_lon = abs(e_lon/in(39+1)/(ddot_sink-ddot_sp));
    if (sat_e_lon>1.0), sat_e_lon=1.0; end;
    thetal = -sat_e_lon*(sat_e_lon-2.0);
    eta_lon = ((ddot_sink-ddot_sp)*thetal + ddot_sp - d_dot)/(ddot_clmb+ddot_sink);
end

t19 = alpha-in(35+1)+in(37+1);
t20 = 1.0/(in(37+1)*in(37+1));
t21 = -alpha+in(36+1)+in(37+1);

if (alpha>(in(35+1)-in(37+1))) 
    a_soft=(t19*t19)*t20;

elseif (alpha>(in(36+1)+in(37+1))) 
    a_soft=0.0;

else 
    a_soft=t20*(t21*t21);
end

% outputs */

out(0+1) = eta_lat;
out(1+1) = eta_lon;
out(2+1) = Vsafe;
out(3+1) = in(8+1);
out(4+1) = in(9+1);
out(5+1) = in(10+1);
out(6+1) = a_soft;
out(7+1) = in(11+1)*(-4.143016944939305)+in(13+1)*4.143016944939305;
out(8+1) = in(13+1);
if (phi_ff>0.523598775598299), phi_ff = 0.523598775598299;
elseif (phi_ff<-0.523598775598299), phi_ff = -0.523598775598299; end;
out(9+1) = in(14+1) - (0.5+0.5*cos(sat_e_lat*3.141592653589793))*phi_ff;
out(10+1) = in(15+1);

aux = [e_lat,e_lon, p_n,p_e,p_d,tP_n,tP_e,tP_d, e_lat/e_b_lat,e_lon/e_b_lon, a_soft, n_dot,e_dot,d_dot, in(32+1),in(33+1),in(34+1), (0.5+0.5*cos(sat_e_lat*3.141592653589793))*phi_ff, le,ln];

