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

p_n = 0.0;
p_e = 0.0;
p_d = 0.0;
tP_n = 1.0;
tP_e = 0.0;
tP_d = 0.0;

pparam_type = in(idx_OD_0+pparam_sel+1);

% LINE SEGMENT
if ( pparam_type < 0.5 ) 

    % calculate tangent
    tP_n = cos(in(idx_OD_0+pparam_sel+6+1))*cos(in(idx_OD_0+pparam_sel+5+1));
    tP_e = cos(in(idx_OD_0+pparam_sel+6+1))*sin(in(idx_OD_0+pparam_sel+5+1));
    tP_d = -sin(in(idx_OD_0+pparam_sel+6+1));
    
    % dot product
    dot_tP_bp = tP_n*(in(0+1) - in(idx_OD_0+pparam_sel+1+1)) + tP_e*(in(1+1) - in(idx_OD_0+pparam_sel+2+1)) + tP_d*(in(2+1) - in(idx_OD_0+pparam_sel+3+1));
    
    % point on track
    p_n = in(idx_OD_0+pparam_sel+1+1) + dot_tP_bp * tP_n;
    p_e = in(idx_OD_0+pparam_sel+2+1) + dot_tP_bp * tP_e;
    p_d = in(idx_OD_0+pparam_sel+3+1) + dot_tP_bp * tP_d;
    
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

    % calculate closest point on loiter circle
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

    % closest point on nearest spiral leg and tangent down component
    if (abs(in(idx_OD_0+pparam_sel+6+1)) < 0.001) 

        p_d = in(idx_OD_0+pparam_sel+3+1);
        tP_d = 0.0;

    else

        RtanGam = abs(in(idx_OD_0+pparam_sel+4+1)) * tan(in(idx_OD_0+pparam_sel+6+1));

        % height down from exit
        delta_d_xi = delta_xi * RtanGam;

        % nearest spiral leg
        delta_d_k = round( (in(2+1) - (in(idx_OD_0+pparam_sel+3+1) + delta_d_xi)) / (6.28318530718*RtanGam) ) * 6.28318530718*RtanGam;
        
        % closest point on nearest spiral leg
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
    
% LOITER UNLIM
elseif ( pparam_type < 2.5 ) 

    if (in(idx_OD_0+pparam_sel+4+1)<0.0)
        pparam_ldir = -1.0;
    else
        pparam_ldir = 1.0;
    end

    % calculate closest point on loiter circle
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
end

% end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

t3 = e_dot*e_dot;
t4 = n_dot*n_dot;
t5 = alpha-in(35+1)+in(37+1);
t6 = 1.0/(in(37+1)*in(37+1));
t7 = -alpha+in(36+1)+in(37+1);

norm_vG_lat = sqrt(t3+t4);
if (norm_vG_lat<1.0) 
    e_v_lat = sqrt(norm_vG_lat-n_dot*tP_n-e_dot*tP_e);
else
    e_v_lat = sqrt((norm_vG_lat*(1.0/2.0)-n_dot*tP_n*(1.0/2.0)-e_dot*tP_e*(1.0/2.0))/norm_vG_lat);
end

norm_vG_lon = sqrt(t3+t4+d_dot*d_dot);
if (norm_vG_lon<1.0) 
    e_v_lon = sqrt(norm_vG_lon-sqrt(tP_e*tP_e+tP_n*tP_n)*norm_vG_lat-d_dot*tP_d);
else
    e_v_lon = sqrt((norm_vG_lon*(1.0/2.0)-norm_vG_lat*sqrt(tP_e*tP_e+tP_n*tP_n)*(1.0/2.0)-d_dot*tP_d*(1.0/2.0))/norm_vG_lon);
end

if (alpha>(in(35+1)-in(37+1))) 
    a_soft=(t5*t5)*t6;
elseif (alpha>(in(36+1)+in(37+1))) 
    a_soft=0.0;
else
    a_soft=t6*(t7*t7);
end

% outputs */

out(0+1) = -tP_n*(in(1+1)-p_e)+tP_e*(in(0+1)-p_n);
out(1+1) = -in(2+1)+p_d;
out(2+1) = e_v_lat;
out(3+1) = e_v_lon;
out(4+1) = Vsafe;
out(5+1) = in(8+1);
out(6+1) = in(9+1);
out(7+1) = in(10+1);
out(8+1) = a_soft;
out(9+1) = in(11+1)*(-4.143016944939305)+in(13+1)*4.143016944939305;
out(10+1) = in(13+1);
out(11+1) = in(14+1);
out(12+1) = in(15+1);

aux = [p_n,p_e,p_d,tP_n,tP_e,tP_d, a_soft, n_dot,e_dot,d_dot, in(32+1),in(33+1),in(34+1)];

