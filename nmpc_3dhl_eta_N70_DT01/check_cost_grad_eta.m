

clear; clc;

T_b_lat = 1.5;

chi_set = linspace(-pi,pi,1001);

pparam_n = 0;
pparam_e = 0;
pparam_d = 0;
pparam_chi = 0;
pparam_gam = 0;

in = [0,30,0];

eps_chi = 0.00001;

for i = 1:length(chi_set)
    
    m_chi = chi_set(i)-eps_chi;
    p_chi = chi_set(i)+eps_chi;
    
    dchi = [chi_set(i),m_chi,p_chi];
%     de = [
    
    for j = 1:length(dchi)
        
        n_dot = 13*cos(dchi(j));
        e_dot = 13*sin(dchi(j));
        d_dot = 0;
    
        tP_n = cos(pparam_gam)*cos(pparam_chi);
        tP_e = cos(pparam_gam)*sin(pparam_chi);
        tP_d = -sin(pparam_gam);

        % dot product
        dot_tP_bp = tP_n*(in(0+1) - pparam_n) + tP_e*(in(1+1) - pparam_e) + tP_d*(in(2+1) - pparam_d);

        % poon track
        p_n = pparam_n + dot_tP_bp * tP_n;
        p_e = pparam_e + dot_tP_bp * tP_e;
        p_d = pparam_d + dot_tP_bp * tP_d;

        t3 = in(1+1)-p_e;
        t4 = in(0+1)-p_n;
        t5 = t3*t3;
        t6 = t4*t4;
        t7 = t5+t6;
        t9 = tP_e*tP_e;
        t10 = tP_n*tP_n;
        t11 = t9+t10;
        t12 = 1.0/sqrt(t11);
        t13 = e_dot*e_dot;
        t14 = n_dot*n_dot;
        t15 = t13+t14;

        norm_rp_ne = sqrt(t7);
        if (norm_rp_ne < 0.00001) 
            rp_n_unit = 0.0;
            rp_e_unit = 0.0;
        else 
            rp_n_unit = -t4/norm_rp_ne;
            rp_e_unit = -t3/norm_rp_ne;
        end

        % LATERAL-DIRECTIONAL GUIDANCE

        e_lat = t4*t12*tP_e-t3*t12*tP_n;
        norm_vG_lat = sqrt(t15);

        if (norm_vG_lat>1.0) 
            e_b_lat = T_b_lat*norm_vG_lat;                               
        else 
            e_b_lat = T_b_lat*(1.0/2.0)+T_b_lat*t15*(1.0/2.0);
        end
        sat_e_lat = abs(e_lat)/e_b_lat;
        if (sat_e_lat>1.0), sat_e_lat = 1.0; end;

        t16 = sat_e_lat-2.0;
        t17 = sat_e_lat*t16;
        t18 = t17+1.0;

        atan2_01 = atan2(-rp_e_unit*sat_e_lat*t16+t12*t18*tP_e, -rp_n_unit*sat_e_lat*t16+t12*t18*tP_n);
        atan2_02 = atan2(e_dot, n_dot);
        eta_lat = atan2_01-atan2_02;
        if (eta_lat>3.141592653589793) 
            eta_lat = eta_lat - 6.283185307179586;
        elseif (eta_lat<-3.141592653589793) 
            eta_lat = eta_lat + 6.283185307179586;
        end
        
        eta3(j) = eta_lat;
        
    end
    if chi_set(i)==pi/2
        stoppp=1;
    end
    
    eta_rec(i)=eta3(1);
    deta_rec(i)=(eta3(3)-eta3(2))/2/eps_chi;
    
end

figure('color','w'); hold on; grid on; box on;
plot(rad2deg(chi_set),rad2deg(eta_rec));
xlabel('\chi')
ylabel('\eta')

figure('color','w'); hold on; grid on; box on;
plot(rad2deg(chi_set),rad2deg(deta_rec));
xlabel('\chi')
ylabel('\eta dot')