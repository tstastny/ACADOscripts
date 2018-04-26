% plot vector field correction

clear; clc;

norm_vG_lat = 13.5;
T_b_lat0 = 1;
ll=501;
etaP_set = linspace(-pi-pi/2,-pi/2,ll);
e_set = linspace(-(norm_vG_lat^2/9.81/tand(30)+norm_vG_lat*T_b_lat0),norm_vG_lat^2/9.81/tand(30)+norm_vG_lat*T_b_lat0,ll);
% e_set = linspace(0,norm_vG_lat*T_b_lat0,ll);
% e_set = linspace(-norm_vG_lat*T_b_lat0,norm_vG_lat*T_b_lat0,ll);
T_b_lat0 = 1;

for i = 1:ll
    for j = 1:ll

        n_dot = norm_vG_lat*cos(etaP_set(i));
        e_dot = norm_vG_lat*sin(etaP_set(i));
        e_lat = e_set(j);
        
        too_close = abs(e_lat/(T_b_lat0*norm_vG_lat + norm_vG_lat*norm_vG_lat/5.6638));
        sig_e = 0.0;
        if (too_close<1.0)
            sig_e = cos(1.570796326794897*too_close);
            sig_e = sig_e*sig_e;
        end

        sig_P = 0.0;
        dot_tP_vG = 1*n_dot + 0*e_dot;
        if (dot_tP_vG<0.0), sig_P = dot_tP_vG*dot_tP_vG/norm_vG_lat/norm_vG_lat; end;

        T_b_lat = T_b_lat0*(1.0 + 0.0*sig_P*sig_e);

        if (norm_vG_lat>1.0)
            e_b_lat = T_b_lat*norm_vG_lat;                               
        else
            e_b_lat = T_b_lat*(1.0/2.0)+T_b_lat*t15*(1.0/2.0);
        end

        % if (norm_vG_lat>1.0) 
        %     e_b_lat = in(38+1)*norm_vG_lat;                               
        % else 
        %     e_b_lat = in(38+1)*(1.0/2.0)+in(38+1)*t15*(1.0/2.0);
        % end
        sat_e_lat = abs(e_lat)/e_b_lat;
        if (sat_e_lat>1.0), sat_e_lat = 1.0; end;

        t16 = sat_e_lat-2.0;
        t17 = sat_e_lat*t16;
        t18 = t17+1.0;

        atan2_01 = atan2(-1*sign(-e_lat)*sat_e_lat*t16+t18*0, -0*sat_e_lat*t16+t18*1);
        atan2_02 = atan2(e_dot, n_dot);
        eta_lat = atan2_01-atan2_02;
        if (eta_lat>3.141592653589793) 
            eta_lat = eta_lat - 6.283185307179586;
        elseif (eta_lat<-3.141592653589793) 
            eta_lat = eta_lat + 6.283185307179586;
        end
        
        T_b_lat_(i,j) = T_b_lat;
        eta_lat_(i,j) = eta_lat;
        ln_(i,j) = -0*sat_e_lat*t16+t18*1;
        le_(i,j) = -1*sign(-e_lat)*sat_e_lat*t16+t18*0;
        sig_etaP_(i,j) = sig_P;
        sig_e_(i,j) = sig_e;
        
    end
end

%%

figure('color','w'); hold on; grid on; box on;

contourf(rad2deg(etaP_set),e_set,abs(rad2deg(eta_lat_')),19,'LineStyle','none');
% mesh(rad2deg(etaP_set),e_set,sig_etaP_);
% mesh(rad2deg(etaP_set),e_set,sig_e_');
% mesh(rad2deg(etaP_set),e_set,rad2deg(T_b_lat_'));

%%
% figure('color','w'); hold on; grid on; box on;
% max_e = max(e_set);
% max_etaP = max(rad2deg(etaP_set));
% [x,y]=meshgrid(rad2deg(etaP_set),e_set);
% quiver(x,y,le_'*max_etaP,ln_'*max_e,'MaxHeadSize',0.01)

        