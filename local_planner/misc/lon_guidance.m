% longitudinal guidance
% >> vertical velocity setpoint (AIR MASS RELATIVE!)

clear;
clc;

len = 501;

eps_vd = 0.01; % such that the logic doesnt break at max sp!
vd_sink = 2;
vd_clmb = -4;
k1 = 2*pi;
k2 = 1;
T_d = 5;

wd = -2;

vd_sp0set = (vd_clmb:0.5:vd_sink) - wd;
len_vd_sp = length(vd_sp0set);

vd_sp0set(vd_sp0set<vd_clmb) = vd_clmb;
vd_sp0set(vd_sp0set>vd_sink) = vd_sink;

dset = -(vd_sink-vd_clmb)*T_d*linspace(-1, 1, len);

vsp = zeros(len_vd_sp,len);
for i = 1:len_vd_sp
    
    vd_sp0 = vd_sp0set(i);    
    
    for j = 1:len
        
        d = dset(j);
        
        if d<0
            delta_vd = vd_sink + eps_vd - vd_sp0;
        else
            delta_vd = vd_clmb - eps_vd - vd_sp0;
        end
        if abs(delta_vd)<eps_vd*1.1
            stoppp=1;
        end
        
        e_b = T_d * delta_vd;
        e_p = abs(d/e_b);
        
        vsp(i,j) = 2/pi*atan(e_p*k1) * delta_vd + vd_sp0;
    end
end

colors_ = parula(len_vd_sp);

figure('color','w');
hold on; grid on; box on;

plot([0, 0], [vd_clmb - eps_vd, vd_sink + eps_vd], '--k');
plot([min(dset), max(dset)], [0, 0], '--k');

for i = 1:len_vd_sp
    plot(0, vd_sp0set(i), 'o', 'color', colors_(i,:));
    plot(dset, vsp(i,:), 'color', colors_(i,:));
end

xlabel('d [m]');
ylabel('v_{d,sp} [m/s]');
    
    
   