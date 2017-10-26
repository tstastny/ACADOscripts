

clear; clc;

ddot_max=-3.5;
ddot_nom=0;
ddot_min=1.5;
ddot_sp=linspace(ddot_max,ddot_min,1001);
vmax=15;
vnom=13.5;
vmin=12;

dv(1:sum(ddot_sp<0)) = (vmax-vnom)*(ddot_sp(ddot_sp<0)/abs(ddot_max)).^2;
dv(sum(ddot_sp<0):length(ddot_sp)) = -(vnom-vmin)*(ddot_sp(sum(ddot_sp<0):length(ddot_sp))/abs(ddot_min)).^2;

plot(ddot_sp,dv+vnom)