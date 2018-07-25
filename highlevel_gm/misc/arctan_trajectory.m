clear;
clc;

len = 11;
[e,n,h] = meshgrid(linspace(-100,100,len),linspace(-100,100,len),linspace(-100,100,len));

chi_t = deg2rad(0);
Gamma_t = deg2rad(-80);
n_t=0;
e_t=0;
h_t=0;

e_lat = 20;
e_lon = 20;

chi_app = atan(1/e_lat * ( (n-n_t)*sin(chi_t) - (e-e_t)*cos(chi_t) ));

ne = n*cos(chi_t) - e*sin(chi_t);
ne_t = n_t*cos(chi_t) - e_t*sin(chi_t);
Gamma_app0 = atan(1/e_lon*( (ne-ne_t)*sin(Gamma_t) - (h-h_t)*cos(Gamma_t) ));
Gamma_app0 = Gamma_app0(:);
for i=1:length(Gamma_app0)
    if Gamma_app0(i) < Gamma_t
        Gamma_app(i) = 2/pi*min(abs(-pi/2-Gamma_t),abs(Gamma_t+Gamma_app0(i)))*Gamma_app0(i);
    else
        Gamma_app(i) = 2/pi*min(abs(pi/2-Gamma_t),abs(Gamma_t+Gamma_app0(i)))*Gamma_app0(i);
    end
end
Gamma_app = reshape(Gamma_app,[len,len,len]);

Gamma_l = Gamma_app + Gamma_t;
% Gamma_l1 = Gamma_l(:);
% Gamma_l1(Gamma_l1>pi/2) = pi/2;
% Gamma_l1(Gamma_l1<-pi/2) = -pi/2;
% Gamma_l = reshape(Gamma_l1,[len,len,len]);
chi_l = chi_app + chi_t;
le_n = cos(Gamma_l).*cos(chi_l);
le_e = cos(Gamma_l).*sin(chi_l);
le_h = sin(Gamma_l);

sc = 10;

figure('color','w'); hold on; grid on;
quiver3(e,n,h,sc*le_e,sc*le_n,sc*le_h,'autoscale','off');

view(90,0)