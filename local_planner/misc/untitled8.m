clear; clc;

cT0=0.1326;
cT1=-0.1152;

np = linspace(0, 9820/60, 101)';
vp = linspace(10, 25, 101)';

u_n = linspace(0,1,11)';
vp1 = linspace(10, 25, 11)';

dprop=0.28;
vmin = 10;
vmax = 25;
n_max = 9820/60;

n_T0_vmin = (-cT1*vmin/dprop + sqrt((cT1*vmin/dprop)^2 - cT0*4))/2/cT0;
n_T0_vmax = (-cT1*vmax/dprop + sqrt((cT1*vmax/dprop)^2 - cT0*4))/2/cT0;


for i = 1:length(vp)
    
    T(:,i) = 1.15*np.^2.*0.28^4.*(cT0 + cT1.*vp(i)./np/0.28);
  
end
k=0;
np1=zeros(length(u_n)*length(vp1),1); T2=np1; vp2=np1;
for i = 1:length(vp1)
    for j = 1:length(u_n)

        k = k+1;
        
        vp2(k) = vp1(i);
        
        sig_v = (vp1(i) - vmin)/(vmax - vmin);
        np1(k) = (n_T0_vmin + u_n(j)*(n_max - n_T0_vmin)) * (1 - sig_v) + ...
            (n_T0_vmax + u_n(j)*(n_max - n_T0_vmax)) * sig_v;

        T2(k) = 1.15*np1(k).^2.*0.28^4.*(cT0 + cT1.*vp1(i)./np1(k)/0.28);
    end
end
figure; hold on; grid on;
mesh(np, vp, T');
scatter3(np1, vp2, T2, 20, [1 0 0]);


