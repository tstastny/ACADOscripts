
clear;
figure(1);
hold on; grid on;

e=linspace(-50,50,2001);
e0=30;
k_e_dir=0*6*1/15;
% k_e_t=6/140;

Jep = (1./(1+exp(-k_e_dir*(e-e0))));
Jem = (1./(1+exp(k_e_dir*(e+e0))));
Jdir= Jep+Jem;

% Je = (2./(1+exp(-k_e_t*(e)))-1);
for i = 1:length(e)
    if abs(e(i))<e0
        e_e0 = sin(e(i)/e0*pi/2);
    else
        e_e0 = 1*sign(e(i));
    end
    Je(i)=e_e0;
%     Je(i) = Jep(i)-Jem(i)+(1-Jdir(i)).*e_e0;
end
% Je = (Jep-Jem)+(1-Jdir).*(e/(e0));

% k_e_Gam=2;
% e_d=e;
% e_d_co=30;
% JGam = 1./(1+exp(-k_e_Gam*(e_d-e_d_co))) + 1./(1+exp(k_e_Gam*(e_d+e_d_co)));
% plot(e_d,JGam)

% subplot(2,1,1); hold on; grid on;
% plot(e,Jep+Jem)
% 
% subplot(2,1,2); hold on; grid on;
% plot(e,-pi/4-((Jep+Jem)*pi/2+(1-(Jep+Jem))*pi/4))

plot(e,Jdir)
plot(e,Je)
% plot(e,Je.^2)


%%
clear;
N=50000;
r=rand(N,1);
for i=1:N
    atan_tic=tic;
    atan(r(i));
    atan_rec(i)=toc(atan_tic);
    exp_tic=tic;
    exp(r(i));
    exp_rec(i)=toc(exp_tic);
end

% plot(1:N,atan_rec,1:N,exp_rec)
mean(atan_rec)
mean(exp_rec)
    