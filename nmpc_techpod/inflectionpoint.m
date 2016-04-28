
%inflection
clear;clc;
cLparams = [0.212657549371211;10.8060289182568;-46.8323561880705;60.6017115061355];
alpha = linspace(-5,25,500)*pi/180;

cL=cLparams(1)+cLparams(2)*alpha+cLparams(3)*alpha.^2+cLparams(4)*alpha.^3;

% cLparams2 = flip(polyfit(alpha,cL,5),2);
% cLparams2 = [0.1;9;-4.70800000000000;-90.0261000000000];
% cL2=cLparams2(1)+cLparams2(2)*alpha+cLparams2(3)*alpha.^2+cLparams2(4)*alpha.^3+cLparams2(5)*alpha.^4+cLparams2(6)*alpha.^5;

alpha_i = -2*cLparams(3)/6/cLparams(4);
alpha2 = alpha(alpha>=alpha_i);
cLa_i = cLparams(2)+2*cLparams(3)*alpha_i+3*cLparams(4)*alpha_i^2;
cL_i = cLparams(1)+cLparams(2)*alpha_i+cLparams(3)*alpha_i.^2+cLparams(4)*alpha_i.^3;
cL2 = cLa_i*alpha2+cL_i-cLa_i*alpha_i;

cL3 = [cL(alpha<alpha_i), cL2];
% cLparams3 = flip(polyfit(alpha,cL,2),2);
% cL4=cLparams3(1)+cLparams3(2)*alpha+cLparams3(3)*alpha.^2;%+cLparams3(4)*alpha.^3;

cL2 = cLa_i*alpha+cL_i-cLa_i*alpha_i;

lgstc = 1./(1+exp(-100*(alpha-alpha_i)));
cL4=cL.*(1-lgstc)+cL2.*lgstc;

figure('color','w'); hold on; grid on;
plot(alpha*180/pi,cL3)
plot(alpha*180/pi,cL4);
