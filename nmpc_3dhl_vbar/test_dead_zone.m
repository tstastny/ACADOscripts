

figure(1);
hold on; grid on;

N=(11--7)*10+1;
x = linspace(-7,11,N);

ap=8;
am=-4;
kap=2;
kam=2;

% axp=x+-ap+(0.5*(ap-am));%kap;
% axm=x+-am-(0.5*(ap-am));%kam;
% 
% lap=(1./(1+exp(-kap*(x-ap)))) .* axp;
% lam=1./(1+exp(kam*(x-am))) .* axm;
% 
% la=(lap + lam)/(ap-am)*2;

% lap=1./(1+exp(-kap*(x-ap)));
% lam=1./(1+exp(kam*(x-am)));

% lap=(x-ap).^2/(2*(dx));
% lam=(x-am).^2/(2*(dx));

% for j = 0.5:0.5:4
%     
% kap=j;
% kam=j;
delta_a=3;
for i=1:length(x)
    
    dlap=kap/2;
    dlam=kam/2;
    
%     if x(i)>ap
%         la1(i)=(x(i)-ap)*dlap+1;
%     elseif x(i)>am
%         la1(i)=2/(1+exp(-kap*(x(i)-ap)))+2/(1+exp(kam*(x(i)-am)));
%     else
%         la1(i)=-(x(i)-am)*dlam+1;
%     end
    if x(i)>(ap-delta_a)
        la1(i)=((x(i)-(ap-delta_a))/delta_a).^2;
    elseif x(i)>(am+delta_a)
        la1(i)=0;
    else
        la1(i)=((x(i)-(am+delta_a))/delta_a).^2;
    end
end
plot(x,la1)
% end

% plot(x,axp,x,axm,x,lap,x,lam)
% plot(x,2*la1)
% % plot(x,0.1*tan((x-0.5*(ap-am)-am)*pi/(ap-am+6)))
ylim([0,1])


%%
syms ap am kap kam a
axp=a+-ap+(1/2*(ap-am));%kap;
axm=a+-am-(1/2*(ap-am));%kam;

lap=(1/(1+exp(-kap*(a-ap)))) * axp;
lam=1/(1+exp(kam*(a-am))) * axm;

la=(lap + lam)/(ap-am)*2;