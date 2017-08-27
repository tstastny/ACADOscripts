%
% check objectives of flight data
% / / / / / / / / / / / / / / / /

clear; clc;

% sysvec
% load sysvec_20161108_nmpc_f02.mat;

% filter out non- auto mode
% sysvec1 = sysvector(sysvector.ASLD_mode==5,:);
% time1 = time(sysvector.ASLD_mode==5);

load ins.mat;
l_ = size(x0,1);

l_out = 16;
l_aux = 2;

% calculate costs
out=zeros(l_,l_out);
aux=zeros(l_,l_aux);
for i = 1:l_
    
    in = [x0(i,:),u1(i,:),od(i,:)]';
    
    [out(i,:),aux(i,:)] = lsq_obj_eval(in);
    
%     out(i,:)=out0;
%     aux(i,:)=aux0;
    
end

u_ref = [0.3*ones(l_,1),zeros(l_,2)];
y_ref = [zeros(l_,5),V_ref,zeros(l_,4),u_ref,u_ref0];


%% PLOT

figure('color','w');

hh(1) = subplot(4,1,1); hold on; grid on;
for i = 1:5
    plot(Q(:,i).*(out(:,i)-y_ref(:,i)).^2);
end
legend('e_ne','e_d','v_n','v_e','v_d')

hh(2) = subplot(4,1,2); hold on; grid on;
for i = 6:10
    plot(Q(:,i).*(out(:,i)-y_ref(:,i)).^2);
end
legend('V','p','q','r','\alpha')

hh(3) = subplot(4,1,3); hold on; grid on;
for i = 11:13
    plot(Q(:,i).*(out(:,i)-y_ref(:,i)).^2);
end
legend('u_T','\phi_{ref}','\theta_{ref}')

hh(4) = subplot(4,1,4); hold on; grid on;
for i = 14:16
    plot(Q(:,i).*(out(:,i)-y_ref(:,i)).^2);
end
legend('\Delta u_T','\Delta \phi_{ref}','\Delta \theta_{ref}')

%%

figure('color','w');

hh(5) = subplot(3,1,1); hold on; grid on;
plot(u1(:,1)); ylabel('u_T')
hh(6) = subplot(3,1,2); hold on; grid on;
plot(rad2deg(u1(:,2)));
plot(rad2deg(x0(:,7))); legend('\phi_{ref}''\phi')
hh(7) = subplot(3,1,3); hold on; grid on;
plot(rad2deg(u1(:,3)));
plot(rad2deg(x0(:,8))); legend('\theta_{ref}','\theta')

%%

linkaxes(hh,'x')


