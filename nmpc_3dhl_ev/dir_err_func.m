
clear; clc;

tP = [1;0];
chi = linspace(-pi,pi,501);
VG_set = [0 0.01 0.05 0.1 0.5 1];
for i=1:length(VG_set)
    VG = VG_set(i);
    vG = VG*[cos(chi);sin(chi)];
    if VG>=1
        e_lat(i,:) = sqrt((VG - (tP(1)*vG(1,:)+tP(2)*vG(2,:)))/VG/2);
    else
        e_lat(i,:) = sqrt((VG - (tP(1)*vG(1,:)+tP(2)*vG(2,:)))/2);
    end
end
plot(rad2deg(chi), e_lat)
legend('0','0.01','0.05','0.1','0.5','1')

%%

clear; clc;

tP = [1;0];
chi = linspace(-pi,pi,101);
VG_set = linspace(0,1.5,101);
for i=1:length(VG_set)
    VG = VG_set(i);
    vG = VG*[cos(chi);sin(chi)];
    if VG>=1
%         e_lat(i,:) = sign(tP(1)*vG(2,:)-tP(2)*vG(1,:)).*sqrt((VG - (tP(1)*vG(1,:)+tP(2)*vG(2,:)))/VG/2);
        e_lat(i,:) = (sign(tP(1)*vG(2,:)-tP(2)*vG(1,:)).*sqrt((VG*tP(1)-vG(1,:)).^2+(VG*tP(2)-vG(2,:)).^2)/VG/2);%.^2;
    else
%         e_lat(i,:) = sign(tP(1)*vG(2,:)-tP(2)*vG(1,:)).*sqrt((VG - (tP(1)*vG(1,:)+tP(2)*vG(2,:)))/2);
%         e_lat(i,:) = (sign(tP(1)*vG(2,:)-tP(2)*vG(1,:)).*-(VG-2).*sqrt((VG*tP(1)-vG(1,:)).^2+(VG*tP(2)-vG(2,:)).^2)/2);%.^2;
        VG1 = 0.5-0.5*cos(pi*VG);
        if VG>0
            VG1_VG=VG1/VG;
        else
            VG1_VG=0;
        end
        e_lat(i,:) = (sign(tP(1)*vG(2,:)-tP(2)*vG(1,:)).*sqrt((VG1*tP(1)-vG(1,:)*VG1_VG).^2+(VG1*tP(2)-vG(2,:)*VG1_VG).^2)/2);%.^2
    end
%     e_lat0(i,:) = sqrt((VG - (tP(1)*vG(1,:)+tP(2)*vG(2,:)))/2);
end

figure('color','w','name','3d'); hold on; grid on; box on;
mesh(rad2deg(chi),VG_set,e_lat)
xlabel('\chi')
ylabel('VG')
zlabel('e')

% figure('color','w','name','2d'); hold on; grid on; box on;
% plot(rad2deg(chi),e_lat0(1:10:length(VG_set),:))
% xlabel('\chi')
% ylabel('e')

%%
clear; clc;

tP = [1;0];
chi = linspace(-pi,pi,501);
VG_set = [1];
for i=1:length(VG_set)
    VG = VG_set(i);
    vG = VG*[cos(chi);sin(chi)];
    if VG>=1
        e_lat0(i,:) = ((VG - (tP(1)*vG(1,:)+tP(2)*vG(2,:)))/VG/2);
        e_lat(i,:) = sqrt((VG - (tP(1)*vG(1,:)+tP(2)*vG(2,:)))/VG/2);
        e_lat2(i,:) = ((VG - (tP(1)*vG(1,:)+tP(2)*vG(2,:)))/VG/2).^(1/4);
    else
        e_lat0(i,:) = sqrt((VG - (tP(1)*vG(1,:)+tP(2)*vG(2,:))));
        e_lat(i,:) = sqrt((VG - (tP(1)*vG(1,:)+tP(2)*vG(2,:))));
        e_lat2(i,:) = ((VG - (tP(1)*vG(1,:)+tP(2)*vG(2,:)))).^(1/4);
    end
end
plot(rad2deg(chi), e_lat0, rad2deg(chi), e_lat, rad2deg(chi), e_lat2)
legend('0','sqrt','1/4')

