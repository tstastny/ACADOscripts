
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
VG_set = linspace(0,1,101);
for i=1:length(VG_set)
    VG = VG_set(i);
    vG = VG*[cos(chi);sin(chi)];
    if VG>=1
        e_lat(i,:) = sqrt((VG - (tP(1)*vG(1,:)+tP(2)*vG(2,:)))/VG/2);
    else
%         e_lat(i,:) = sqrt((VG-2)*(-VG + (tP(1)*vG(1,:)+tP(2)*vG(2,:)))/2);
        e_lat(i,:) = sqrt(-(VG-2)*(VG - (tP(1)*vG(1,:)+tP(2)*vG(2,:)))/2);
    end
end

mesh(rad2deg(chi),VG_set,e_lat)
xlabel('\chi')
ylabel('VG')
zlabel('e')

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

