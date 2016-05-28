
clear; clc;

llo = [47.602866, 8.534066];
psio = 0;
href = 0;
lla = [47.602795, 8.533831, href];

f = 1/298.25642;
R = 6378136.6;
 
dlat = (lla(1)-llo(1))*pi/180;
dlon = (lla(2)-llo(2))*pi/180;

ff = 2*f-f^2;
RN = R/sqrt(1-ff*sind(llo(1))^2);
RM = RN*(1-ff)/(1-ff*sind(llo(1))^2);

dN = dlat/atan(1/RM);
dE = dlon/atan(1/(RN*cosd(lla(1))));


pos = lla2flat(lla,llo,psio,href);


pos(1)
dN
pos(2)
dE