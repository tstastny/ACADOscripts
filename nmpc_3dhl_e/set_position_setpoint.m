function [p1] = set_position_setpoint(ned,VW,VA,p,Ts_step,N)

% e_lat = norm(p(1:2) - ned(1:2));
% e_lon = abs(ned(3));
% vg_lat = norm(ned_dot(1:2));
% vg_lon = abs(ned_dot(3));
% 
% e_b_lat = vg_lat*Ts_step*N;
% if e_b_lat < 1, e_b_lat=1; end;
% e_b_lon = vg_lon*Ts_step*N;
% if e_b_lon < 1, e_b_lon=1; end;
% 
% W=[1/e_b_lat^2;1/e_b_lon^2];

% if e_lat == 0
%     e_lat_unit = zeros(2,1);
% else
%     e_lat_unit = (p(1:2)-ned(1:2))/norm(p(1:2)-ned(1:2));
% end
% p1_lat = p(1:2) + e_lat_unit * vg_lat * Ts_step * N;
% 
% if e_lon == 0
%     e_lon_unit = 0;
% else
%     e_lon_unit = (p(3)-ned(3))/norm(p(3)-ned(3));
% end
% p1_lon = p(3) + e_lon_unit * vg_lon * Ts_step * N;

l_hor = (VA+VW) * Ts_step * N;

e = norm(p-ned);
if e < 0.001;
    e_unit = zeros(3,1);
else
    e_unit = (p-ned)/e;
end

if e<l_hor
    p1 = p;
else
    p1 = ned + e_unit * l_hor;
end
