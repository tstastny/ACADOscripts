function [p1] = set_position_setpoint(ned,VW,VA,p,Ts_step,N)

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