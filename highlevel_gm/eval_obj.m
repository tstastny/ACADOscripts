function [out,aux] = eval_obj(in)

% path tangent unit vector  
tP_n_bar = cos(in(17)); 
tP_e_bar = sin(in(17)); 
 
% "closest" point on track 
tp_dot_br = tP_n_bar*(in(1)-in(13)) + tP_e_bar*(in(2)-in(14)); 
tp_dot_br_n = tp_dot_br*tP_n_bar; 
tp_dot_br_e = tp_dot_br*tP_e_bar; 
p_n = in(13) + tp_dot_br_n; 
p_e = in(14) + tp_dot_br_e; 
p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar; 
p_d = in(15) - p_lat*tan(in(16)); 
 
% directional error 
v_c_gamma = in(9)*cos(in(4)); 
v_n = v_c_gamma*cos(in(5)); 
v_e = v_c_gamma*sin(in(5)); 
tp_dot_vg = tP_n_bar*(v_n+in(10)) + tP_e_bar*(v_e+in(11)); 
 
% terrain 
h_ter = 10.0; 
h_ter_normalized = (in(3) + h_ter)/in(18);
if h_ter_normalized <= 0
    h_ter_normalized = 0.0;
end
 
% state output 
out(1) = (in(2)-p_e)*cos(in(17)) - (in(1)-p_n)*sin(in(17)); 
out(2) = p_d - in(3); 
out(3) = tp_dot_vg*0.5+0.5; 
out(4) = h_ter_normalized*h_ter_normalized; 
 
% control output 
out(5) = in(7); % gamma ref 
out(6) = in(8); % mu ref 
out(7) = (in(7) - in(4))/1; % gamma dot 
out(8) = (in(8) - in(6))/0.7; % mu dot 

aux = 0;