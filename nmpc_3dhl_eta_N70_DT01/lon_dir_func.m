

clear; clc;

ddot_sink = 1.5*1.1;
ddot_clmb = 3.5*1.1;

ddot_sp = 0;

e_b_d = 2;

e = linspace(-ddot_clmb*5/1.1*2,ddot_sink*5/1.1*2,501);
ddot = linspace(-ddot_clmb*2,ddot_sink*2,501);

for i=1:length(e)
    
    if ddot_sp<-ddot_clmb/1.1, ddot_sp=-ddot_clmb/1.1; end;
    if ddot_sp>ddot_sink/1.1, ddot_sp=ddot_sink/1.1; end;
    
    if e(i)<0
        sat_e = abs(e(i)/e_b_d/(-ddot_clmb-ddot_sp));
        if sat_e>1.0, sat_e=1.0; end;
        thetal = -sat_e*(sat_e-2);
        delta_ddot = (-ddot_clmb-ddot_sp)*thetal;
        ddot_guide(i)=delta_ddot + ddot_sp;
%         ev(i,:) = sign(delta_ddot + ddot_sp - ddot).*sqrt(abs(delta_ddot + ddot_sp - ddot)/(ddot_clmb+ddot_sink));
%         ev(i,:) = abs(sign(delta_ddot + ddot_sp - ddot).*sqrt(abs(delta_ddot + ddot_sp - ddot)/(ddot_clmb+ddot_sink)));
        ev(i,:) = abs((delta_ddot + ddot_sp - ddot)/(ddot_clmb+ddot_sink));
%         ev_temp = abs((delta_ddot + ddot_sp - ddot)/(ddot_clmb+ddot_sink));
%         ev(i,:) = -ev_temp.*(ev_temp-2);
    else
        sat_e = abs(e(i)/e_b_d/(ddot_sink-ddot_sp));
        if sat_e>1.0, sat_e=1.0; end;
        thetal = -sat_e*(sat_e-2);
        delta_ddot = (ddot_sink-ddot_sp)*thetal;
        ddot_guide(i)=delta_ddot + ddot_sp;
%         ev(i,:) = sign(delta_ddot + ddot_sp - ddot).*sqrt(abs(delta_ddot + ddot_sp - ddot)/(ddot_clmb+ddot_sink));
%         ev(i,:) = abs(sign(delta_ddot + ddot_sp - ddot).*sqrt(abs(delta_ddot + ddot_sp - ddot)/(ddot_clmb+ddot_sink)));
        ev(i,:) = abs((delta_ddot + ddot_sp - ddot)/(ddot_clmb+ddot_sink));
%         ev_temp = abs((delta_ddot + ddot_sp - ddot)/(ddot_clmb+ddot_sink));
%         ev(i,:) = -ev_temp.*(ev_temp-2);
    end
end

figure('color','w'); hold on; grid on; box on;
% mesh(e,ddot,zeros(length(e),length(ddot)));
mesh(e,ddot,ev');

xlabel('e')
ylabel('ddot')
zlabel('ev')

colorbar;

figure('color','w'); hold on; grid on; box on;
plot(e,ddot_guide/(ddot_clmb+ddot_sink));

xlabel('e')
ylabel('ddot_{guide}')

% colorbar;

% %%
% 
% clear; clc;
% 
% syms ddot_clmb e ddot_sp ddot ddot_sink ebd
% 
% sat_e_clmb = e/ebd/ddot_clmb;
% thetal_clmb = -sat_e_clmb*(sat_e_clmb-2);
% ev_clmb = sqrt(abs(ddot_clmb*thetal_clmb+ddot_sp-ddot)/(ddot_clmb+ddot_sink));
% 
% sat_e_sink = e/ebd/ddot_sink;
% thetal_sink = -sat_e_sink*(sat_e_sink-2);
% ev_sink = sqrt(abs(ddot_sink*thetal_sink+ddot_sp-ddot)/(ddot_clmb+ddot_sink));
% 
% j_e_clmb = jacobian(ev_clmb,e);
% pretty(simplify(subs(j_e_clmb,e,0)))
% 
% j_e_sink = jacobian(ev_sink,e);
% pretty(simplify(subs(j_e_sink,e,0)))
% 
% j_d_clmb = jacobian(ev_clmb,ddot);
% pretty(simplify(subs(j_d_clmb,[ddot ddot_sp],[0 0])))
% 
% j_d_sink = jacobian(ev_sink,ddot);
% pretty(simplify(subs(j_d_sink,[ddot ddot_sp],[0 0])))






