function c_Ltotal = C_Ltotal(xi)
%xi = linspace(-0.2,pi,1000);
c_Ltotal = C_Lflatplate(xi).*sigmoid(xi)+C_Lairfoil(xi).*(1-sigmoid(xi));
%plot(xi,c_L)
end