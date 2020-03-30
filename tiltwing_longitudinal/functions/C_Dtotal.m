function c_Dtotal = C_Dtotal(xi)
c_Dtotal = C_Dflatplate(xi).*sigmoid(xi)+C_Dairfoil(xi).*(1-sigmoid(xi));
end

