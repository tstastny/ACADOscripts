% check gradient on path following cost

psi_set = deg2rad(linspace(170,190,201))';
e_set = linspace(-10,10,201)';
v = 13;
Wvn = 1;
Wve = 1;
We = 1;
vnsp = v;
vesp = 0;

J = zeros(length(psi_set),length(e_set));
for i = 1:length(psi_set)
    for j = 1:length(e_set)
        
        vn = v*cos(psi_set(i));
        ve = v*sin(psi_set(i));
        
        J(i,j) = Wvn*(vnsp-vn) + Wve*(vesp-ve) + We*e_set(j);
        
    end
end

mesh(rad2deg(psi_set),e_set,J')
xlabel('\psi');
ylabel('e');