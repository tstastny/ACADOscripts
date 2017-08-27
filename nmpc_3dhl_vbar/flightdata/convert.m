
for i=1:length(n)
    x_n(i,:) = eval(n{i});
    x_e(i,:) = eval(e{i});
    x_d(i,:) = eval(d{i});
    x_V(i,:) = eval(V{i});
    x_gam(i,:) = eval(gamma1{i});
    x_xi(i,:) = eval(xi{i});
    x_phi(i,:) = eval(p{i});
    x_theta(i,:) = eval(q{i});
    x_p(i,:) = eval(r{i});
    x_q(i,:) = eval(n{i});
    x_r(i,:) = eval(n{i});
    x_throt(i,:) = eval(n{i});
    x_xsw(i,:) = eval(n{i});
end


%%

for i=1:length(uT)
    
    uT_horiz=eval(uT{i});
    phi_ref_horiz=eval(phi_ref{i});
    theta_ref_horiz=eval(theta_ref{i});
    
    u0(i,:) = [uT_horiz(1), phi_ref_horiz(1), theta_ref_horiz(1)];
    u1(i,:) = [uT_horiz(2), phi_ref_horiz(2), theta_ref_horiz(2)];
    
end

%%

for i = 1:length(Qdiag)
    
    Q_pre(i,:) = eval(Qdiag{i});
    
end

%%

Q = interp1(t_nmpcparams,Q_pre,t_acadovars,'previous');

%%

u_ref = [y_uT0,y_phi_ref0,y_theta_ref0];

%%

V_ref = yV;


