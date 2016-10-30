clear;
NNN=1001;
ve=linspace(-1,1,NNN);
vn=linspace(-1,1,NNN);

eps_v=0.5;
k=1;
for i=1:NNN
    for j = 501;%1:NNN
        veplot(k)=ve(j);
        vnplot(k)=vn(i);
        vv=norm([ve(j),vn(i)]);
        if vv<eps_v
            if vv<0.0001
                vbar_n(k) = 0;
                vbar_e(k) = 0;
            else
                vbar_n(k) = ((sin(vv/eps_v*pi/2)/vv-1)+1)*vn(i);
                vbar_e(k) = ((sin(vv/eps_v*pi/2)/vv-1)+1)*ve(j);
            end
            
            vbar(k) = norm([vbar_n(k),vbar_e(k)]);%+sin(vv*pi/2)-vv;
        else
            vbar_n(k) = vn(i)/vv;
            vbar_e(k) = ve(j)/vv;
            vbar(k)=1;
        end
        
        k=k+1;
    end
end
plot(vnplot,vbar,'.')
% plot3(veplot,vnplot,vbar,'.')