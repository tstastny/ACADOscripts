% loop testing

clear; clc;

NX = 15; NU = 5; NY = 12; NYN = 7;

%% lsq

out = 0:NY;

for i=0:NX-1
    for j=0:NY-1
        out(NY+j*NX+i+1)= NY+j*NX+i;
    end
end
for i=0:NU-1
    for j=0:NY-1
        out(NY+NY*NX+j*NU+i+1) =  NY+NY*NX+j*NU+i;
    end
end

%% lsqN

out = 0:NYN;

for i=0:NX-1
    for j=0:NYN-1
        out(NYN+j*NX+i+1)= NYN+j*NX+i;
    end
end