clear;clc;


load jac_135_line.mat

NX = 13;
NU = 3;
NY = 10;

Q_scale     = [1 1, 1 1 1, 1, 50*pi/180 50*pi/180 50*pi/180, 1];
R_scale     = [1 30*pi/180 15*pi/180 1 5*pi/180 5*pi/180];
Q_output    = [150 180 5 5 5 5 30 30 5 10]./Q_scale.^2;
R_controls  = [100 5 40 1 1 1]./R_scale.^2;

Qy = diag(Q_output);
R = diag(R_controls(1:3));

C = dh_dx(1:NY,1:NX);

A = df_dxu(1:NX,1:NX);
B = df_dxu(1:NX,(NX+1):end);


sel_x = [4:5,9:11];
sel_u = 1:NU;
sel_y = 5:NY-1;

C_sel = C(sel_y,sel_x);
Qy_sel = Qy(sel_y,sel_y);
Qx_sel = C_sel'*Qy_sel*C_sel;
A_sel = A(sel_x,sel_x);
B_sel = B(sel_x,sel_u);
R_sel = R(sel_u,sel_u);

[Px,L,G] = care(A_sel,B_sel,Qx_sel,R_sel);

A1 = C_sel * A_sel * C_sel';
E1 = C_sel * C_sel';
B1 = C_sel * B_sel;
S1 = zeros(length(sel_y),length(sel_u));

[Py_sel,L1,G1] = care(A1,B1,Qy_sel,R,S1,E1);

Py = Qy;
Py(sel_y,sel_y) = Py_sel;
Py(Py<1e-2) = 0;

