
clear; clc;

syms an;
syms ae;
syms ad;
syms bn;
syms be;
syms bd;
syms n;
syms e;
syms d;
syms n_dot;
syms e_dot;
syms d_dot;

abn = bn - an;
abe = be - ae;
abd = bd - ad;
norm_ab2 = abn*abn + abe*abe + abd*abd;
norm_ab = sqrt(norm_ab2);
abn_unit = abn / norm_ab;
abe_unit = abe / norm_ab;
abd_unit = abd / norm_ab;

pan = an - n;
pae = ae - e;
pad = ad - d;
cx = abe_unit*pad - pae*abd_unit;
cy = -(abn_unit*pad - pan*abd_unit);
cz = abn_unit*pae - pan*abe_unit;
et = sqrt( cx^2 + cy^2 + cz^2 );

et_dot = jacobian(et,n) * n_dot + jacobian(et,e) * e_dot + jacobian(et,d) * d_dot;