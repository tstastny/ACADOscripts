clear; clc;

syms dis x1 x2 y1 y2 x y f11 f12 f21 f22;


fxy_bl = 1/(dis*dis) * [x2 - x, x - x1] * [f11 f12; f21 f22] * [y2 - y; y - y1];

fxy_bl_x = jacobian(fxy_bl, x)
fxy_bl_y = jacobian(fxy_bl, y)