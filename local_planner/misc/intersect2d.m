function p_i = intersect2d(r, v0, a, b)

ar = r - a;
ab = b - a;

p_i = dot(v0 + ar, ab) * ab / sum(ab.^2) + a;


