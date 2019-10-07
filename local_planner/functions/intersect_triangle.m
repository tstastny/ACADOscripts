function [ret, d_occ, p_occ] = intersect_triangle(r0, v_ray, p1, p2, p3) 
% following: "Fast, Minimum Storage Ray/Triangle Intersection",
 % Moeller et. al., Journal of Graphics Tools, Vol.2(1), 1997
 %/
EPSILON = 0.00001;
ret = 0;
d_occ = 0;
p_occ = zeros(3,1);
 
% NOTE: all vectors in here are E,N,U

% find vectors for two edges sharing p1
e1 = [p2(1) - p1(1), p2(2) - p1(2), p2(3) - p1(3)];
e2 = [p3(1) - p1(1), p3(2) - p1(2), p3(3) - p1(3)];

% begin calculating determinant - also used to calculate U parameter
pvec = cross(v_ray, e2);

% we don't test for backwards culling here as, assuming this ray casting
% algorithm is properly detecting the first occluding triangles, we should
% not run into a case of a true "backwards" facing triangle (also the
% current BR and TL definitions have opposite vertex spin definitions..
% should probably change that in the future.. could maybe avoid a few
% divisions of the determinant)

% if the determinant is near zero, ray lies in the triangle plane
det = dot(e1, pvec);
if (det > -EPSILON && det < EPSILON)
    return;
end

% divide the determinant (XXX: could possibly find a way to avoid this until the last minute possible..)
inv_det = 1.0 / det;

% calculate distance from p1 to ray origin
tvec = [r0(1) - p1(1), r0(2) - p1(2), r0(3) - p1(3)];

% calculate u parameter and test bounds
u = dot(tvec, pvec) * inv_det;
if (u < 0.0 || u > 1.0)
    return;
end

% prepare to test v parameter
qvec = cross(tvec, e1);

% calculate v parameter and test bounds
v = dot(v_ray, qvec) * inv_det;
if (v < 0.0 || u + v > 1.0) 
    return;
end

% calculate d_occ, scale parameters, ray intersects triangle
d_occ = dot(e2, qvec) * inv_det;

% calculate and return intersection point
one_u_v = (1 - u - v);
p_occ(1) = one_u_v * p1(1) + u * p2(1) + v * p3(1);
p_occ(2) = one_u_v * p1(2) + u * p2(2) + v * p3(2);
p_occ(3) = one_u_v * p1(3) + u * p2(3) + v * p3(3);

ret = 1;
