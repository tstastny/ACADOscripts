function [ret, t, p] = intersect_triangle(r, v_ray, p1, p2, p3)
% following: "Fast, Minimum Storage Ray/Triangle Intersection", Moeller et.
% al., Journal of Graphics Tools, Vol.2(1), 1997

% NOTE: all vectors in here are E,N,U

epsilon = 0.000001;

t = 0.0;
p = zeros(3,1);

% find vectors for two edges sharing p1
e1 = p2 - p1;
e2 = p3 - p1;

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
if (det > -epsilon && det < epsilon)
    ret = false;
    return;
end
inv_det = 1.0 / det;

% calculate distance from p1 to ray origin
tvec = r - p1;

% calculate u parameter and test bounds
u = dot(tvec, pvec) * inv_det;
if (u < 0.0 || u > 1.0)
    ret = false;
    return;
end

% prepare to test v parameter
qvec = cross(tvec, e1);

% calculate v parameter and test bounds
v = dot(v_ray, qvec) * inv_det;
if (v < 0.0 || u + v > 1.0)
    ret = false;
    return;
end

% calculate t, scale parameters, ray intersects triangle
t = dot(e2, qvec) * inv_det;

% calculate and return intersection point
p = (1 - u - v) * p1 + u * p2 + v * p3;
ret = true;
