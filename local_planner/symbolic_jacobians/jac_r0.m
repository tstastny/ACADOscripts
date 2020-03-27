function out = jac_r0(gamma,k_r_offset,v,v_ray_e,v_ray_h,v_ray_n,v_rel,xi)
%JAC_R0
%    OUT = JAC_R0(GAMMA,K_R_OFFSET,V,V_RAY_E,V_RAY_H,V_RAY_N,V_REL,XI)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    07-Feb-2020 17:02:40

t2 = cos(gamma);
t3 = sin(gamma);
t4 = cos(xi);
t5 = sin(xi);
out = [0.0,0.0,0.0,k_r_offset.*v_rel.*(t3.*v_ray_h+t2.*t5.*v_ray_e+t2.*t4.*v_ray_n).*2.0,k_r_offset.*v.*v_rel.*(-t2.*v_ray_h+t3.*t5.*v_ray_e+t3.*t4.*v_ray_n).*-2.0,k_r_offset.*t2.*v.*v_rel.*(t4.*v_ray_e-t5.*v_ray_n).*2.0];