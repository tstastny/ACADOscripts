clear; clc;

xi_wrap = [0:0.1:pi, -pi:0.1:pi, -pi:0.1:pi, -pi:0.1:pi/2, pi/2:-0.1:-pi];

xi_wrap_k_1 = xi_wrap(1);

xi_unwrap_k_1 = xi_wrap(1);

for k = 1:length(xi_wrap)
    
    delta = xi_wrap(k) - xi_wrap_k_1;
    if delta < -pi, delta = delta + 2*pi; end
    if delta > pi, delta = delta - 2*pi; end
    
    xi_unwrap_k = xi_unwrap_k_1 + delta;
    
    xi_unwrap(k) = xi_unwrap_k;
    xi_unwrap_k_1 = xi_unwrap_k;
    xi_wrap_k_1 = xi_wrap(k);
end

figure; hold on; plot(xi_wrap); plot(xi_unwrap)