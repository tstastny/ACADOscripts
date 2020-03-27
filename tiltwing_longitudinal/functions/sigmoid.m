function sigm = sigmoid(xi)
k_slope = 45;
sigm = 1./(1+exp(-k_slope*(xi-0.23)));
end

