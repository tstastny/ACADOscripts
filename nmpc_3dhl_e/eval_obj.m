function [out] = eval_obj(in,Ns)

% optimized intermediate calculations */

alpha = -in(4+1)+in(7+1);

t2 = alpha-in(18+1)+in(20+1);
t3 = 1.0/(in(20+1)*in(20+1));
t4 = -alpha+in(19+1)+in(20+1);
t5 = in(0+1)-in(21+1);
t6 = in(1+1)-in(22+1);

if (alpha>(in(18+1)-in(20+1)))
    a_soft=(t2*t2)*t3;
elseif (alpha>(in(19+1)+in(20+1)))
    a_soft=0.0;
else
    a_soft=t3*(t4*t4);
end

% outputs */

out(0+1) = sqrt(t5*t5+t6*t6);
out(1+1) = -in(2+1)+in(23+1);
out(2+1) = in(3+1);
out(3+1) = in(8+1);
out(4+1) = in(9+1);
out(5+1) = in(10+1);
out(6+1) = a_soft;
out(7+1) = in(11+1)*(-4.143016944939305)+in(12+1)*4.143016944939305;
out(8+1) = in(12+1);
out(9+1) = in(13+1);
out(10+1) = in(14+1);