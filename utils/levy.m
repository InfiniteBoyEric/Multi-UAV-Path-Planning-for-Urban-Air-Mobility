function sigma=levy(d) 
    b = 1.5;
    s = (gamma(1+b)*sin(pi*b/2) / (gamma((1+b)/2)*b*2^((b-1)/2)))^(1/b);
    u = randn(1, d)*s;
    v = randn(1, d);
    sigma = u./abs(v) .^ (1/b);
end
