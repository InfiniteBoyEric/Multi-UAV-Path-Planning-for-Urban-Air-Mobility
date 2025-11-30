function [cauchy] = CauchyRand(m,c)

cauchy = c*tan(pi*(rand()-0.5)) + m;

end
%___________________________________________________%
%[Used in the Strategy 3: The movement of the bee to build the next cel (Eq. 7) Section 2.2.3] %
