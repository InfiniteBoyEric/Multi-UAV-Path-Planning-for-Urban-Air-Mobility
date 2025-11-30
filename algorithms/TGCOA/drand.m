function val= drand() %Uniform Distribution%

RAND_MAX=2147483647;

val= (rand()+1.0)/(RAND_MAX+1.0);

end
%___________________________________________________%
%[Used in the Strategy 3: The movement of the bee to build the next cel (Eq. 7) Section 2.2.3] %

