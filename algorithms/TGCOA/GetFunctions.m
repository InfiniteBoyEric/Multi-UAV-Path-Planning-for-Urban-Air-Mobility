%========================================================================
%
% Benchmarks;
%_________________________________________________________________________%
%lb is the lower bound: lb= [lb_1, lb_2,.....,lb_d]
%up is the uppper bound: ub= [ub_1,ub_2,.....,ub_d]
%dim is the number of variables (dimension of the problem)
function [lb,ub,dim,fobj] = GetFunctions(F)
switch F
    case 'F1'     %
        fobj = @F1;%Brown Function   
        lb= -1;
        ub= 4;
        dim= 30;
    
    case 'F2'  %
        fobj = @F2;
        lb=-600;
        ub=600;
        dim=30;
           
      case 'F3' % Schwefel 2.20 Function
        fobj = @F3;
        lb=-100;
        ub=100;
        dim=30; % dim  n   
            
end
end

%*********************************************************

function o = F1(x)
n = size(x, 2);  
    o = 0;
    
    x = x .^ 2;
    for i = 1:(n-1)
        o = o + x(:, i) .^ (x(:, i+1) + 1) + x(:, i+1).^(x(:, i) + 1);
    end
end

function o = F2(x)
    n = size(x, 2);
    
    sumcomp = 0;
    prodcomp = 1;
    
    for i = 1:n
        sumcomp = sumcomp + (x(:, i) .^ 2);
        prodcomp = prodcomp .* (cos(x(:, i) / sqrt(i)));
    end
    
    o = (sumcomp / 4000) - prodcomp + 1;
end

function o = F3(x)
    o = sum(abs(x), 2);
end

%========================================================================

