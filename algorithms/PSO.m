function [fMin, bestX, convergenceCurve] = PSO(pop, maxgen, lb, ub, dim, fobj)
    w_max = 0.9;
    w_min = 0.4;
    c1 = 2.0;
    c2 = 2.0;
    vmax = 0.2 * (ub - lb);
    vmin = -vmax;
    
    lb = lb .* ones(1, dim);
    ub = ub .* ones(1, dim);
    
    x = initialization_single(pop, dim, ub, lb);
    v = vmin + (vmax - vmin) .* rand(pop, dim);
    
    pbest = x;
    pbest_fit = inf(pop, 1);
    
    for i = 1:pop
        pbest_fit(i) = fobj(x(i, :));
    end
    
    [gbest_fit, idx] = min(pbest_fit);
    gbest = pbest(idx, :);
    
    convergenceCurve = zeros(1, maxgen);
    
    for iter = 1:maxgen
        w = w_max - (w_max - w_min) * iter / maxgen;
        
        for i = 1:pop
            v(i, :) = w * v(i, :) + c1 * rand(1, dim) .* (pbest(i, :) - x(i, :)) + c2 * rand(1, dim) .* (gbest - x(i, :));
            v(i, :) = max(min(v(i, :), vmax), vmin);
            x(i, :) = x(i, :) + v(i, :);
            x(i, :) = max(min(x(i, :), ub), lb);
            
            fitness = fobj(x(i, :));
            
            if fitness < pbest_fit(i)
                pbest(i, :) = x(i, :);
                pbest_fit(i) = fitness;
                
                if fitness < gbest_fit
                    gbest = x(i, :);
                    gbest_fit = fitness;
                end
            end
        end
        
        convergenceCurve(iter) = gbest_fit;
    end
    
    fMin = gbest_fit;
    bestX = gbest;
end
