function [Best_score,Best_pos, Convergence_curve] = GGO(SearchAgents_no, Max_iter, lb, ub, dim, fobj)

% Parameter initialization
a = 2;
c = 2;
b = 1;

% Population initialization
X = initialization(SearchAgents_no, dim, ub, lb);

% Fitness initialization
fitness = zeros(1, SearchAgents_no);
for i = 1:SearchAgents_no
    fitness(i) = fobj(X(i, :));
end

% Find optimum
[Best_score, best_idx] = min(fitness);
Best_pos = X(best_idx, :);

% Convergence curve initialization
Convergence_curve = zeros(1, Max_iter);

% Search agents initialization
n1 = round(SearchAgents_no * 0.5);
n2 = SearchAgents_no - n1;

% Record for break loop
no_improvement_count = 0;
prev_best_score = Best_score;

for t = 1:Max_iter
    % Refresh parameters
    a = 2 - t * (2 / Max_iter);
    z = 1 - (t / Max_iter)^2;
    
    % Randomly permute population
    rand_indices = randperm(SearchAgents_no);
    X = X(rand_indices, :);
    fitness = fitness(rand_indices);
    
    for i = 1:n1
        r1 = rand(); r2 = rand(); r3 = rand(); r4 = rand(); r5 = rand();
        A = 2 * a * r1 - a;
        C = 2 * r2;
        
        w1 = 2 * rand(); w2 = 2 * rand(); w3 = 2 * rand(); w4 = 2 * rand();
        
        if mod(t, 2) == 0
            if r3 < 0.5
                if abs(A) < 1
                    X(i, :) = Best_pos - A .* abs(C .* Best_pos - X(i, :));
                else
                    paddle_indices = randperm(SearchAgents_no, 3);
                    X_paddle1 = X(paddle_indices(1), :);
                    X_paddle2 = X(paddle_indices(2), :);
                    X_paddle3 = X(paddle_indices(3), :);
                    
                    X(i, :) = w1 * X_paddle1 + z * w2 * (X_paddle2 - X_paddle3) + ...
                             (1 - z) * w3 * (X(i, :) - X_paddle1);
                end
            else
                l = 2 * rand() - 1;
                X(i, :) = w4 * abs(Best_pos - X(i, :)) .* exp(b * l) .* cos(2 * pi * l) + ...
                         (2 * w1 * (r4 + r5)) * Best_pos;
            end
        else
            D = rand(1, dim);
            w = rand();
            X_flock1 = X(randi(SearchAgents_no), :);
            X(i, :) = X(i, :) + D .* (1 + z) * w .* (X(i, :) - X_flock1);
        end
    end    

    [~, sorted_indices] = sort(fitness);
    indices_of_min3 = sorted_indices(1:3);
    
    for i = (n1 + 1):SearchAgents_no
        if mod(t, 2) == 0
            X_sentry1 = X(indices_of_min3(1), :);
            X_sentry2 = X(indices_of_min3(2), :);
            X_sentry3 = X(indices_of_min3(3), :);
            
            r1_1 = rand(); r1_2 = rand(); r1_3 = rand();
            r2_1 = rand(); r2_2 = rand(); r2_3 = rand();            
            A1 = 2 * a * r1_1 - a;
            A2 = 2 * a * r1_2 - a;
            A3 = 2 * a * r1_3 - a;
            C1 = 2 * r2_1;
            C2 = 2 * r2_2;
            C3 = 2 * r2_3;
            
            X1 = X_sentry1 - A1 .* abs(C1 .* X_sentry1 - X(i, :));
            X2 = X_sentry2 - A2 .* abs(C2 .* X_sentry2 - X(i, :));
            X3 = X_sentry3 - A3 .* abs(C3 .* X_sentry3 - X(i, :));
            
            X(i, :) = (X1 + X2 + X3) / 3;
        else
            D = rand(1, dim);
            w = rand();
            X_flock1 = X(indices_of_min3(1), :);
            X(i, :) = X(i, :) + D .* (1 + z) * w .* (X(i, :) - X_flock1);
        end
    end
    
    % Check the boundary
    for i = 1:SearchAgents_no
        X(i, :) = max(X(i, :), lb);
        X(i, :) = min(X(i, :), ub);
    end
    
    % Re-calculate Fitness
    for i = 1:SearchAgents_no
        fitness(i) = fobj(X(i, :));
    end
    
    % Re-find optimum
    [current_best, best_idx] = min(fitness);
    if current_best < Best_score
        Best_score = current_best;
        Best_pos = X(best_idx, :);
    end
    
    % Dynamically adjust exploring group and developing group
    if abs(Best_score - prev_best_score) < 1e-10
        no_improvement_count = no_improvement_count + 1;
    else
        no_improvement_count = 0;
    end
    
    % Expand exploring group if no improvement in 3 iterations
    if no_improvement_count >= 3
        n1 = min(n1 + 1, SearchAgents_no - 1);
        n2 = SearchAgents_no - n1;
        no_improvement_count = 0;
    else
        % Else gradually contract exploring group and expand developing group
        if n1 > 1
            n1 = max(1, n1 - 1);
            n2 = SearchAgents_no - n1;
        end
    end
    
    prev_best_score = Best_score;
    
    % Record convergence curve
    Convergence_curve(t) = Best_score;
    
end

fprintf('GGO Algorithm completed. Best Score = %.6e\n', Best_score);
end

function X = initialization(SearchAgents_no, dim, ub, lb)
    % Initialize the boundary
    Boundary_no = size(ub, 2);
    
    if Boundary_no == 1
        X = rand(SearchAgents_no, dim) .* (ub - lb) + lb;
    else
        for i = 1:dim
            ub_i = ub(i);
            lb_i = lb(i);
            X(:, i) = rand(SearchAgents_no, 1) .* (ub_i - lb_i) + lb_i;
        end
    end
end
