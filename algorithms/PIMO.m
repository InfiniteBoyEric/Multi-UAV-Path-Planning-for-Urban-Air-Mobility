%%
% Projection-Iterative-Methods-based Optimizer: A novel metaheuristic
% algorithm for continuous optimization problems and feature selection
%                                                                                                     
%  Developed in MATLAB R2020a                                                                 
%                                                                                                     
% You can simply define your cost function in a seperate file and load its handle to fobj 
% The initial parameters that you need are:
%__________________________________________
% fobj = @YourCostFunction
% dim = number of your variables
% Max_iter = Maximum number of iterations
% N = number of search agents
% lb=[lb1,lb2,...,lbn] where lbn is the lower bound of variable n
% ub=[ub1,ub2,...,ubn] where ubn is the upper bound of variable n
% If all the variables have equal lower bound you can just
% define lb and ub as two single numbers
% 2025-05-05
%______________________________________________________________________________________________
function [Best_Score, Best_Pos, CG_curve] = PIMO(N, Max_iter, lb, ub, dim, fobj)
    Position = initialization(N, dim, ub, lb);  
    Fitness = zeros(N, 1);                    
    epsilon = 1e-6;
    alphabet = 1 : N;
    %% Record the initial optimal solution and fitness
    for i = 1:N
        Fitness(i) = fobj(Position(i,:));      
    end
    [~, Ind] = sort(Fitness);     
    Best_Score = Fitness(Ind(1));
    Best_Pos = Position(Ind(1),:);
    
    CG_curve = zeros(1, Max_iter);% Store convergence information
    %% Main optimization loop
    for it = 1:Max_iter
        delta = sin(pi/2 * (1 - (2*it / Max_iter)).^5); % eq.(12)
        a = (1 - it/Max_iter) * rand(1, dim);
        for i = 1:N
            rr1 = rand;rr2 = rand;
            if rr1 >= rr2
                Pr =  (Fitness.^2)/((norm(Fitness)^2)+eps);
                try
                    P = randsrc(1, 2, [alphabet; Pr']);
                catch
                    P =randi([1, N], 1, 2);
                end
                P1 = P(1);P2 = P(2);
            else
                P = Ind(1:2);
                P1 = P(1);P2 = P(2);
            end
            r = rand;
            J = Get_jacobian(fobj, Position(P1, :), epsilon); % Generalised Jacobi matrix
            if 3*rand >= 2*rand
                % eq.(14)
                grad = (r * (Position(P1, :) - Best_Pos) + (1-r) * (Position(P2, :) - Best_Pos)) / 2;
                pos_n1 = Position(i,:) - delta * grad;
            else
                % eq.(15)
                grad = (r * (Position(P2, :) - Best_Pos) + (1-r) * (Position(P1, :) - Best_Pos)) / 2;
                pos_n1 = Position(i,:) + delta * J * grad';
            end
            newpos1 = max(min(pos_n1, ub), lb);
            fitt = fobj(newpos1);
            if fitt < Fitness(i)
                Fitness(i)= fitt;
                Position(i,:) = newpos1;
            end
            
            if rand > rand
                P = randperm(N, 2); % eq.(13)
                P1 = P(1);P2 = P(2);
                if 3*rand >= 2*rand
                    % eq.(14)
                    grad = (r * (Position(P1, :) - Best_Pos) + (1-r) * (Position(P2, :) - Best_Pos)) / 2;
                    pos_n2 = Position(i,:) - delta * grad;
                else
                    % eq.(15)
                    grad = (r * (Position(P2, :) - Best_Pos) + (1-r) * (Position(P1, :) - Best_Pos)) / 2;
                    pos_n2 = Position(i,:) + delta * J * grad';
                end
                newpos2 = max(min(pos_n2, ub), lb);
                fitt = fobj(newpos2);
                if fitt < Fitness(i)
                    Fitness(i)= fitt;
                    Position(i,:) = newpos2;
                end
            end
                        
            for j = 1 : dim
                r1 = 1+rand;r2 = 1+rand; % eq.(16)
                pho_1 = r1 * Position(i, :) + (1-r1) * Best_Pos + r2 * (Position(i, :) - Best_Pos); % eq.(17)
                pho_2 = Position(i,:) + a.* (newpos1 - Best_Pos); % eq.(18)
                pos_n3 = Position(i,:);
                if rand/j > rand
                    pos_n3(j) = pho_1(j);
                else
                    pos_n3(j) = pho_2(j);
                end
                newpos3 = max(min(pos_n3, ub), lb);
                fitt = fobj(newpos3);
                if fitt < Fitness(i)
                    Fitness(i)= fitt;
                    Position(i,:) = newpos3;
                end
            end     
        end
        
        if rand <= 1/2*(tanh(9*it/Max_iter-5)+1)
            pos_n4 = zeros(1, dim);
            d=rand()*(1-it/Max_iter)^2;
            Step_length=levy(N, dim,1.5);
            Elite=repmat(Best_Pos, N, 1);
            r=rand;
            for i=1:N
                for j=1:dim
                    pos_n4(j)=r*Elite(i,j)+(1-r)*Step_length(i,j)*d*...
                        (Elite(i,j)-Position(i,j)*(2*it/Max_iter)); % eq.(21)
                end
                newpos4 = max(min(pos_n4, ub), lb);
                fitt = fobj(newpos4);
                if fitt < Fitness(i)
                    Fitness(i) = fitt;
                    Position(i,:) = newpos4;
                end
            end
        end
       %% Record convergence curve
       [~, Ind] = sort(Fitness);     
        if Fitness(Ind(1)) < Best_Score
            Best_Score = Fitness(Ind(1));
            Best_Pos = Position(Ind(1),:);
        end
        CG_curve(it) = Best_Score;
    end
end
% Finite difference method for approximating generalised Jacobi matrix
function J = Get_jacobian(fobj, x, epsilon)
% f: handle to target function, accepts vector x
% x: current point (vector)
% epsilon: small perturbation value
    n = length(x);
    fx = fobj(x);
    m = length(fx);
    J = zeros(m, n); 
    % Calculate the finite difference in each direction according to eqs. (9) and (10).
    for i = 1:n
        x_perturbed = x;
        x_perturbed(i) = x_perturbed(i) + epsilon;
        f_perturbed = fobj(x_perturbed);
        J(:, i) = (f_perturbed - fx) / epsilon;
    end
end
function [z] = levy(n,m,beta)
    % beta is set to 1.5 in this paper
    num = gamma(1+beta)*sin(pi*beta/2);
    den = gamma((1+beta)/2)*beta*2^((beta-1)/2);
    sigma_u = (num/den)^(1/beta); % eq.(20)
    u = random('Normal',0,sigma_u,n,m);
    v = random('Normal',0,1,n,m);
    z =u./(abs(v).^(1/beta));
end
% Initialization function
function X = initialization(N, Dim, UB, LB)
    B_no = size(UB, 2); % number of boundaries
    
    if B_no == 1
        X = rand(N, Dim) .* (UB - LB) + LB;
    end
    % If each variable has a different lb and ub
    if B_no > 1
        X = zeros(N, Dim);
        for i = 1:Dim
            Ub_i = UB(i);
            Lb_i = LB(i);
            X(:, i) = rand(N, 1) .* (Ub_i - Lb_i) + Lb_i;
        end
    end
end
