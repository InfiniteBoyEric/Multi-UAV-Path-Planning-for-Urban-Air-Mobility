%_________________________________________________________________________%
%  Tetragonula carbonaria Optimization Algorithm (TGCOA)  (source code)   %
%                                                                         %
%  Developed in MATLAB R2024b                                             %
%                                                                         %
%  Authors/Programmers : M.T.A. Mildret Guadalupe Martínez Gámez          %
%                        Dr. Hernán Peraza Vázquez                        %
%                                                                         %
%         e-Mails: hperaza@ipn.mx   or   hperaza@ieee.org                 %
%         Telegram: @CodebugMx                                            %
%                                                                         %
% Paper: A Novel Swarm Optimization Algorithm Based on Hive               %
%        Construction by Tetragonula Carbonaria Builder Bees              %
% Mathematics (2025)                                                      %
% Doi:                                                                    %
% ________________________________________________________________________%

function [fMin, bestX, convergenceCurve]=TGCOA(nPop, MaxIt, lb, ub, dim, fobj)
    
    SearchAgents_no = nPop;
    Max_iter = MaxIt;

    Positions=initialization(SearchAgents_no,dim,ub,lb);
    Fitness = zeros(1, size(Positions,1));
    for i=1:size(Positions,1)
        Fitness(i)=fobj(Positions(i,:)); % get fitness     
    end
    [vMin, minIdx]= min(Fitness);  % the min fitness value vMin and the position minIdx
    theBestVct= Positions(minIdx,:);  % the best vector 
    
    convergenceCurve=zeros(1,Max_iter);
    convergenceCurve(1)= vMin;
    lT= 10;  % lower temperature allowed
    uT= 40;  % upper temperature allowed
    k= 0.03; % temperature generated for heating  and -k for cooling
    alfa= 50; 
    a=0;  %
    b= 10;  %
    
    v= zeros(size(Positions));
    for t=2:Max_iter
      T= uT*exp(-k*t); % Eq. 10
      for r=1:size(Positions,1)
        nb= round(a+(b-a)*rand(1)); % Eq. 1
        if nb==0
            nb=1;
        end
        if nb > SearchAgents_no
            nb= SearchAgents_no;
        end
        sumatory= Tg( SearchAgents_no, nb, Positions, r ); % Eq. 2
        v(r,:)= Positions(r,:) + alfa*rand(1)*(sumatory); % Eq. 3
        
        if T > 30 % Eq. 9
            v(r,:)= v(r,:) + T*rand(1)*(theBestVct - Positions(r,:)); % Eq. 4
        else
            if rand(1) < 0.5
                v(r,:)= theBestVct + T*randn(1)*cos(2*pi*rand(1)); % Eq. 5
            else
                v(r,:)= theBestVct + T*randn(1)*sin(2*pi*rand(1)); % Eq. 6
            end
        end
        
        % Boundary check
        Flag4Upperbound=v(r,:)>ub;
        Flag4Lowerbound=v(r,:)<lb;
        v(r,:)=(v(r,:).*(~(Flag4Upperbound+Flag4Lowerbound)))+ub.*Flag4Upperbound+lb.*Flag4Lowerbound;

        Fnew= fobj(v(r,:));
        if Fnew <= Fitness(r)
            Positions(r,:)= v(r,:);
            Fitness(r)= Fnew;
        end
        if Fnew <= vMin
            theBestVct= v(r,:);
            vMin= Fnew;
        end    
      end
      convergenceCurve(t)= vMin;
    end
    
    fMin = vMin;
    bestX = theBestVct;
end