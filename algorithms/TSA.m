%% Tunicate Swarm Algorithm - TSA
%%% Designed and Developed by Dr. Gaurav Dhiman (http://dhimangaurav.com/) %%%
function [Score,Position,Convergence]=TSA(Search_Agents,Max_iterations,Lowerbound,Upperbound,dimensions,objective)
Position=zeros(1,dimensions);
Score=inf;
Positions=initialization(Search_Agents,dimensions,Upperbound,Lowerbound);
Convergence=zeros(1,Max_iterations);
t=0;
while t<Max_iterations
    for i=1:size(Positions,1)
        Flag4Upperbound=Positions(i,:)>Upperbound;
        Flag4Lowerbound=Positions(i,:)<Lowerbound;
        Positions(i,:)=(Positions(i,:).*(~(Flag4Upperbound+Flag4Lowerbound)))+Upperbound.*Flag4Upperbound+Lowerbound.*Flag4Lowerbound;

        fitness=objective(Positions(i,:));
        if fitness<Score
            Score=fitness;
            Position=Positions(i,:);
        end
    end

    xmin=1;
    xmax=4;
    xr=xmin+rand()*(xmax-xmin);
    xr=fix(xr);

    for i=1:size(Positions,1)
        for j=1:size(Positions,2)
            A1=((rand()+rand())-(2*rand()))/xr;
            c2=rand();
            if(i==1)
                c3=rand();
                if(c3>=0)
                    d_pos=abs(Position(j)-c2*Positions(i,j));
                    Positions(i,j)=Position(j)+A1*d_pos;
                else
                    d_pos=abs(Position(j)-c2*Positions(i,j));
                    Positions(i,j)=Position(j)-A1*d_pos;
                end
            else

                c3=rand();
                if(c3>=0)
                    d_pos=abs(Position(j)-c2*Positions(i,j));
                    Pos(i,j)=Position(j)+A1*d_pos;
                else
                    Pos(i,j)=Position(j)-A1*d_pos;

                end
                Positions(i,j)=(Pos(i,j)+Positions(i-1,j))/2;
            end
        end
    end
    t=t+1;
    Convergence(t)=Score;
    [t Score];
end
end

function Positions=initialization(SearchAgents_no,dim,ub,lb)

Boundary_no= size(ub,2); % numnber of boundaries

% If the boundaries of all variables are equal and user enter a signle
% number for both ub and lb
if Boundary_no==1
    Positions=rand(SearchAgents_no,dim).*(ub-lb)+lb;
end

% If each variable has a different lb and ub
if Boundary_no>1
    for i=1:dim
        ub_i=ub(i);
        lb_i=lb(i);
        Positions(:,i)=rand(SearchAgents_no,1).*(ub_i-lb_i)+lb_i;
    end
end
end
