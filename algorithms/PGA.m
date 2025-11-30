%%
function [Fbest,Xbest,CNVG]=PGA(N,T,lb,ub,dim,fobj)
% initialise the best cell
Fbest=inf;
vec_flag=[1,-1];
Nl=round((0.2*rand+0.4)*N);%eq.(2&3)
%Initialise the plant cells
lb = ones(1,dim).*lb;
ub = ones(1,dim).*ub;
X=initialization(N,dim,ub,lb);
for i=1:N
    fitness(i)=feval(fobj,X(i,:));
end
[GYbest, gbest] = min(fitness);
Xbest= X(gbest,:);
Ns=N-Nl;
Xs=X(1:Ns,:);
Xl=X(Ns+1:N,:);

% Initialisng fitness variables
fitness_s=fitness(1:Ns);
fitness_l=fitness(Ns+1:N);

[fitnessBest_s, gbest1] = min(fitness_s);
Xbest_s = Xs(gbest1,:);

[fitnessBest_l, gbest2] = min(fitness_l);
Xbest_l = Xl(gbest2,:);

Curvaure=1;
% ConcS=Curvaure*(mean(Xs)./Xbest_s).*(mean(fitness_s)/fitnessBest_s);
% ConcL=Curvaure*(mean(Xl)./Xbest_l).*(mean(fitness_l)/fitnessBest_l);

t=0; % Loop counter

while t<T
    alpha = exp(-1*t/T); % eq(5)
    
    for i=1:size(Xs,1)
        FU=Xs(i,:)>ub;
        FL=Xs(i,:)<lb;
        Xs(i,:)=(Xs(i,:).*(~(FU+FL)))+ub.*FU+lb.*FL;
        fitness_s(i) = feval(fobj,Xs(i,:));
        if fitness_s(i)<fitnessBest_s
            fitnessBest_s=fitness_s(i);
            Xbest_s=Xs(i,:);
        end
    end
    for i=1:size(Xl,1)
        FU=Xl(i,:)>ub;
        FL=Xl(i,:)<lb;
        Xl(i,:)=(Xl(i,:).*(~(FU+FL)))+ub.*FU+lb.*FL;
        fitness_l(i) = feval(fobj,Xl(i,:));
        if fitness_l(i)<fitnessBest_l
            fitnessBest_l=fitness_l(i);
            Xbest_l=Xl(i,:);
        end
    end
    flag_index = floor(2*rand()+1);
    beta=vec_flag(flag_index);
    Curvaure=beta*(alpha-mean(fitness_s)/Fbest); %Eq(17)
    for i=1:size(Xl,1)
        r2=2*rand(1,dim)-1;
        r3=2*rand(1,dim)-1;
        r4=2*rand(1,dim)-1;
        dd=randi([1,N]);
        flag_index = floor(2*rand()+1);
        beta=vec_flag(flag_index);
        Xlnew1(1,:)=X(dd,:)+beta*alpha.*r2.*abs(X(dd,:)-Xl(i,:))+beta*alpha.*r3.*abs(Xbest_l-Xl(i,:));%Eq(9)
        Xlnew1(2,:)=Xl(i,:)+alpha.*r4.*abs(Xbest_l-Xl(i,:));%Eq(13)
        for j=1:2
            Tp=Xlnew1(j,:)>ub;Tm=Xlnew1(j,:)<lb;Xlnew1(j,:)=(Xlnew1(j,:).*(~(Tp+Tm)))+ub.*Tp+lb.*Tm;
            fitness_lNew(j)=feval(fobj, Xlnew1(j,:));
        end
        Xl=[Xl; Xlnew1];
        fitness_l=[fitness_l fitness_lNew];
    end
    
    for i=1:size(Xs,1)
        r=2*rand(1,dim)-1;
        flag_index = floor(2*rand()+1);
        beta=vec_flag(flag_index);
        Xsnew1(1,:)=Xs(i,:)+beta*alpha.*r.*abs(Xs(i,:)-X(randi([1,N]),:));% Eq(15)
        Xsnew1(2,:)=Xl(randi(Ns),:)+beta*alpha.*r.*(Xbest_l-Xs(i,:));%% Eq(16)
        for j=1:2
            Tp=Xsnew1(j,:)>ub;Tm=Xsnew1(j,:)<lb;Xsnew1(j,:)=(Xsnew1(j,:).*(~(Tp+Tm)))+ub.*Tp+lb.*Tm;
            fitness_sNew(j)=feval(fobj, Xsnew1(j,:));
        end
        
        Xs=[Xs; Xsnew1];
        fitness_s=[fitness_s fitness_sNew];
    end
    
    [fitness_s, SortOrder]=sort(fitness_s);
    Xs=Xs(SortOrder,:);
    
    [fitnessBest_s,Sbest]=min(fitness_s);
    Xbest_s=Xs(Sbest,:);
    
    Xs=Xs(1:Ns,:);
    fitness_s=fitness_s(1:Ns);
    
    [fitness_l, SortOrder]=sort(fitness_l);
    Xl=Xl(SortOrder,:);
    [fitnessBest_l,lbest]=min(fitness_l);
    
    Xbest_l=Xl(lbest,:);
    Xl=Xl(1:Nl,:);
    fitness_l=fitness_l(1:Nl);
    
    if fitnessBest_l<Fbest
        Fbest=fitnessBest_l;
        Xbest=Xbest_l;
    elseif fitnessBest_s<Fbest
        Fbest=fitnessBest_s;
        Xbest=Xbest_s;
    end
    
    X=[Xs;Xl];
    fitall=[fitness_s fitness_l];
    
    for i=1:N
        r=2*rand(1,dim)-1;
        flag_index = floor(2*rand()+1);
        beta=vec_flag(flag_index);
        FOC=r.*(Curvaure.*X(i,:)-Xbest);  %eq(18)
        Cell_vicinity=beta*alpha.*r.*(X(i,:)+X(i+1,:))/2;   %eq(19)
        Xnew(1,:)=X(i,:)+FOC+Cell_vicinity;  %eq(20)
        
        for j=1:1
            Tp=Xnew(j,:)>ub;Tm=Xnew(j,:)<lb;Xnew(j,:)=(Xnew(j,:).*(~(Tp+Tm)))+ub.*Tp+lb.*Tm;
            fitnessn(j)=feval(fobj, Xnew(j,:));
        end
        X=[X; Xnew];
        fitall=[fitall fitnessn];
    end
    
    [fitall, SortOrder]=sort(fitall);
    X=X(SortOrder,:);
    
    [Fbest,best]=min(fitall);
    Xbest=X(best,:);
    
    X=X(1:N,:);
    
    Nl=N-Ns;
    Xs=X(1:Ns,:);
    Xl=X(Ns+1:N,:);
    fitness_s=fitness(1:Ns);
    fitness_l=fitness(Ns+1:N);
    [fitnessBest_s, gbest1] = min(fitness_s);
    Xbest_s = Xs(gbest1,:);
    [fitnessBest_l, gbest2] = min(fitness_l);
    Xbest_l = Xl(gbest2,:);
    
    %     Update iteration counter
    t=t+1;
    CNVG(t)=Fbest;
    
end

end
