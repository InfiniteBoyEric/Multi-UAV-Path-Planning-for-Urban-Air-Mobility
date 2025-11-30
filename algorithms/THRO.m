%% %%    Tianji's horse racing optimization (THRO) for 23 functions    %% %%
function [BestFit,BestX,His_BestFit]=THRO(nPop0,MaxIt,Low,Up,Dim,FOBJ)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FunIndex: Index of function                                              %
% nPop0: Sum of horse population of Tianji and King                        %
% nPop: Number of horses each in Tianji's population and King's population %
% MaxIt: Maximum number of iterations                                      %
% Tianji_PopPos: Position of Tianji's horse population                     %
% King_PopPos: Position of King's horse population.                        %  
% Tianji_PopFit: Fitness of Tianji's horse population.                     %
% King_PopFit: Fitness of King's horse population.                         %
% Tianji_SlowestPos: Position of Tian's current slowest horse              %
% King_SlowestPos: Position of King's current slowest horse                %
% Tianji_FastestPos: Position of Tian's current fastest horse              %
% King_FastestPos: Position of King's current fastest horse                %
% Tianji_SlowestId: Tianji's current horse Id                              %
% King_SlowestPos: King's current slowest horse Id                         %
% Tianji_FastestId: Tianji's current fastest horse Id                      %
% King_FastestPos: King's current horse Id                                 %
% Dim: dimensionality of prloblem                                          %
% BestX: Best solution found so far                                        %
% BestFit: Best fitness corresponding to BestX                             %
% His_BestFit: History best fitness over iterations                        %
% LT: Trainging factor of Tianji's horse population                        %
% MT: Trainging factor of Tianji's horse population                        %
% LK: Trainging factor of King's horse population                          %
% MK: Trainging factor of King's horse population                          %
% Low: Low bound of search space                                           %
% Up: Up bound of search space                                             %
% R: Running factor                                                        %
% p: Weight                                                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nPop=nPop0/2;
Low = Low.*ones(1,Dim);
Up = Up.*ones(1,Dim);


Tianji_PopPos=zeros(nPop,Dim);
Tianji_PopFit=zeros(nPop,1);
King_PopPos=zeros(nPop,Dim);
King_PopFit=zeros(nPop,1);

for i=1:nPop
    Tianji_PopPos(i,:)=Low+rand(1,Dim).*(Up-Low);
    Tianji_PopFit(i)=FOBJ(Tianji_PopPos(i,:));
    King_PopPos(i,:)=Low+rand(1,Dim).*(Up-Low);
    King_PopFit(i)=FOBJ(King_PopPos(i,:));
end

BestFit=inf;
BestX=[];

for i=1:nPop
    if Tianji_PopFit(i)<=BestFit
        BestFit=Tianji_PopFit(i);
        BestX=Tianji_PopPos(i,:);
    end
end

for i=1:nPop
    if King_PopFit(i)<=BestFit
        BestFit=King_PopFit(i);
        BestX=King_PopPos(i,:);
    end
end

His_BestFit=zeros(MaxIt,1);


for It=1:MaxIt
    Rand_nPop0=randperm(nPop0);
    PopPos=[Tianji_PopPos;King_PopPos];
    PopFit=[Tianji_PopFit;King_PopFit];
    Tianji_PopPos=PopPos(Rand_nPop0(1:nPop),:);
    Tianji_PopFit=PopFit(Rand_nPop0(1:nPop));
    King_PopPos=PopPos(Rand_nPop0(nPop+1:nPop0),:);
    King_PopFit=PopFit(Rand_nPop0(nPop+1:nPop0));
    [Tianji_PopFit,ind]=sort(Tianji_PopFit);
    Tianji_PopPos=Tianji_PopPos(ind,:);
    [King_PopFit,ind]=sort(King_PopFit);
    King_PopPos=King_PopPos(ind,:);
    T_B=zeros(nPop,Dim);
    K_B=zeros(nPop,Dim);
    p=1-It/MaxIt;

    for i=1:nPop
        RandDim=randperm(Dim);
        RandNum=ceil(sin(pi/2*rand)*Dim);
        T_B(i,RandDim(1:RandNum))=1;
        RandDim=randperm(Dim);
        RandNum=ceil(sin(pi/2*rand)*Dim);
        K_B(i,RandDim(1:RandNum))=1;
    end

    Tianji_SlowestId=nPop;
    Tianji_FastestId=1;
    King_SlowestId=nPop;
    King_FastestId=1;

    for i=1:nPop
        Tianji_Alpha=1+round(0.5*(0.5+rand))*randn;
        T_Beta=round(0.5*(0.1+rand))*randn;
        King_Alpha=1+round(0.5*(0.5+rand))*randn;
        K_Beta=round(0.5*(0.1+rand))*randn;
        Tianji_R=levy(1)*T_B(i,:); % Eq. (7)
        King_R=levy(1)*K_B(i,:); % Eq. (7)
      
        %% Scenario 1: If Tianji's current slowest horse is faster than King's current slowest horse
        if Tianji_PopFit(Tianji_SlowestId)<King_PopFit(King_SlowestId)
            Tianji_SlowestPos=Tianji_PopPos(Tianji_SlowestId,:);
            King_SlowestPos=King_PopPos(King_SlowestId,:);
            Tianji_newPopPos=((p*Tianji_SlowestPos+(1-p)*Tianji_PopPos(1,:))+ Tianji_R.* ...
                (Tianji_PopPos(1,:)-Tianji_SlowestPos+p*(mean(Tianji_PopPos)-mean(King_PopPos))))*Tianji_Alpha+T_Beta; % Eq. (3)           
            Tianji_newPopPos=SpaceBound(Tianji_newPopPos,Up,Low);
            Tianji_newPopFit=FOBJ(Tianji_newPopPos);
            if Tianji_newPopFit<Tianji_PopFit(Tianji_SlowestId)
                Tianji_PopFit(Tianji_SlowestId)=Tianji_newPopFit;
                Tianji_PopPos(Tianji_SlowestId,:)=Tianji_newPopPos;
            end
            King_newPopPos=((p.*King_SlowestPos+(1-p).*Tianji_SlowestPos)+ King_R.* ...
                (Tianji_SlowestPos-King_SlowestPos+p*(mean(Tianji_PopPos)-mean(King_PopPos))))*King_Alpha+K_Beta; % Eq. (14)            
            King_newPopPos=SpaceBound(King_newPopPos,Up,Low);
            King_newPopFit=FOBJ(King_newPopPos);
            if King_newPopFit<King_PopFit(King_SlowestId)
                King_PopFit(King_SlowestId)=King_newPopFit;
                King_PopPos(King_SlowestId,:)=King_newPopPos;
            end
            Tianji_SlowestId=Tianji_SlowestId-1;   %Eq. (3)
            King_SlowestId=King_SlowestId-1;   %Eq. (14)
        else
            %% Scenario 2: If Tianji's current slowest horse is slower than King's current slowest horse
            if Tianji_PopFit(Tianji_SlowestId)>King_PopFit(King_SlowestId)
                Tianji_SlowestPos=Tianji_PopPos(Tianji_SlowestId,:);
                King_FastestPos=King_PopPos(King_FastestId,:);
                Tr1=randi(nPop);
                Tianji_newPopPos=((p*Tianji_SlowestPos+(1-p)*Tianji_PopPos(Tr1,:))+ Tianji_R.* ...
                    (Tianji_PopPos(Tr1,:)-Tianji_SlowestPos+p*(mean(Tianji_PopPos)-mean(King_PopPos))))*Tianji_Alpha+T_Beta;% Eq. (15)                
                Tianji_newPopPos=SpaceBound(Tianji_newPopPos,Up,Low);
                Tianji_newPopFit=FOBJ(Tianji_newPopPos);
                if Tianji_newPopFit<Tianji_PopFit(Tianji_SlowestId)
                    Tianji_PopFit(Tianji_SlowestId)=Tianji_newPopFit;
                    Tianji_PopPos(Tianji_SlowestId,:)=Tianji_newPopPos;
                end
                King_newPopPos=((p.*King_FastestPos+(1-p).*King_PopPos(1,:))+ King_R.* ...
                    (King_PopPos(1,:)-King_FastestPos+p*(mean(Tianji_PopPos)-mean(King_PopPos))))*King_Alpha+K_Beta; %Eq. (16)
                King_newPopPos=SpaceBound(King_newPopPos,Up,Low);
                King_newPopFit=FOBJ(King_newPopPos);
                if King_newPopFit<King_PopFit(King_FastestId)
                    King_PopFit(King_FastestId)=King_newPopFit;
                    King_PopPos(King_FastestId,:)=King_newPopPos;
                end
                Tianji_SlowestId=Tianji_SlowestId-1;
                King_FastestId=King_FastestId+1;

            else  
                %% Scenario 3: If Tianji's current slowest horse runs as fast as King's current slowest horse, 
                %% and if Tianji's current fastest horse is faster than King's current fastest horse.
                if Tianji_PopFit(Tianji_FastestId)<King_PopFit(King_FastestId)
                    Tianji_FastestPos=Tianji_PopPos(Tianji_FastestId,:);
                    King_FastestPos=King_PopPos(King_FastestId,:);
                    Tianji_newPopPos=((p.*Tianji_FastestPos+(1-p).*Tianji_PopPos(1,:))+ Tianji_R.* ...
                        (Tianji_PopPos(1,:)-Tianji_FastestPos+p*(mean(Tianji_PopPos)-mean(King_PopPos))))*Tianji_Alpha+T_Beta; %Eq. (17)
                    Tianji_newPopPos=SpaceBound(Tianji_newPopPos,Up,Low);
                    Tianji_newPopFit=FOBJ(Tianji_newPopPos);
                    if Tianji_newPopFit<Tianji_PopFit(Tianji_FastestId)
                        Tianji_PopFit(Tianji_FastestId)=Tianji_newPopFit;
                        Tianji_PopPos(Tianji_FastestId,:)=Tianji_newPopPos;
                    end
                    King_newPopPos=((p.*King_FastestPos+(1-p).*Tianji_FastestPos)+King_R.* ...
                        ( Tianji_FastestPos-King_FastestPos+p*(mean(Tianji_PopPos)-mean(King_PopPos))))*King_Alpha+K_Beta; %Eq. (18)
                    King_newPopPos=SpaceBound(King_newPopPos,Up,Low);
                    King_newPopFit=FOBJ(King_newPopPos);
                    if King_newPopFit<King_PopFit(King_FastestId)
                        King_PopFit(King_FastestId)=King_newPopFit;
                        King_PopPos(King_FastestId,:)=King_newPopPos;
                    end
                    Tianji_FastestId=Tianji_FastestId+1;
                    King_FastestId=King_FastestId+1;
                else                  
                    %% Scenario 4: If Tianji's current slowest horse runs as fast as King's current slowest horse, 
                    %% and when Tianji's current fastest horse is slower than King's current fastest horse.
                    if   Tianji_PopFit(Tianji_FastestId)> King_PopFit(King_FastestId)
                        King_FastestPos=King_PopPos(King_FastestId,:);
                        Tianji_SlowestPos=Tianji_PopPos(Tianji_SlowestId,:);
                        Tr2=randi(nPop);
                        Tianji_newPopPos=((p.*Tianji_SlowestPos+(1-p).*Tianji_PopPos(Tr2,:))+ Tianji_R.* ...
                            (Tianji_PopPos(Tr2,:)-Tianji_SlowestPos+p*(mean(Tianji_PopPos)-mean(King_PopPos))))*Tianji_Alpha+T_Beta; %Eq. (19)
                        Tianji_newPopPos=SpaceBound(Tianji_newPopPos,Up,Low);
                        Tianji_newPopFit=FOBJ(Tianji_newPopPos);
                        if Tianji_newPopFit<Tianji_PopFit(Tianji_SlowestId)
                            Tianji_PopFit(Tianji_SlowestId)=Tianji_newPopFit;
                            Tianji_PopPos(Tianji_SlowestId,:)=Tianji_newPopPos;
                        end
                        King_newPopPos=((p.*King_FastestPos+(1-p).*King_PopPos(1,:))+ King_R.* ...
                            (King_PopPos(1,:)-King_FastestPos+p*(mean(Tianji_PopPos)-mean(King_PopPos))))*King_Alpha+K_Beta;
                        % Eq.  (20)
                        King_newPopPos=SpaceBound(King_newPopPos,Up,Low);
                        King_newPopFit=FOBJ(King_newPopPos);
                        if King_newPopFit<King_PopFit(King_FastestId)
                            King_PopFit(King_FastestId)=King_newPopFit;
                            King_PopPos(King_FastestId,:)=King_newPopPos;
                        end
                        Tianji_SlowestId=Tianji_SlowestId-1;%no
                        King_FastestId=King_FastestId+1;%no
                    else                       
                        %% Scenario 5: When Tianji's current slowest horse runs as fast as King,s current slowest horse 
                        %% and if Tianji's current fastest horse runs as fast as King's current fastest horse.
                        if  Tianji_PopFit(Tianji_FastestId)==King_PopFit(King_FastestId)
                            Tianji_SlowestPos=Tianji_PopPos(Tianji_SlowestId,:);
                            King_FastestPos=King_PopPos(King_FastestId,:);
                            Tr3=randi(nPop);
                            Tianji_newPopPos=((p.*Tianji_SlowestPos+(1-p).*Tianji_PopPos(Tr3,:))+ Tianji_R.* ...
                                (Tianji_PopPos(Tr3,:)-Tianji_SlowestPos+p*(mean(Tianji_PopPos)-mean(King_PopPos))) )*Tianji_Alpha+T_Beta; %Eq. (21)
                            Tianji_newPopPos=SpaceBound(Tianji_newPopPos,Up,Low);
                            Tianji_newPopFit=FOBJ(Tianji_newPopPos);
                            if Tianji_newPopFit<Tianji_PopFit(Tianji_SlowestId)
                                Tianji_PopFit(Tianji_SlowestId)=Tianji_newPopFit;
                                Tianji_PopPos(Tianji_SlowestId,:)=Tianji_newPopPos;
                            end                            
                            King_newPopPos=((p.*King_FastestPos+(1-p).*King_PopPos(1,:))+ King_R.* ...
                                (King_PopPos(1,:)-King_FastestPos+p*(mean(Tianji_PopPos)-mean(King_PopPos))))*King_Alpha+K_Beta; %Eq.  (22)
                            King_newPopPos=SpaceBound(King_newPopPos,Up,Low);
                            King_newPopFit=FOBJ(King_newPopPos);
                            if King_newPopFit<King_PopFit(King_FastestId)
                                King_PopFit(King_FastestId)=King_newPopFit;
                                King_PopPos(King_FastestId,:)=King_newPopPos;
                            end
                            Tianji_SlowestId=Tianji_SlowestId-1;
                            King_FastestId=King_FastestId+1;
                        end
                    end
                end
            end
        end
    end

    for i=1:nPop
        if King_PopFit(i)<BestFit
            BestFit=King_PopFit(i);
            BestX=King_PopPos(i,:);
        end
    end

    for i=1:nPop
        if Tianji_PopFit(i)<BestFit
            BestFit=Tianji_PopFit(i);
            BestX=Tianji_PopPos(i,:);
        end
    end
    
    [~,FKingId]=min(King_PopFit); [~,FTianId]=min(Tianji_PopFit);
    
    for i=1:nPop     
           for j=1:Dim
            Rand_nPop=randperm(nPop);   
            
            if rand>0.5 
                Tr4=Rand_nPop(1);
                Tr5=Rand_nPop(2);
                LT=0.2*levy(1);
                Tianji_newPopPos(1,j)=Tianji_PopPos(i,j)+LT*(Tianji_PopPos(Tr4,j)-Tianji_PopPos(Tr5,j)); % Eq. (24)
            else
                MT=1/2*(1+0.001*(1-It/MaxIt)^2*sin(pi*rand)); % Eq. (23)
                Tianji_newPopPos(1,j)=(Tianji_PopPos(FTianId,j)) +MT*(Tianji_PopPos(FTianId,j)-Tianji_PopPos(i,j)); 
            end
           end
       
        for j=1:Dim
            Rand_nPop=randperm(nPop); 
            
            if rand>0.5 
                Kr1=Rand_nPop(1); 
                Kr2=Rand_nPop(2);
                LK=0.2*levy(1);
                King_newPopPos(1,j)=King_PopPos(i,j)+LK*(King_PopPos(Kr1,j)-King_PopPos(Kr2,j)); % Eq. (26)
            else 
                MK=1/2*(1+0.001*(1-It/MaxIt)^2*sin(pi*rand)); % Eq. (25)
                King_newPopPos(1,j)=(King_PopPos(FKingId,j)) +MK*(King_PopPos(FKingId,j)-King_PopPos(i,j));
            end
        end

        Tianji_newPopPos=SpaceBound(Tianji_newPopPos,Up,Low);
        King_newPopPos=SpaceBound(King_newPopPos,Up,Low);
        Tianji_newPopFit=FOBJ(Tianji_newPopPos);
        King_newPopFit=FOBJ(King_newPopPos);

        if Tianji_newPopFit< Tianji_PopFit(i)
            Tianji_PopFit(i)=Tianji_newPopFit;
            Tianji_PopPos(i,:)=Tianji_newPopPos;
        end

        if King_newPopFit< King_PopFit(i)
            King_PopFit(i)=King_newPopFit;
            King_PopPos(i,:)=King_newPopPos;
        end
    end
 
    for i=1:nPop
        if King_PopFit(i)<BestFit
            BestFit=King_PopFit(i);
            BestX=King_PopPos(i,:);
        end
    end
    for i=1:nPop
        if Tianji_PopFit(i)<BestFit
            BestFit=Tianji_PopFit(i);
            BestX=Tianji_PopPos(i,:);
        end
    end
    His_BestFit(It)=BestFit;
end
