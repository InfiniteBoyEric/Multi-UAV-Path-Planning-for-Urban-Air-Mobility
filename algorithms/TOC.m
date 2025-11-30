function [TornadoCost,Tornadoposition,ccurve]=TOC(n,max_it,lb,ub,dim,fobj)
    %% Convergence curve
    ccurve=zeros(1,max_it);
    nto=4;
    nt =3;
        
    %% Generate initial population for the Tornado Optimizer with Coriolis force (TOC) 
    
    % Create initial population for Tornado, thunderstorms, and windstorms, and initialize the positions of population
     ub = ub.*ones(1,dim);
      lb = lb.*ones(1,dim);
    y=initialization(n,dim,ub,lb);
    
    % Evaluate the fitness of initial population for Tornado, thunderstorms, and windstorms 
    
    fit = zeros(size(y,1),1);
    
    for i=1:size(y,1)
         fit(i,1)=fobj(y(i,:));
    end
    
    [~,index]=sort(fit) ;
    
    %% Forming Windstorms, Thunderstorms, and Tornado of the Tornado Optimizer with Coriolis force (TOC) 
    
    % nto: Number of thunderstorms and tornadoes
    % Nt : Number of thunderstorms
    % To:  Number of tornadoes
    % nw: number of windstorms
    
    To= nto - nt;% Tornadoes
    
    nw=n-nto; % Windstorms
    
    
    %================ Forming and evaluating the population of the Tornadoes ================
    
    Tornadoposition=y(index(1:To),:) ; % 
    TornadoCost=fit(index(1:To)); 
            
     %================ Forming and evaluating the population of the Thunderstorms ================
    Thunderstormsposition(1:nto-1,:)=y(index(2:nto),:);
    ThunderstormsCost(1:nto-1)=fit(index(2:nto));
    
    bThunderstormsCost = ThunderstormsCost;
    gThunderstormsCost=zeros(1,nto-1);
         
    [~,ind]=min(ThunderstormsCost);
    
    bThunderstormsposition = Thunderstormsposition; % Initial best Thunderstorms position
    gThunderstormsCost = Thunderstormsposition(ind,:); % Initial global Thunderstorms position
    
    %================ Forming and evaluating the population of the Windstorms ================
      
    Windstormsposition(1:nw,:)=y(index(nto+1:nto+nw),:) ; 
    WindstormsCost(1:nw)=fit(index(nto+1:nto+nw)) ;  
    
    gWindstormsposition=zeros(1,nw);
    
    bWindstormsCost = WindstormsCost;
    [~,ind]=min(WindstormsCost);
    
    bWindstormsposition = Windstormsposition; % % Initial best windstorms position (Update the best positions of windstorms)
    gWindstormsposition = Windstormsposition(ind,:); % Initial global windstorms position
     
    %% Velcity term of TOC 
    
    vel_storm = 0.1*Windstormsposition; % Velocity of windstorms
    
    %%  Designate windstorms to thunderstorms and Tornadoes 
    nwindstorms=1:nw;
     
    nwindstorms=nwindstorms(sort(randperm(nw,nto)));
     
    % Combining windstorms to tornado
    
    nWT=diff(nwindstorms)';
    nWT(end+1)= nw-sum(nWT);
    % nWT1=sort(nWT1,'descend');
    
    nWT1 = nWT(1);
    
    % Combining windstorms to thunderstorms
    
    nWH=nWT(2:end);
    
    %% Parameter setting s for TOC
    b_r=100000;  
    fdelta=[-1,1];
    
    %% Key functions of Tornado Optimizer with Coriolis force (TOC) 
    
    chi=4.10;
    eta=2/abs(2-chi-sqrt(chi^2-4*chi)) ;
    %% ================  Main Loop for TOC ================ 
    % disp('================  Tornado Optimizer with Coriolis force (TOC) ================ ');
    
    t=1;
     
    while t<=max_it
        
    %% Key adaptive functions (Adaptive parameters) of TOC
         
       nu  =(0.1*exp(-0.1*(t/max_it)^0.1))^16;
       mu = 0.5 + rand/2;
       ay=(max_it-(t^2/max_it))/max_it ;
       
       Rl = 2/(1+exp((-t+max_it/2)/2)) ;  
       Rr = -2/(1+exp((-t+max_it/2)/2)) ;  
            
     %%  Evolution of windstorms to Tornadoes
         % Update velocity     
    
    for i=1:nw
        for j = 1:dim
             if rand > 0.5
               
              delta1=fdelta(ceil(2*rand()));
              zeta=ceil((To).*rand(1,To))'; %  
    
              wmin=1; wmax=4.0; rr=wmin+rand()*(wmax-wmin);
              wr= (((2*rand()) - (1*rand()+rand()))/rr); 
     
              c=b_r*delta1*wr;
               
              omega = 0.7292115E-04; 
    %            w_r = sin(-1 + 2.*rand(1,1));
              f= 2*omega*sin(-1 + 2.*rand(1,1));
             
              phi(i,j) = Tornadoposition(zeta,j) - Windstormsposition(i,j);
    
              if sign(Rl)>=0 
                  if sign(phi(i,j))>=0 
                    phi(i,j) = -phi(i,j);
                  end
              end
              
               CFl =(((f^2*Rl^2)/4) -Rl* 1 *phi(i,j));
                
                if sign(CFl)< 0 
                     CFl= -CFl;
                end
                
                vel_storm(i,j)= eta*  (mu*vel_storm(i,j) - c* (f*Rl)/2 +(sqrt(CFl)));
              
             else
    
              delta1=fdelta(ceil(2*rand()));
              zeta=ceil((To).*rand(1,To))'; %  
    
             rmin=1; rmax=4.0; rr=rmin+rand()*(rmax-rmin);
              wr= (((2*rand()) - (1*rand()+rand()))/rr); 
                c=b_r*delta1*wr; 
     
                phi(i,j) = Tornadoposition (zeta,j)-Windstormsposition(i,j);
     
             if sign(Rr)<=0 
                  if sign(phi(i,j))<=0 
                    phi(i,j) = -phi(i,j);
                   end
             end
             
            omega =0.7292115E-04;% s�1
    %          w_r = sin(-1 + 2.*rand(1,1));
             f= 2*omega*sin(-1 + 2.*rand(1,1));
             
             CFr =(((f^2*Rr^2)/4) -Rr* 1 *phi(i,j));
                
                if sign(CFr)<0 
                     CFr= -CFr;
                end 
                
               vel_storm(i,j)= eta  *(mu* vel_storm(i,j) - c* (f*Rr)/2 +(sqrt(CFr)))  ;        
             end
         end
            
    end 
                   
    %% %% Exploration - Evolution of windstorms to Tornadoes
      
    for i=1:nWT1
              rand_index = floor((nWT1).*rand(1,nWT1))+1;
              rand_w = Windstormsposition(rand_index, :);
       
                alpha=abs(2*ay*rand-1*rand)   ;
         
             Windstormsposition(i,:)=Windstormsposition(i,:)+2*alpha*(Tornadoposition  - rand_w(i,:)) + vel_storm(i,:);
    %  
                        
                 ub_=Windstormsposition(i,:)>ub; lb_=Windstormsposition(i,:)<lb;
                 
                 Windstormsposition(i,:)=(Windstormsposition(i,:).*(~(ub_+lb_)))+ub.*ub_+lb.*lb_;
    
                 
                WindstormsCost (i)=fobj(Windstormsposition(i,:));
         
           %%% Finding out the best positions
    
                   if WindstormsCost(i)<bWindstormsCost(i)
                        bWindstormsposition(i,:)=Windstormsposition(i,:) ; % Best solutions
                        bWindstormsCost(i)=WindstormsCost(i);   % Best cost
                  end
          
    end
    
          [minTornadoCost,in]=min(bWindstormsCost); % finding out the best Pos
    
       if (minTornadoCost<TornadoCost)
             TornadoCost=minTornadoCost;
             Tornadoposition = bWindstormsposition(in,:); % Update the global best positions
       end 
    
       
    %% ================ Exploitation -  Evolution of windstorms to thunderstorms ================ 
    %% ================  Combining windstorms together to form thunderstorms ================  
        for i=1:nt
            for j=1:nWH(i)
                
             rand_index = floor((nt).*rand(nt))+1;
             rand_w = Windstormsposition(rand_index, :);
             
             c1=abs(2*ay*rand-1*ay); 
           
            c2=abs(ay - 2*ay*rand);
             
     Windstormsposition((j+sum(nWT(1:i))),:)=Windstormsposition((j+sum(nWT(1:i))),:)+2*rand*(Thunderstormsposition(i,:)-Windstormsposition((j+sum(nWT(1:i))),:))+...
                    + 2*rand*(Tornadoposition (1,:)-Windstormsposition((j+sum(nWT(1:i))),:));
    
    
                  ub_=Windstormsposition((j+sum(nWT(1:i))),:)>ub; lb_=Windstormsposition((j+sum(nWT(1:i))),:)<lb;
                 
                 Windstormsposition((j+sum(nWT(1:i))),:)=(Windstormsposition((j+sum(nWT(1:i))),:).*(~(ub_+lb_)))+ub.*ub_+lb.*lb_;
                 
                 
                WindstormsCost((j+sum(nWT(1:i))))=fobj(Windstormsposition((j+sum(nWT(1:i))),:));
                
                      
                 if WindstormsCost((j+sum(nWT(1:i))))<ThunderstormsCost(i)              
    
                      bThunderstormsposition(i,:) =Windstormsposition((j+sum(nWT(1:i))),:);
                     
                    Thunderstormsposition(i,:)=Windstormsposition((j+sum(nWT(1:i))),:);
                    ThunderstormsCost(i)=WindstormsCost((j+sum(nWT(1:i))));
                     
                 end 
                
                 
            end
        end   
        
    %
       [minTornadoCost, in]=min(ThunderstormsCost); % finding out the best Pos
    
       if (minTornadoCost<TornadoCost)
             TornadoCost=minTornadoCost;
             Tornadoposition = bThunderstormsposition(in,:); % Update the global best positions
      end 
     
        
    %% ================  Evolution of thunderstorms to tornado ================ 
    
      for i=1:nt        
                      zeta=ceil((To).*rand(1,To)); %  
     
                      alpha=abs(2*ay*rand-1*rand)    ; 
    
                      p = floor((nt).*rand(1,nt))+1;
                      rand_w = Thunderstormsposition(p, :);
             
    Thunderstormsposition(i,:)=Thunderstormsposition(i,:)+2.*alpha*(Thunderstormsposition(i,:) - Tornadoposition(zeta,:))+...
                +2.*alpha*(rand_w(i,:) - Thunderstormsposition(i,:));
        
                
            ub_=Thunderstormsposition(i,:)>ub; lb_=Thunderstormsposition(i,:)<lb;
                 
                 Thunderstormsposition(i,:)=(Thunderstormsposition(i,:).*(~(ub_+lb_)))+ub.*ub_+lb.*lb_;
                 
             ThunderstormsCost(i) =fobj(Thunderstormsposition(i,:));
    
      	    if ThunderstormsCost(i)<bThunderstormsCost(i)         
                bThunderstormsposition(i,:) =Thunderstormsposition(i,:);
                bThunderstormsCost(i) =ThunderstormsCost(i);
     	    end
                    
      end
    
       [minTornadoCost,in]=min(bThunderstormsCost); % finding out the best Pos
    
       if (minTornadoCost<TornadoCost)
             TornadoCost=minTornadoCost;
             Tornadoposition = bThunderstormsposition(in,:); % Update the global best positions
      end 
      
     
    %%   ================  Random formation of windstorms, tornadoes and thunderstorms ================ 
    
       % Check windstorms formation for windstorms and tornadoes
    
     for i=1:nWT1
            if  ((norm(Windstormsposition(i,:)-Tornadoposition)<nu))
                 
                   delta2=fdelta(floor(2*rand()+1));
                  
                Windstormsposition(i,:)=  Windstormsposition(i,:) - (2*ay*(rand*(lb-ub) - lb))*delta2;
    
            end
    end  
      
    % Check windstorms formation for windstorms and thunderstorms
          
    for i=1:nt
             if  ((norm(Windstormsposition(i,:)-Thunderstormsposition(i,:))<nu))
                for j=1:nWH(i)
                     delta2=fdelta(floor(2*rand()+1)) ;
                    Windstormsposition((j+ sum(nWT(1:i))),:)=  Windstormsposition((j+ sum(nWT(1:i))),:) - (2*ay*(rand*(lb-ub) - lb))*delta2;
     
              end
            end
    end 
     
    %% Results and Plot   
        
        ccurve(t)=TornadoCost;
      
     %{
     if t>2
            line([t-1 t], [ccurve(t-1) ccurve(t)],'Color','b'); 
            title({'Convergence characteristic curve'},'interpreter','latex','FontName','Times','fontsize',12);
            xlabel('Iteration');
            ylabel('Best score obtained so far');
            drawnow 
     end 
    %}
        t=t+1;
        
    end
 
end
