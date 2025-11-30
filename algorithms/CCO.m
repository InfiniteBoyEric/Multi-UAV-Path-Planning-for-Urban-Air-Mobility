%%
%_________________________________________________________________________________
% 
%  CCO source code (Developed in MATLAB R2022b)
%  Paper:
%  Tian-Lei Wang, Shao-Wei Gu, Ren-Ju Liu, Le-Qing Chen, Zhu Wang, Zhi-Qiang Zeng, 
%  Cuckoo Catfish Optimizer: A New Meta-Heuristic Optimization Algorithm 
%  Artificial Intelligence Review 
%__________________________________________________________________________________

function [BestF,BestX,HisBestFit,FW_CCO,FA_CCO,SE_CCO]=CCO(nPop,MaxIt,Low,Up,Dim,BenFunctions)

alpha=1.34; beta=0.3;
BestF=inf; BestX=[];
for i=1:nPop
    PopPos(i,:)=rand(1,Dim).*(Up-Low)+Low;
    PopFit(i)=BenFunctions(PopPos(i,:));
    theta=(1-10*i/nPop)*pi;
    r=alpha*exp(beta*theta/3); x(i)=r.*cos(theta); y(i)=r.*sin(theta);
    if PopFit(i)<=BestF
        BestF=PopFit(i);
        BestX=PopPos(i,:);
    end
end
[worst windex]=max(PopFit);  
WorstX=PopPos(windex,:);
Dis=abs(mean(mean((PopPos-BestX)./(WorstX-BestX+eps)))); Lx=abs(randn)*rand;
HisBestFit=zeros(MaxIt,1);
s=0;z=0;t=0;

for It=1:MaxIt
    C=(1-It/MaxIt);
    T=(1-(sin((pi*It)/(2*MaxIt))))^(It/MaxIt);
    if t<15
       die=0.02*T;
    else
       die=0.02; C=0.8;
    end
    n=1;
    for i=1:nPop
        Q=0;
        F=sign(0.5-rand);
        E=n*T+rand;
        R1=rand(1,Dim);R4=rand(1,Dim);r1=rand;r2=rand;
        S=sin(pi*R4*C);
        k=randperm(nPop);
        PopPosrand=PopPos(k,:); PopFitrand=PopFit(k);
        if rand>C
            J(i)=abs(mean((PopPos(i,:)-BestX)./(WorstX-BestX+eps)));
            if rand>C
                Cy= 1./(pi*(1+C^2));
                if J(i)>Dis
                    newPopPos(i,:)=BestX+F*S.*(BestX-PopPos(i,:));
                else
                    if Dis*Lx<J(i)
                        newPopPos(i,:)=BestX*(1+T^5*Cy*E)+F*(S.*(BestX-PopPos(i,:)));
                    else
                        newPopPos(i,:)=BestX*(1+T^5*normrnd(0,C^2))+F*(S.*(BestX-PopPos(i,:)));
                    end
                end
            else
                if rand>C
                    if mod(i,2)==1
                        r3=rand;
                        step=(BestX-E*PopPos(i,:));
                        newPopPos(i,:)=C/It*(r1*BestX-r3*PopPos(i,:))+T^2*lev(1,Dim).*abs(step);
                    else
                        R2=rand(1,Dim);R3=rand(1,Dim);
                        step=PopPos(i,:)-E*BestX;
                        DE=C*F;
                        newPopPos(i,:)=0.5*(BestX+PopPosrand(1,:))+DE*(2*R1.*(step)-R2/2.*(DE*R3-1));
                    end
                else
                    if rand<rand
                        if J(i)<Dis
                            V=2*((rand)*(mean(PopPos)-PopPos(i,:))+(rand)*(BestX-PopPos(i,:)));
                        else
                            V=2*((rand)*(PopPosrand(2,:)-PopPosrand(3,:))+(rand)*(PopPosrand(1,:)-PopPos(i,:)));
                        end
                        if PopFit(i)<= PopFitrand(i)
                            step=PopPos(i,:)-E*PopPosrand(i,:);
                            if mod(i,2)==1
                                newPopPos(i,:)=(PopPos(i,:)+T^2*y(i)*(1-R1).*abs(step))+F*R1.*step/2+V*J(i)/It;
                            else
                                newPopPos(i,:)=(PopPos(i,:)+T^2*x(i)*(1-R1).*abs(step))+F*R1.*step/2+V*J(i)/It;
                            end
                        else
                            step=PopPosrand(i,:)-E*PopPos(i,:);
                            if mod(i,2)==1
                                newPopPos(i,:)=(PopPosrand(i,:)+T^2*y(i)*(1-R1).*abs(step))+F*R1.*step/2+V*J(i)/It;
                            else
                                newPopPos(i,:)=(PopPosrand(i,:)+T^2*x(i)*(1-R1).*abs(step))+F*R1.*step/2+V*J(i)/It;
                            end
                        end
                        s=s+1;
                        if s>10
                            lesp1=r1*PopPos(randperm(nPop,1),:)+(1-r1)*PopPos(randperm(nPop,1),:);
                            newPopPos(i,:)=round(lesp1)+F*r1*R1/(It^4).*newPopPos(i,:);
                            s=0;
                        end
                    else
                        [frank index]=sort(PopFit);
                        A2=randperm(4,1);A1=randperm(4,1);
                        D=[PopPos(index(1:3),:);mean(PopPos)];B=D(A1,:);
                        Rt1=randperm(360,Dim)*pi/360;Rt2=randperm(360,Dim)*pi/360;
                        w=1-((exp(It/MaxIt)-1)/(exp(1)-1))^2;
                        if rand<0.33
                            newPopPos(i,:)=B+2*w*F*(cos(Rt1)).*(sin(Rt2)).*(B-PopPos(i,:));
                        else
                            if rand<0.33
                                newPopPos(i,:)=B+2*w*F*(sin(Rt1)).*(cos(Rt2)).*(B-PopPos(i,:));
                            else
                                newPopPos(i,:)=B+2*w*F*(cos(Rt2)).*(B-PopPos(i,:));
                            end
                        end
                        if A2==4
                            Q=1;
                        end
                        z=z+1;
                        if z>5
                            newPopPos(i,:)=BestX.*(1-(1-1./(PopPos(randperm(nPop,1),:)+eps)).*R1);
                            z=0;
                        end
                    end
                end
            end
        else
            if rand>C
                if rand>C
                    newPopPos(i,:)=PopPosrand(3,:)+abs(randn)*(BestX-PopPos(i,:)+PopPosrand(1,:)-PopPosrand(2,:));
                else
                    Z2=rand(1,Dim)<rand;
                    newPopPos(i,:)=Z2.*(PopPosrand(3,:)+abs(randn)*(PopPosrand(1,:)-PopPosrand(2,:)))+(1-Z2).*PopPos(i,:);
                end
            else
                Z1=rand<rand;
                newPopPos(i,:)=PopPos(i,:)+(Z1*abs(randn)*((BestX+PopPosrand(1,:))/2-PopPosrand(2,:))+rand/2*(PopPosrand(3,:)-PopPosrand(4,:)));
            end
            if rand>C || t>0.8*nPop
                for j=1:Dim
                    if rand<0.2*C+0.2
                        newPopPos(i,j)=newPopPos(i,j);
                    else
                        newPopPos(i,j)=PopPos(i,j);
                    end
                end
            end
        end
        
        if rand<die
            if rand>C
                newPopPos(i,:)=(rand(1,1).*(Up-Low)+Low);
            else
                best=BestX*(lev(1,1)*(r1>r2)+abs(randn)*(r1<=r2));
                Upc=max(best);Lowc=min(best);
                newPopPos(i,:)=rand(1,1)*(Upc-Lowc)+Lowc;
            end
        end
        newPopPos(i,:)=SpaceBound(newPopPos(i,:),Up,Low);
        newPopFit(i)=BenFunctions(newPopPos(i,:));
        if newPopFit(i) < PopFit(i)
            PopFit(i)=newPopFit(i);
            PopPos(i,:)=newPopPos(i,:);
            if Q==1
                PopPos(index(nPop),:)=PopPos(i,:);
                PopFit(index(nPop))=PopFit(i);
            end
            t=0;
        else
            t=t+1;
        end
        if PopFit(i)<=BestF
            BestF=PopFit(i);
            BestX=PopPos(i,:);
        end
        [worst windex]=max(PopFit);WorstX=PopPos(windex,:);
    end
    HisBestFit(It)=BestF;
end
[FW_CCO,index_w] =max(PopFit);
FA_CCO           =mean(PopFit);
SE_CCO           =std(PopFit);
toc;
end

function  X=SpaceBound(X,Up,Low)
Dim=length(X);
if rand<rand
    S=(X>Up)+(X<Low);
    X=(rand(1,Dim).*(Up-Low)+Low).*S+X.*(~S);
else
    X=min(max(X,Low),Up);
end
end

function levy=lev(n,m)
Beta=1.5;
u =random('Normal',0, ((gamma(1+Beta)*sin(pi*Beta/2)/(gamma(((1+Beta)/2)*Beta*2^((Beta-1)/2))))^(1/Beta)),n,m);
v =random('Normal',0, 1,n,m);
levy=0.05*u./abs(v).^(-Beta);
end
