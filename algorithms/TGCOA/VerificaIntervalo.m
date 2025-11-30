function o=VerificaIntervalo (v, r, ub, lb)
      Flag4ub=v(r,:)>ub;
      Flag4lb=v(r,:)<lb;
      dim=size (v,2);
      Boundary_no= size(ub,2);
        if Boundary_no==1
            for i=1:dim
                if (Flag4ub (i)|| Flag4lb (i))
                    v(r,i)=lb+(ub-lb)*rand();
                end
            end
        else 
            for i=1:dim
                if (Flag4ub (i)|| Flag4lb (i))
                   v(r,i)=lb(i)+(ub(i)-lb(i))*rand();
                end
            end
        end
        o= v;
end