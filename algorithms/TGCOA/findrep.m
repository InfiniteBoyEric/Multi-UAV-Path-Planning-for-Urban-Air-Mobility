%_________________________________________________________________________%
%  Tetragonula carbonaria Optimization Algorithm (TGCOA) source code      %
%  Authors/Programmers : M.T.A. Mildret Guadalupe Martínez Gámez          %
%                        Dr. Hernán Peraza Vázquez                        %
%                                                                         %
%         e-Mails: hperaza@ipn.mx   or   hperaza@ieee.org                 %
%         Telegram: @CodebugMx                                            %
%_________________________________________________________________________%    
function [ band ] = findrep( val, vector )
% return 1= repeated  0= not repeated
band= 0;
for i=1:size(vector, 2)
    if val== vector(i)
        band=1;
        break;
    end
end
end
%_________________________________________________________________________%
%[Used in the Strategy 1:  Slight fluttering motion and cluster to warm the hive (Eq. 4) Section 2.2.1, and 
% Strategy 2:Strong fluttering motion and clustering for hive cooling (Eq. 5) Section 2.2.2]%