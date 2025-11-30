%_________________________________________________________________________%
%  Tetragonula carbonaria Optimization Algorithm (TGCOA) source code      %
%  Authors/Programmers : M.T.A. Mildret Guadalupe Martínez Gámez          %
%                        Dr. Hernán Peraza Vázquez                        %
%                                                                         %
%         e-Mails: hperaza@ipn.mx   or   hperaza@ieee.org                 %
%         Telegram: @CodebugMx                                            %
%_________________________________________________________________________%               
function [ vTg ] = vectorTg( SearchAgents_no,nb )
c=1; 
vTg=[];
 while(c<=nb)
    idx =round( 1+ (SearchAgents_no-1) * rand());
    if ~findrep(idx, vTg)
        vTg(c) = idx;
        c=c+1;
    end
 end
%_________________________________________________________________________%
