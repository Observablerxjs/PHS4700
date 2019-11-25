function [normale] = computeNormale(point_intersection,n1,n2, rayonDehors)    facteur = 1;    if n1 > n2 || (n1 == n2 && !rayonDehors)    facteur = -1;  endif    normale = [(point_intersection(1)-4)/9 (point_intersection(2)-4)/9 (point_intersection(3)-11)/81];  normale = (normale / norm(normale)) * facteur;    
endfunction
