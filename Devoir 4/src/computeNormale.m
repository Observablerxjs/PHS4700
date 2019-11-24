function [normale] = computeNormale(point_intersection, rayonDehors)  normale = [(point_intersection(1)-4)/9 (point_intersection(2)-4)/9 (point_intersection(3)-11)/81];  normale = normale / norm(normale);  if !rayonDehors    normale = -normale;  endif
endfunction
