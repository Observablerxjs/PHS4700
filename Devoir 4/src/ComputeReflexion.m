function [rayonVec] = ComputeReflexion(rayonInitiale, normale)  rayonVec = rayonInitiale - 2*(dot(rayonInitiale,normale))*normale;
endfunction
