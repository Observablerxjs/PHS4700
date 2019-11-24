function [rayonVec] = ComputeRefraction(rayonInital, normale, theta1, theta2)        A = (rayonInital + cos(theta1)*normale)/sin(theta1);        B = -normale * cos(theta2);        # rayon réfracté        rayonVec = A + B;
endfunction
