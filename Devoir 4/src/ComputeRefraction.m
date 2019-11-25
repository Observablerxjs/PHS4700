function [rayonVec] = ComputeRefraction(rayonInital, normale, theta1, theta2)        epsilon = 0.000001;        if (sin(theta1) - sin(theta2)) > epsilon          A = ((rayonInital + cos(theta1)*normale)/sin(theta1))*sin(theta2);          B = -normale * cos(theta2);          # rayon réfracté          rayonVec = A + B;        else          rayonVec = rayonInital;        end
endfunction
