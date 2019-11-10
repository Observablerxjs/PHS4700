function [Touche] = VerifierConditions(pos_sommets,posBloc,posBalle)
  # Verifier si aucun des points n'a touche le sol pour Balle et Bloc
  
  rBalle = 0.02;
  
  if (posBalle(3)-rBalle) <= 0
    Touche = 1;
    return;
  end

  for i=1:8
    if pos_sommets{i}(3) <= 0
      Touche = 1;
      return;
    end
  end
   
  # Verifier Colision
  
  bloc_faces = {[1,2,3], [1,3,5], [2,4,6], [5,6,7], [1,2,5],[3,4,7]};
  
  for j=1:6
    
    p0 = pos_sommets{bloc_faces{j}(1)};
    p1 = pos_sommets{bloc_faces{j}(2)}; 
    p2 = pos_sommets{bloc_faces{j}(3)};

    plane_vectors = [p0;p1;p2];
    
    plane = createPlane(plane_vectors);
    
    line_vector =  posBloc - posBalle;
    
    line = [posBalle(1) posBalle(2) posBalle(3)  line_vector(1) line_vector(2) line_vector(3)];
    intersection_point = intersectLinePlane(line, plane);
    
    x_condition = posBloc(1) - 0.03<= intersection_point(1) && intersection_point(1) <= posBloc(1) + 0.03;
    y_condition = posBloc(2) - 0.03<= intersection_point(2) && intersection_point(2) <= posBloc(2) + 0.03;
    z_condition = posBloc(3) - 0.03<= intersection_point(3) && intersection_point(3) <= posBloc(3) + 0.03;
    
    if  x_condition && y_condition && z_condition
      d = distancePoints(posBalle, intersection_point);
      
      if d <= rBalle
         Touche = 0;
         return;
       end
    end
   
   end
    
   Touche = -1;
   
endfunction
