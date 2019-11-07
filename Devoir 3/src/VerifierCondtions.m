function [Resultat] = VerifierConditions(posBloc,posBalle)  # Verifier si aucun des points n'a touche le sol pour Balle et Bloc
  Touche = 1;
  # Verifier Colision
  Touche = 0;
   # Sinon
   Touche = -1;
endfunction
