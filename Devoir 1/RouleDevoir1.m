clear
%
% Procedure servant à rouler le devoir 1
%
% Cas 1
posCas1=[0;0;0];
muCas1=0.0;
vaCas1=[0;0;0];
fiCas1=[1; 1; 1;1];

function  [pcm_res MI_res aa_res] = Devoir1(pos, mu, va, fi) 
  
  pcm_colis = [0; 0.1; -0.125];
  pcm_demi_sphere = [0; 0; (3*0.3)/8];
  pcm_bras_1 = [0.3+(0.5/2); 0; 0.25/2];
  pcm_bras_3 = [-(0.3+(0.5/2)); 0; 0.25/2];
  pcm_bras_2 = [0; 0.3+(0.5/2); 0.25/2];
  pcm_bras_4 = [0; -(0.3+(0.5/2)); 0.25/2];
  pcm_moteur_1 = [(0.3+0.5+0.05); 0; 0.1/2];
  pcm_moteur_3 = [-(0.3+0.5+0.05); 0; 0.1/2];
  pcm_moteur_2 = [0; (0.3+0.5+0.05); 0.1/2];
  pcm_moteur_4 = [0; -(0.3+0.5+0.05); 0.1/2];
  
  masse_colis = 1.2;
  masse_moteur = 0.4;
  masse_demi_sphere = 1.5;
  masse_bras = 0.2;
  masse_drone = masse_demi_sphere + 4*masse_bras + 4*masse_moteur;
  masse_totale = masse_colis + masse_drone;
  
  pcms=[pcm_colis,pcm_demi_sphere,pcm_bras_1,pcm_bras_2,pcm_bras_3,pcm_bras_4,...
  pcm_moteur_1,pcm_moteur_2,pcm_moteur_3,pcm_moteur_4];
  
  masses=[masse_colis,masse_demi_sphere,masse_bras,masse_bras,masse_bras,...
  masse_bras,masse_moteur,masse_moteur,masse_moteur,masse_moteur];
  
  pcm_colis_drone = [0;0;0];
  MI_res = [[0;0;0],[0;0;0],[0;0;0]];
  aa_res = [0;0;0];
  
  for i = 1:size(pcms)(2)
      pcm_colis_drone += (pcms(:,i)*masses(i))/masse_totale;
  endfor
  
  pcm_res = pcm_colis_drone + pos;
  
  #Moment d'inertie
  
  moment_inertie_demi_sphere = [ 83/320, 0, 0 ;
                                0, 83/320, 0 ;
                                0, 0, 2/5 ] ;
                                
  I_demi_sphere = masse_demi_sphere * 0.3^2 * moment_inertie_demi_sphere;
  
  I_bras = [(masse_bras/2)*(0.025^2) + (masse_bras/12)*(0.5^2), 0, 0 ;
            0, (masse_bras/2)*(0.025^2) + (masse_bras/12)*(0.5^2), 0 ;
            0, 0, masse_bras*(0.025^2) ] ;
  
  I_bras_1 = I_bras + masse_bras * Matrice_Inertie(pcm_colis_drone - pcm_bras_1);
  I_bras_2 = I_bras + masse_bras * Matrice_Inertie(pcm_colis_drone - pcm_bras_2);
  I_bras_3 = I_bras + masse_bras * Matrice_Inertie(pcm_colis_drone - pcm_bras_3);
  I_bras_4 = I_bras + masse_bras * Matrice_Inertie(pcm_colis_drone - pcm_bras_4);
  
  I_moteur = [(masse_moteur/4)*(0.05^2) + (masse_moteur/12)*(0.1^2), 0, 0;
        0, (masse_moteur/4)*(0.05^2) + (masse_moteur/12)*(0.1^2), 0;
        0, 0, (masse_moteur/2)*(0.05^2)]; 
        
  I_moteur_1 = I_moteur + masse_moteur * Matrice_Inertie(pcm_colis_drone - pcm_moteur_1);
  I_moteur_2 = I_moteur + masse_moteur * Matrice_Inertie(pcm_colis_drone - pcm_moteur_2);
  I_moteur_3 = I_moteur + masse_moteur * Matrice_Inertie(pcm_colis_drone - pcm_moteur_3);
  I_moteur_4 = I_moteur + masse_moteur * Matrice_Inertie(pcm_colis_drone - pcm_moteur_4);
  
  I_colis = [(masse_colis/12)*(0.4^2+0.25^2), 0, 0 ;
             0, (masse_colis/12)*(0.5^2+0.25^2), 0 ;
             0, 0, (masse_colis/12)*(0.7^2+0.4^2) ] ;
  
  I_colis += masse_colis * Matrice_Inertie(pcm_colis_drone - pcm_colis);
    
  I = I_demi_sphere + I_bras_1+ I_bras_2 + I_bras_3 + I_bras_4 + I_moteur_1 + ...
      I_moteur_2 + I_moteur_3 + I_moteur_4 + I_colis; 
  
  # matrice de rotation
  Ry = [cos(mu),0,sin(mu);
        0,1,0;
        -sin(mu),0,cos(mu) ] ;
        
  MI_res = Ry * I * (Ry');
  
  # on commence par calculer le moment cinetique
  L = MI_res * va;
  
  # ensuite on cherche le moment de force
  # formule : t = (rj - ri) X F
  
  force_totale = sum(fi*25);
  
  moteur_positions = [ [0.3+0.50+0.05; 0; 0.1/2], [0; 0.3+0.50+0.05; 0.1/2],...
  [-(0.3+0.50+0.05) ;0; 0.1/2],[0; -(0.3+0.50+0.05); 0.1/2] ];
  
  rj = [0;0;0];
  
  for k= 0:size(fi)(1)-1 
    rj = rj(:,1) + ( (moteur_positions(1+k*3:3+k*3)' + pos)*(fi(k+1)*25))/force_totale;
  endfor
  
  t = cross((rj-pcm_res),[force_totale*sin(mu);0;force_totale*cos(mu)]);
 
  #acceleration angulaire
  aa_res = inv(MI_res)*(t-cross(L, va));
  
endfunction

function I = Matrice_Inertie (M)
  I=[M(2)^2+M(3)^2, -M(1)*M(2), -M(1)*M(3);
     -M(2)*M(1), M(1)^2+M(3)^2, -M(2)*M(3);
     -M(3)*M(1), -M(3)*M(2), M(1)^2+M(2)^2];
endfunction

[pcmCas1 MICas1 aaCas1]=Devoir1(posCas1,muCas1,vaCas1,fiCas1);
fprintf('\nCas 1\nConditions initiales\n');
fprintf('  Position drone (m) = ( %10.5f,  %10.5f,  %10.5f )\n',posCas1(1),posCas1(2),posCas1(3));
fprintf('  Rotation drone (r) = %10.5f\n',muCas1);
fprintf('  Vitesse angulaire drone (r/s) = ( %10.5f,  %10.5f,  %10.5f )\n',vaCas1(1),vaCas1(2),vaCas1(3));
fprintf('  Forces (N) = ( %10.5f,  %10.5f,  %10.5f,  %10.5f ) \n',fiCas1(1),fiCas1(2),fiCas1(3),fiCas1(4));
fprintf('Resultats avion sur la piste \n');
fprintf('  Centre de masse (m) = ( %10.5f,  %10.5f,  %10.5f )\n',pcmCas1(1),pcmCas1(2),pcmCas1(3));
fprintf('  Moment inertie (kg/m^2) =\n   %10.5f & %10.5f & %10.5f  \n   %10.5f & %10.5f & %10.5f  \n   %10.5f & %10.5f & %10.5f \n',...
     MICas1(1,1),MICas1(1,2),MICas1(1,3),MICas1(2,1),MICas1(2,2),MICas1(2,3),MICas1(3,1),MICas1(3,2),MICas1(3,3));
fprintf('  acc angulaire (r/s^2) = ( %10.5f,  %10.5f,  %10.5f ) \n\n',aaCas1(1),aaCas1(2),aaCas1(3));
%
% Cas 2
posCas2=[2.5;0.0;30.5];
muCas2=0.05;
vaCas2=[0.0; 0.05; 0.001];
fiCas2=[0.4; 0.5; 0.6; 0.5];
[pcmCas2 MICas2 aaCas2]=Devoir1(posCas2,muCas2,vaCas2,fiCas2);
fprintf('\nCas 2\nConditions initiales\n');
fprintf('  Position drone = ( %10.5f,  %10.5f,  %10.5f )\n',posCas2(1),posCas2(2),posCas2(3));
fprintf('  Rotation drone (r) = %10.5f \n',muCas2);
fprintf('  Vitesse angulaire drone (r/s)= ( %10.5f,  %10.5f,  %10.5f )\n',vaCas2(1),vaCas2(2),vaCas2(3));
fprintf('  Forces/Fmax (N) = ( %10.5f,  %10.5f,  %10.5f,  %10.5f )\n',fiCas2(1),fiCas2(2),fiCas2(3),fiCas2(4));
fprintf('\nResultats avion en vol\n');
fprintf('  Centre de masse (m) = ( %10.5f,  %10.5f,  %10.5f )\n',pcmCas2(1),pcmCas2(2),pcmCas2(3));
fprintf('  Moment inertie  (kg/m^2) =\n   %10.5f & %10.5f & %10.5f \\\\ \n   %10.5f & %10.5f & %10.5f \\\\ \n   %10.5f & %10.5f & %10.5f \\\\ \n',...
     MICas2(1,1),MICas2(1,2),MICas2(1,3),MICas2(2,1),MICas2(2,2),MICas2(2,3),MICas2(3,1),MICas2(3,2),MICas2(3,3));
fprintf('  acc angulaire (r/s^2) = ( %10.5f,  %10.5f,  %10.5f )\n\n',aaCas2(1),aaCas2(2),aaCas2(3));
