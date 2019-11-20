%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Roule Devoir 4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc ; clear all ; format loose %compact 
format long;
close all ; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%% D�finir les cas %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cm   = [4,4,11]; %--- centre de masse ellipso�de
rad    = 3; %--- x^2/(rad^2), y^2/(rad^2)
bval   = 9; %--- z^2/(bval^2)
Lame = [3 4; 3 5; 12 17];
nout = [1 1   1   1.2];
nin  = [1  1.5 1.5 1];
dep  = [0 0 5; 0 0 5; 0 0 0; 0 0 5];

for itst=1:1
  tic
  clf;
  hold;
  scatter3(dep(itst,1),dep(itst,2),dep(itst,3));
  hold on;
  
  pointHaut = cm + [-rad,0,bval];
  pointBas = cm + [-rad,0,-bval];
  axeZ = [0,0,1];
  scatter3(pointHaut(1),pointHaut(2),pointHaut(3));
  hold on;
  scatter3(pointBas(1),pointBas(2),pointBas(3));
  hold on;
  vec1 = pointHaut - dep(itst,:); 
##  quiver3(dep(itst,1),dep(itst,2),dep(itst,3),vec1(1),vec1(2),vec1(3));
##  hold on;
##  vec2 = pointBas - dep(itst,:);
##  quiver3(dep(itst,1),dep(itst,2),dep(itst,3),vec2(1),vec2(2),vec2(3));
##  hold on;
  quiver3(0,0,0,pointHaut(1),pointHaut(2),pointHaut(3));
  hold on;
##  quiver3(0,0,0,pointBas(1),pointBas(2),pointBas(3));
##  hold on;
  
  polMin = acos(dot(pointHaut, axeZ) / (norm(pointHaut) * norm(axeZ)));
  polMax = acos(dot(axeZ, pointBas) / (norm(axeZ) * norm(pointBas)));
  
  rayX = sin(polMin)*cos(atan(pointHaut(2)/pointHaut(1)));
  rayY = sin(polMin)*sin(atan(pointHaut(2)/pointHaut(1)));
  rayZ = cos(polMin)
  
  scatter3(rayX,rayY,rayZ);
  hold on;
  scatter3(0,0,17);
  hold on;
  
##  [x, y, z] = ellipsoid(cm(1),cm(2),cm(3),rad,rad,bval,30);
##  hold on;
##  surf(x, y, z,'edgecolor','b','EdgeAlpha',0.5)
##  hold on;
  
##  Face1x=[Lame(1,1) Lame(1,1) Lame(1,1) Lame(1,1) Lame(1,1)];
##  Face2x=[Lame(1,2) Lame(1,2) Lame(1,2) Lame(1,2) Lame(1,2)];
##  Face12y=[Lame(2,1) Lame(2,2) Lame(2,2) Lame(2,1) Lame(2,1)];
##  Face12z=[Lame(3,1) Lame(3,1) Lame(3,2) Lame(3,2) Lame(3,1)];
##
##  Face34x=[Lame(1,1) Lame(1,2) Lame(1,2) Lame(1,1) Lame(1,1)];  
##  Face3y=[Lame(2,1) Lame(2,1) Lame(2,1) Lame(2,1) Lame(2,1)];
##  Face4y=[Lame(2,2) Lame(2,2) Lame(2,2) Lame(2,2) Lame(2,2)];
##  Face34z=Face12z;
##
##  Face56x=Face34x;
##  Face56y=[Lame(2,1) Lame(2,1) Lame(2,2) Lame(2,2) Lame(2,1)];
##  Face5z=[Lame(3,1) Lame(3,1) Lame(3,1) Lame(3,1) Lame(3,1)];
##  Face6z=[Lame(3,2) Lame(3,2) Lame(3,2) Lame(3,2) Lame(3,2)];
##
##  line(Face1x,Face12y,Face12z,'Color',[0.6 0.6 0.6]);
##  line(Face2x,Face12y,Face12z,'Color',[0.6 0.6 0.6]);
##  line(Face34x,Face3y,Face34z,'Color',[0.6 0.6 0.6]);
##  line(Face34x,Face4y,Face34z,'Color',[0.6 0.6 0.6]);
##  line(Face56x,Face56y,Face5z,'Color',[0.6 0.6 0.6]);
##  line(Face56x,Face56y,Face6z,'Color',[0.6 0.6 0.6]);
##  axis equal
##  [xi,yi,zi,face]=Devoir4(nout(itst),nin(itst),dep(itst,:));
##  nbpoint=length(face);
##  for ipoint=1:nbpoint
##    if face(ipoint) == 1
##      plot3([xi(ipoint)],[yi(ipoint)],[zi(ipoint)],'r.');
##    elseif face(ipoint) == 2
##      plot3([xi(ipoint)],[yi(ipoint)],[zi(ipoint)],'c.');
##    elseif face(ipoint) == 3
##      plot3([xi(ipoint)],[yi(ipoint)],[zi(ipoint)],'g.');
##    elseif face(ipoint) == 4
##      plot3([xi(ipoint)],[yi(ipoint)],[zi(ipoint)],'y.');
##    elseif face(ipoint) == 5
##      plot3([xi(ipoint)],[yi(ipoint)],[zi(ipoint)],'b.');
##    elseif face(ipoint) == 6
##      plot3([xi(ipoint)],[yi(ipoint)],[zi(ipoint)],'m.');
##    end
##  end
##  hold;
##  toc
##  pause
end
