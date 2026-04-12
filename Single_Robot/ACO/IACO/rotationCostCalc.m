function rotationCostCalc(ant)
%funcion para calcular metrica de coste por rotacion 
n=length(ant.path); 
rot_cost=0;
oldTheta=atan2(ant.path(n,1)-ant.path(1,1),ant.path(n,2)-ant.path(1,2));%orientacion inicial

for i=2:n
    newTheta=atan2(ant.path(i,1)-ant.path(i-1,1),ant.path(i,2)-ant.path(i-1,2));
    rot_cost=rot_cost+abs(oldTheta-newTheta);
    oldTheta=newTheta;    

end
rot_cost=rot_cost/n;%Promedio de rotaciones 
ant.rot_cost=rot_cost;
end
% oldTheta=atan2(ant.path(n,2)-ant.path(1,2),ant.path(n,1)-ant.path(1,1));%orientacion inicial
% 
% for i=2:n
%     newTheta=atan2(ant.path(i,2)-ant.path(i-1,2),ant.path(i,1)-ant.path(i-1,1));
%     rot_cost=rot_cost+abs(oldTheta-newTheta);
%     oldTheta=newTheta;    
% 
% end

