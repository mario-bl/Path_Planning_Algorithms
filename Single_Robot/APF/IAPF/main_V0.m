%*********************DEFINICION CONSTANTES***************************
% ********************************************************************
tic;
k_att=10;
k_rep=5;
k_rep_v=3;
r_goal=0.5;%radio respecto al punto objetivo que se considera que el robot 
r_obs=1.5; %radio de los obstaculos, mayor que el de una celda 
d_inf=1;% distancia de influencia del campo repulsivo de los obstaculos


dT=0.01;%incremento temporal 
J=10000;%cantidad de pasos
Theta0=1.57;%Umbral para juzgar jitter
f=0.8;%Jitter factor
m=1;%masa del robot--> F=a  
cts=struct('k_att',k_att,'k_rep',k_rep,'r_obs',r_obs,'d_inf',d_inf);
v_max =1; % Velocidad máxima permitida
l=v_max*dT;
virtualFlag=true;%Bandera que permite añadir obstaculos virtuales

mapWidth = 20.5;  % Width of the map
mapHeight = 20.7; % Height of the map

%***********************************************************************

%*************POSICIONES DE LOS OBSTÁCLOS A LO LARGO DEL MAPA***********
%***********************************************************************
obstacles(1) = struct('x', 5.0, 'y', 2.0, 'r', 1.69, 'colour', 'black', 'k_rep', k_rep);%, 'd_inf', d_inf);
obstacles(2) = struct('x', 11.0, 'y', 3.0, 'r', 1.69, 'colour', 'black', 'k_rep', k_rep);%, 'd_inf', d_inf);
obstacles(3) = struct('x', 18.5, 'y', 3.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);%, 'd_inf', d_inf);
obstacles(4) = struct('x', 3.5, 'y', 5.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);%, 'd_inf', d_inf);
obstacles(5) = struct('x', 18.0, 'y', 7.0, 'r', 1.69, 'colour', 'black', 'k_rep', k_rep);%, 'd_inf', d_inf);
obstacles(6) = struct('x', 10.0, 'y', 8.0, 'r', 1.69, 'colour', 'black', 'k_rep', k_rep);%, 'd_inf', d_inf);
obstacles(7) = struct('x', 13.5, 'y', 7.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);%, 'd_inf', d_inf);
obstacles(8) = struct('x', 5.5, 'y', 8.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);%, 'd_inf', d_inf);
obstacles(9) = struct('x', 13.5, 'y', 11.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);%, 'd_inf', d_inf);
obstacles(10) = struct('x', 17.5, 'y', 12.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);%, 'd_inf', d_inf);
obstacles(11) = struct('x', 3.0, 'y', 14.0, 'r', 1.69, 'colour', 'black', 'k_rep', k_rep);%, 'd_inf', d_inf);
obstacles(12) = struct('x', 8.0, 'y', 15.0, 'r', 1.69, 'colour', 'black', 'k_rep', k_rep);%, 'd_inf', d_inf);
obstacles(13) = struct('x', 14.5, 'y', 16.5, 'r', 2.26, 'colour', 'black', 'k_rep', k_rep);%, 'd_inf', d_inf);
obstacles(14) = struct('x', 2.5, 'y', 18.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);%, 'd_inf', d_inf);
obstacles(15) = struct('x', 8.5, 'y', 19.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);%, 'd_inf', d_inf);
n_obs_r=length(obstacles);
%***********************************************************************

%*************POSICIONES DE SALIDA/LLEGADA DEL ROBOT************************
%***********************************************************************
xGoal= 20;
yGoal =20;
goal = [xGoal, yGoal];
xStart=0;
yStart=0;

if(~targetPointCheck(obstacles,goal)) || (~startPointCheck(obstacles,[xStart,yStart]))
    return;
end
%***********************************************************************

rob=Robot(xStart,yStart,goal,v_max);
it=0;%iterador para salier en caso de no convergencia

%Bucle principal 
%while norm([goal(1) - rob.x, goal(2) - rob.y]) > r_goal && it < J
while (sqrt((xGoal-rob.x)^2+(yGoal-rob.y)^2)>r_goal) && it < J

    F=IAPF_calculos(rob,obstacles,goal,cts);
    rob.nextPosition(F,dT,l,f,v_max)
    [obstacles,virtualFlag]=checkLocalMinima(rob,obstacles,l,d_inf,virtualFlag,k_rep_v);%Añadimos obstaculo virtual en caso de minimo local
    % [obstacles,virtualFlag]=checkLocalMinima(rob,obstacles,n_obs_r,l,d_inf,virtualFlag,k_rep_v);%Añadimos obstaculo virtual en caso de minimo local
    it=it+1;
end

if norm([goal(1) - rob.x, goal(2) - rob.y]) <= r_goal
    rob.update_costs;
    tiempo_transcurrido=toc;
    fprintf('El algoritmo ha convergido en %d iteraciones.\n', it);
    fprintf('Coste del camino en distancia: %.2f \n', rob.cost_l);
    fprintf('Curvatura media del robot: %.2f \n', rob.cost_w);
    fprintf('Tiempo transcurrido para realizar los calculos del algoritmo: %.2f\n',tiempo_transcurrido);
else
    disp('El algoritmo no ha convergido.');

end

figure
plot(rob.path(:,1),rob.path(:,2),'b','LineWidth',1.5);
hold on 
xlim([-1,mapWidth+1])
ylim([-1,mapHeight+1])
title("IAPF-SR algorithm path")
xlabel('X (dm)')
ylabel('Y (dm)')

for i = 1:length(obstacles)
    % Extract obstacle properties
    x = obstacles(i).x;
    y = obstacles(i).y;
    r = obstacles(i).r;
    c=obstacles(i).colour;
    
    % Draw a filled circle (disk) using the rectangle function
    rectangle('Position', [x - r, y - r, 2 * r, 2 * r], ...
              'Curvature', [1, 1], ... % Makes it a circle
              'FaceColor', c, ...   % Fill color
              'EdgeColor', 'black');   % Border color
end

plot(xGoal, yGoal, '*r', 'MarkerSize', 15, 'LineWidth', 2);
plot(xStart,yStart,'or', 'MarkerSize', 15, 'LineWidth', 2);

