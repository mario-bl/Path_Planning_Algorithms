%*********************DEFINICION CONSTANTES***************************
% ********************************************************************
tic;
k_att=10;
k_rep=8;
k_rep_v=3;
r_goal=0.5;%radio respecto al punto objetivo que se considera que el robot 
r_obs=1.5; %radio de los obstaculos, mayor que el de una celda 
d_inf=1;% distancia de influencia del ca mpo repulsivo de los obstaculos
d_inf_robs=2;
 
dT=0.01;%incremento temporal 
J=10000;%cantidad de pasos
Theta0=1.57;%Umbral para juzgar jitter
f=0.8;%Jitter factor
m=1;%masa del robot--> F=a  
v_max =1; % Velocidad máxima permitida
l=v_max*dT;%Step length
cts=struct('k_att',k_att,'k_rep',k_rep,'k_rep_v',k_rep_v,'r_obs',r_obs,'d_inf',d_inf,'d_inf_robs',d_inf_robs,'l',l);
virtualFlag=true;%Bandera que permite añadir obstaculos virtuales

mapWidth = 20.5;  % Width of the map
mapHeight = 20.7; % Height of the map
% ********************************************************************


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

%*************POSICIONES DE SALIDA DE LOS ROBOTS************************
%***********************************************************************
start_path1=[0,20];
start_path2=[0,0];
StartPoints=[start_path1;start_path2];
% % %definimos punto a alcanzar
end_path1=[20,0];
end_path2=[20,20];
EndPoints=[end_path1;end_path2];

if(~targetsCheck(obstacles,EndPoints)) || (~startsCheck(obstacles,StartPoints))
    return;
end
%***********************************************************************
Robots=Robot.empty(); 
n_robots=size(StartPoints,1);
for j=1:n_robots
    Robots(j)=Robot(StartPoints(j,:),EndPoints(j,:),v_max,obstacles);
end

it=0;%iterador para salier en caso de no convergencia

%Bucle principal 

while (it<J) && continueAPF_algorithm(Robots,r_goal)
    for i=1:length(Robots)   
        if Robots(i).APF_flag
            F=IAPF_MRS(Robots(i),Robots,cts,n_obs_r,i);
            Robots(i).nextPosition(F,dT,l,f,v_max,r_goal)
            checkLocalMinima(Robots(i),l,d_inf,k_rep_v);%Añadimos obstaculo virtual en caso de minimo local a cada Robot 
        end
    end
    it=it+1;
end

tiempo_transcurrido=toc;
fprintf('Tiempo transcurrido para realizar los calculos del algoritmo: %.2f\n',tiempo_transcurrido);
for i=1:length(Robots)
    Robots(i).updateW;
    if(norm(Robots(i).goal-Robots(i).path(end,:))>=r_goal)
        fprintf("El %d Robot no ha alcanzado su punto final",i);
    else
        fprintf('Robot: %d Longitud = %.2f Curvatura promedia = %.2f\n',i, Robots(i).cost_l,Robots(i).cost_w);
    end

end

figure
title("IAPF-MRS")
xlabel('X')
ylabel('Y')
xlim([0,mapWidth])
ylim([0,mapHeight])

upObs=updateObstacles(Robots,obstacles);

for i = 1:length(upObs)
    % Extract obstacle properties
    x = upObs(i).x;
    y = upObs(i).y;
    r = upObs(i).r;
    c=upObs(i).colour;
    
    % Draw a filled circle (disk) using the rectangle function
    rectangle('Position', [x - r, y - r, 2 * r, 2 * r], ...
              'Curvature', [1, 1], ... % Makes it a circle
              'FaceColor', c, ...   % Fill color
              'EdgeColor', 'black');   % Border color
end
hold on

colors = lines(length(Robots)); % Colores distintos para cada robot
h = gobjects(1, length(Robots)); % Handles para las líneas
p = gobjects(1, length(Robots)); % Handles para los puntos

for i = 1:length(Robots)
    h(i) = plot(NaN, NaN, 'Color', colors(i, :), 'LineWidth', 1.5); % Línea para el robot i
    p(i) = scatter(NaN, NaN, 'filled', 'MarkerFaceColor', colors(i, :)); % Puntos para el robot i
end

% Animación
max_points = max(arrayfun(@(robot) size(robot.path,1), Robots)); % Máximo número de puntos

for i = 1:5:max_points
    for j = 1:length(Robots)
        if i <= size(Robots(j).path,1)
            % Verificar si el objeto gráfico aún es válido
            if isvalid(h(j)) && isvalid(p(j))
                % Actualizar la línea y el punto del robot j
                set(h(j), 'XData', Robots(j).path(1:i, 1), 'YData', Robots(j).path(1:i, 2));
                set(p(j), 'XData', Robots(j).path(i, 1), 'YData', Robots(j).path(i, 2));
            else
                error('Objeto gráfico eliminado o no válido.');
            end
        end
    end
    
    pause(0.01); % Controlar la velocidad de la animación
    drawnow; % Actualizar la figura
end


