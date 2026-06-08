clear;
clc;
tic;
%*********************DEFINICION CONSTANTES**************************%


%********************************************************************%
k_att=10;
k_rep=8;
k_rep_v=3;
r_goal=0.5;%radio respecto al punto objetivo que se considera que el robot 
r_obs=1.5; %radio de los obstaculos, mayor que el de una celda 
d_inf=1;% distancia de influencia del ca mpo repulsivo de los obstaculos
d_inf_robs=3;
 
dT=0.01;%incremento temporal 
J=10000;%cantidad de pasos
f=0.8;%Jitter factor 
v_max =1; % Velocidad máxima permitida
l=v_max*dT;%Step length
cts=struct('k_att',k_att,'k_rep',k_rep,'k_rep_v',k_rep_v,'r_obs',r_obs,'d_inf',d_inf,'d_inf_robs',d_inf_robs,'l',l);
% ********************************************************************

mapWidth = 20.5;  % Width of the map
mapHeight = 20.7; % Height of the map
% ********************************************************************


%*************POSICIONES DE LOS OBSTÁCLOS A LO LARGO DEL MAPA***********
%***********************************************************************
obstacles(1) = struct('x', 5.0, 'y', 2.0, 'r', 1.69, 'colour', 'black', 'k_rep', k_rep);
obstacles(2) = struct('x', 11.0, 'y', 3.0, 'r', 1.69, 'colour', 'black', 'k_rep', k_rep);
obstacles(3) = struct('x', 18.5, 'y', 3.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);
obstacles(4) = struct('x', 3.5, 'y', 5.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);
obstacles(5) = struct('x', 18.0, 'y', 7.0, 'r', 1.69, 'colour', 'black', 'k_rep', k_rep);
obstacles(6) = struct('x', 10.0, 'y', 8.0, 'r', 1.69, 'colour', 'black', 'k_rep', k_rep);
obstacles(7) = struct('x', 13.5, 'y', 7.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);
obstacles(8) = struct('x', 5.5, 'y', 8.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);
obstacles(9) = struct('x', 13.5, 'y', 11.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);
obstacles(10) = struct('x', 17.5, 'y', 12.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);
obstacles(11) = struct('x', 3.0, 'y', 14.0, 'r', 1.69, 'colour', 'black', 'k_rep', k_rep);
obstacles(12) = struct('x', 8.0, 'y', 15.0, 'r', 1.69, 'colour', 'black', 'k_rep', k_rep);
obstacles(13) = struct('x', 14.5, 'y', 16.5, 'r', 2.26, 'colour', 'black', 'k_rep', k_rep);
obstacles(14) = struct('x', 2.5, 'y', 18.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);
obstacles(15) = struct('x', 8.5, 'y', 19.5, 'r', 1.13, 'colour', 'black', 'k_rep', k_rep);
n_obs_r=length(obstacles);
%***********************************************************************

%*************POSICIONES DE SALIDA DE LOS ROBOTS************************
%***********************************************************************
trajectories(1) = struct('x0', 0,  'y0', 20, 'xObj', 20, 'yObj', 0);
trajectories(2) = struct('x0', 0,  'y0', 0,  'xObj', 20, 'yObj', 20);

if(~trajectoriesCheck(obstacles, trajectories))
    return;
end
%***********************************************************************

Robots=RobotAPF.empty(); 
n_robots=size(trajectories,2);
metrics(1,n_robots) = struct('d',0,'wl',0);
for j=1:n_robots
    Robots(j)=RobotAPF([trajectories(j).x0,trajectories(j).y0], ...
                    [trajectories(j).xObj,trajectories(j).yObj], ...
                    v_max,obstacles,n_robots);
end

it=0;
while (it<J) && continueAPF_algorithm(Robots,r_goal)

    for i=1:length(Robots)   
        if Robots(i).APF_flag
            F=IAPF_MRS(Robots(i),Robots,cts,i);
            Robots(i).nextPosition(F,dT,l,f,v_max,r_goal,i)
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
        fprintf('Robot: %d Longitud=%.2f Coste rotaciones= %.2f\n',i, Robots(i).cost_l,Robots(i).cost_w);
    end

end
LineTracker.lineTracker_MRS_APF_V2(Robots,obstacles);


%%ADITIONAL FUNCTIONS%%
function [flag] = trajectoriesCheck(obs, trajectories)
    flag = true;
    
    for i=1:length(obs)
        for k=1:size(trajectories,2)
    
            if(sqrt((trajectories(k).x0-obs(i).x)^2+(trajectories(k).y0-obs(i).y)^2)<=obs(i).r)
                fprintf("El punto inicial seleccionado (%d,%d) esta dentro de un obstaculo \n ",trajectories(k).x0,trajectories(k).y0)
                flag=false;
                break;
            elseif (sqrt((trajectories(k).xObj-obs(i).x)^2+(trajectories(k).yObj-obs(i).y)^2)<=obs(i).r)
                fprintf("El punto final seleccionado (%d,%d) esta dentro de un obstaculo \n ",trajectories(k).xObj,trajectories(k).yObj)
                flag = false;
            end
    
        end
    end

end