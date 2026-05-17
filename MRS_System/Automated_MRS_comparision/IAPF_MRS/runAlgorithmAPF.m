

function[metrics, tiempo_transcurrido, failedAlgth] = runAlgorithmAPF(obstacles,trajectories,simulateMovement)
    failedAlgth = true;
    tLocal = tic;
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
    
    
    for k = 1:length(obstacles)
        obstacles(k).k_rep = k_rep;
    end
    
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
    
    for i=1:length(Robots)
        Robots(i).update_costs;
        if(norm(Robots(i).goal-Robots(i).path(end,:))>=r_goal)
            fprintf("El %d Robot no ha alcanzado su punto final",i);
            return
        else
            metrics(i).d = Robots(i).cost_l;
            metrics(i).wl = Robots(i).cost_w;
        end
    
    end
    failedAlgth = false;
    tiempo_transcurrido=toc(tLocal);
    if simulateMovement
        LineTracker.lineTracker_MRS_APF_V2(Robots,obstacles);
    end
end

%%ADITIONAL FUNCTIONS
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