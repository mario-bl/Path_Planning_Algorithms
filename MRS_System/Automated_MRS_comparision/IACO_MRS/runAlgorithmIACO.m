%******************SIMULACION DESPLIEGUE EN MRS CON ALGORITMO IACO*******************************

function [metrics, tiempo_transcurrido, failedAlgth] = runAlgorithmIACO(mapa, trajectories, simulateMovement)
failedAlgth = true;
%******************DEFINICION DE MAPA DE TRABAJO ***************
%***************************************************************
tLocal = tic;
mapSize = size(mapa);
%*************POSICIONES DE SALIDA/LLEGADA DE LOS ROBOTS************************
%*******************************************************************************

[startNodes, goalNodes, correctSelection] = generateNodesV2(trajectories, mapSize);
if ~correctSelection
    return
end


if size(startNodes,1) ~= size(goalNodes,1)  
    fprintf("Please check start and end points\n")
    return
end

numRobots = size(startNodes, 1);
metrics(1,numRobots) = struct('d',0,'wl',0);
for r=1:numRobots
    startNode = startNodes(r, :);
    goalNode = goalNodes(r, :);
    if (~checkRobPosition(startNode,goalNode,mapa))
        return;
    end
end

%*******************************************************************************
numAnts=10;
maxIter=50;
alpha=1;
beta=8;
rho=0.1;
bestPaths= struct('path',{},'cost', {},'rot_cost', {}); %Nos quedaremos con los tres mejores caminos
bestPathsRobots= struct('path',{},'cost', {},'rot_cost', {}); %Nos quedaremos con los tres mejores caminos
for r = 1:numRobots
    
    startNode = startNodes(r, :);
    goalNode = goalNodes(r, :);
    
    % Inicialización de feromonas y heurística (separadas por robot)
    pherom = inic_map_feromonas(mapa);
    ants = Ant.empty(numAnts, 0);
    for i = 1:numAnts
        ants(i) = Ant(startNode,goalNode);
    end
    %Limpiamos los caminos previamente guardados
    bestPaths=resetBestPaths(3);
  
    for t = 1:maxIter
        for i = 1:numAnts
            ants(i).reset(startNode,goalNode);
            while ~isequal(goalNode, [ants(i).y, ants(i).x])
                heuristic=Yang_heuristic(mapa,goalNode,startNode,ants(i));
                celdasVecinas = celdas_accesibles(mapa, ants(i));
                if isempty(celdasVecinas)
                    fprintf('Hormiga %d atrapada en (%d, %d)', ants(i).x, ants(i).y);
                    continue;
                end
                probabilidades = computeProbabilities(celdasVecinas, ants(i), pherom, heuristic, alpha, beta);
                nextNode = selectNextNode(probabilidades, celdasVecinas, [ants(i).y, ants(i).x]);
                ants(i).move(nextNode);
            end
        end
        pherom = updatePherom(pherom, ants, rho);
        for i = 1:length(ants)
            rotationCostCalc(ants(i))
        end
        bestPaths = updateBestPaths(ants, bestPaths);%mejores caminos para 1 robot
    end
    %bestPaths=updateCosts(bestPaths);
    bestPathsRobots{r} = bestPaths(1);

end
bestPathsRobots = collisionAvoidance(bestPathsRobots,mapa);
%Estudio de colisiones y recalculo de las mismas

%clase para simular robots
robots = RobotACO.empty(length(bestPathsRobots), 0);
for i=1:length(bestPathsRobots)
    if (norm(bestPathsRobots{i}.path(end,:)-goalNodes(i,:))==0)
        robots(i)=RobotACO(bestPathsRobots{i}.path,length(bestPathsRobots),bestPathsRobots{i}.cost,bestPathsRobots{i}.rot_cost,0);
        robots(i).update_costs();
        %robots(i).smoothPathWithBSpline(3);%ojo con esto
        %fprintf("Robot: %d Longitud=%.2f Curvatura promedia =%.2f\n",i,robots(i).cost_l,robots(i).cost_w)
        metrics(i).d = robots(i).cost_l;
        metrics(i).wl = robots(i).cost_w;
    else
        fprintf("Robot %d of IACO couldn't achive the end point\n",i);
        return
    end

end
failedAlgth = false;
applyCollisionAvoidance(robots); %%BASTANTE CUESTIONABLE SI ES NECESARIO
tiempo_transcurrido=toc(tLocal);

%fprintf('Tiempo transcurrido para realizar los calculos del algoritmo: %.2f\n',tiempo_transcurrido);
if simulateMovement
    LineTracker.MRS_ACO_Tracker(robots,mapa);
    %plotMRSpaths(robots,mapa);
end
end



function [matriz_vecinos]=celdas_accesibles(mapaObs,ant)
    [n,m]=size(mapaObs);
    matriz_vecinos=[];
    
    for i=max(1,ant.y-1):min(n,ant.y+1)
        for j=max(1,ant.x-1):min(m,ant.x+1)
            if mapaObs(i,j)==1 && (ant.y~=i || ant.x~=j) %Condicion espacio libre y no se queda atrapada en el mismo nodo
                matriz_vecinos=[matriz_vecinos;[i,j]];
                %%Se almacena las coordenadas del espacio libre 
            end
        end
    end
    
end


function bestPaths = resetBestPaths(n)
% Crea un vector de n estructuras vacías con los campos definidos

    % Inicializa el primer elemento
    vacio = struct('path',{},'cost', {},'rot_cost', {});
    
    % Rellena el resto
    bestPaths = repmat(vacio, 1, n);
end