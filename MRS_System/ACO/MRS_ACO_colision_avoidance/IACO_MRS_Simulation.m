%******************SIMULACION DESPLIEGUE EN MRS CON ALGORITMO IACO*******************************

%******************DEFINICION DE MAPA DE TRABAJO ***************
%***************************************************************
tic;
% mapa=ones(20,20);
% mapa(1,8:9)=0;
% mapa(2,8:9)=0;
% mapa(19,8:9)=0;
% mapa(20,8:9)=0;
mapa = [
    1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 1 1 1 1;
    1 0 0 1 1 1 1 0 0 1 1 1 1 1 1 1 1 1 1 1;
    1 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 1 1 1 1;
    1 1 1 1 1 1 0 0 0 1 1 1 0 0 0 0 1 1 1 1;
    1 0 0 0 1 1 0 0 0 1 1 1 0 0 0 0 1 1 1 1;
    1 0 0 0 1 1 0 0 0 1 1 1 1 1 1 1 1 1 1 1;
    1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 0 0 1 1 0 0 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1; 
    1 1 1 1 0 0 1 1 0 0 0 1 1 1 1 1 1 1 1 1;
    1 1 1 1 0 0 1 1 0 0 0 1 0 0 1 1 0 0 0 1;
    1 1 1 1 1 1 1 1 0 0 0 1 0 0 1 1 0 0 0 1;
    1 1 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1;
    1 1 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
    1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0 1;
    1 1 1 0 0 0 1 1 1 0 0 0 1 1 1 1 1 0 0 1;
    1 1 1 0 0 0 1 1 1 0 0 0 1 1 1 1 1 1 1 1;
    1 1 1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
];
[nRows,nCols]=size(mapa);

%*************POSICIONES DE SALIDA/LLEGADA DE LOS ROBOTS************************
%*******************************************************************************
% startNode1=[15, 1];
% goalNode1=[15,20];
% startNode2=[15, 20];
% goalNode2=[15,1];
startNode1=[1, 1];
goalNode1=[20,20];
startNode2=[20, 1];
goalNode2=[1,19];

robotStartNodes = [startNode1;startNode2];
robotGoalNodes  = [goalNode1;goalNode2];
numRobots = size(robotStartNodes, 1);
for r=1:numRobots
    startNode = robotStartNodes(r, :);
    goalNode = robotGoalNodes(r, :);
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
bestPaths= struct('opt_path', {}, 'cost', {},'path',{} ,'rot_cost', {}); %Nos quedaremos con los tres mejores caminos
bestPathsRobots= struct('opt_path', {}, 'cost', {},'path',{} ,'rot_cost', {}); %Nos quedaremos con los tres mejores caminos
for r = 1:numRobots
    
    startNode = robotStartNodes(r, :);
    goalNode = robotGoalNodes(r, :);
    
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
    bestPaths=optimizeBestPaths(bestPaths,mapa);
    %bestPaths=updateCosts(bestPaths);
    bestPathsRobots{r} = bestPaths(1);

end
%clase para simular robots
robots = Robot.empty(length(bestPathsRobots), 0);
for i=1:length(bestPathsRobots)
    
    robots(i)=Robot(bestPathsRobots{i}.opt_path,length(bestPathsRobots),bestPathsRobots{i}.cost,bestPathsRobots{i}.rot_cost,0);
    robots(i).update_costs();
    fprintf("Robot: %d Longitud=%.2f Curvatura promedia =%.2f\n",i,robots(i).cost_l,robots(i).cost_w)

end
applyCollisionAvoidance(robots,mapa);
tiempo_transcurrido=toc;
fprintf('Tiempo transcurrido para realizar los calculos del algoritmo: %.2f\n',tiempo_transcurrido);    
addpath("lineTracker_MRS_ACO\")
lineTracker_MRS_ACO_V2(robots,mapa);


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




function [mapa_heuristic]=fun_heuristica(map,goalNode)
    [n,m]=size(map);
    mapa_heuristic=zeros(n,m);
        for i=1:n
            for j=1:m
                %distancia euclidiana 
                distancia=sqrt((i-goalNode(1))^2+(j-goalNode(2))^2);
                if distancia==0
                    mapa_heuristic(i,j)=1;
                else
                    mapa_heuristic(i,j)=1/distancia;
                end
                
            end
        end
        mapa_heuristic(goalNode(1),goalNode(2))=1.2;
end

function bestPaths = resetBestPaths(n)
% Crea un vector de n estructuras vacías con los campos definidos

    % Inicializa el primer elemento
    vacio = struct('opt_path', {}, 'cost', {},'path',{} ,'rot_cost', {});
    
    % Rellena el resto
    bestPaths = repmat(vacio, 1, n);
end