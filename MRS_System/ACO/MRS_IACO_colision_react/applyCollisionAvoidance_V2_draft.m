function applyCollisionAvoidance_V2_draft(robots, mapa, safety_dist)
    if nargin < 3
        safety_dist = 1.5; % umbral de distancia crítica para colisión
    end

    nRobots = length(robots);
    maxT = max(cellfun(@(r) size(r.smoothedPath, 1), num2cell(robots)));

    flag_matrix = zeros(nRobots); % 0 = nada, 1 = espera al otro, 2 = es esperado

    for t = 2:maxT
        positions = nan(nRobots, 2); % Posiciones actuales en el tiempo t

        % Obtener posiciones de cada robot en este instante
        for r = 1:nRobots
            if t <= size(robots(r).smoothedPath, 1)
                positions(r, :) = robots(r).smoothedPath(t, :);
            else
                positions(r, :) = robots(r).smoothedPath(end, :); % Se queda quieto al final
            end
        end

        % Comprobación de conflictos entre pares
        for i = 1:nRobots
            recalculatePath = false;
            rk=0;
            dataRecalc = struct('pos',{},'t',{});
            for j = i+1:nRobots
                % Comprobación robusta antes de acceder a .log_stop(end)

                if norm(positions(i,:) - positions(j,:)) < safety_dist && ...
                   (isempty(robots(i).log_stop) && isempty(robots(j).log_stop) || ...
                    (~isempty(robots(i).log_stop) && robots(i).log_stop(end) + 150 < t) || ...
                    (~isempty(robots(j).log_stop) && robots(j).log_stop(end) + 150 < t))
                    % Conflicto detectado → el robot con menor prioridad se detiene

                    % El robot j espera
                    recalculatePath = true;
                    rk = rk + 1;
                    robots(j).smoothedPath = [robots(j).smoothedPath(1:t-1,:); ...
                                              robots(j).smoothedPath(t-1,:); ...
                                              robots(j).smoothedPath(t:end,:)];
                    robots(j).log_stop=[robots(i).log_stop;t];%ALmacenamos la fila en la que hay que parar el robot
                    flag_matrix(i,j) = 2;
                    flag_matrix(j,i) = 1;
                    dataRecalc(rk).pos = positions(j,:);
                    dataRecalc(rk).t = t;                     
                    
                end

            end

            if recalculatePath
                startPoint = robots(i).smoothedPath(1,:);
                endPoint = robots(i).smoothedPath(end,:);
                IACO_CollisionAvoidnace(dataRecalc,mapa,startPoint,endPoint,robots(i));
            end

        end
    end
end


function IACO_CollisionAvoidnace(dataRecalc,mapa,startNode,endPoint,robot)
    numAnts=10;
    maxIter=50;
    alpha=1;
    beta=8;
    rho=0.1;
    bestPaths= struct('opt_path', {}, 'cost', {},'path',{} ,'rot_cost', {}); %Nos quedaremos con los tres mejores caminos
    pherom = inic_map_feromonas(mapa);
    ants = Ant.empty(numAnts, 0);
    for i = 1:numAnts
        ants(i) = Ant(startNode,endPoint);
    end

    for t = 1:maxIter
        for i = 1:numAnts
            ants(i).reset(startNode,endPoint);
            time=0;
            while ~isequal(endPoint, [ants(i).y, ants(i).x])
                time = time +1;
                heuristic=Yang_heuristic(mapa,endPoint,startNode,ants(i),time,dataRecalc);
                celdasVecinas = celdas_accesibles(mapa, ants(i),time,dataRecalc);
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
    robot.updatepath(bestPaths(1).path,bestPaths(1).cost,bestPaths(1).rot_cost);
end


function [matriz_vecinos]=celdas_accesibles(mapaObs,ant,time,dataRecalc)
    [n,m]=size(mapaObs);
    matriz_vecinos=[];
    
    for i=max(1,ant.y-1):min(n,ant.y+1)
        for j=max(1,ant.x-1):min(m,ant.x+1)
            colisionFlag = ((time == dataRecalc.t) && (dataRecalc.pos(1)==i) && (dataRecalc.pos(2)==j));%Condicion para 
            if mapaObs(i,j)==1 && (ant.y~=i || ant.x~=j) && ~colisionFlag %Condicion espacio libre y no se queda atrapada en el mismo nodo
                matriz_vecinos=[matriz_vecinos;[i,j]];
                %%Se almacena las coordenadas del espacio libre 
            end
        end
    end
    
end

