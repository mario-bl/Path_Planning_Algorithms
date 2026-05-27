function [pathsRobots] = collisionAvoidance(pathsRobots,mapa)
safety_dist = 1.5;
nRob = length(pathsRobots);
lengths = cellfun(@(r) size(r.path,1), pathsRobots);
maxT = max(lengths);
recalcT_matrix = zeros(nRob);
for t = 2:maxT
    positions = nan(nRob,2);
    prevPos = nan(nRob,2);
    for r = 1:nRob
        if t <= size(pathsRobots{r}.path, 1)
            positions(r, :) = pathsRobots{r}.path(t, :);
            prevPos(r, :) = pathsRobots{r}.path(t-1, :);
        else
            positions(r, :) = pathsRobots{r}.path(end, :); % Se queda quieto al final
            prevPos(r, :) = pathsRobots{r}.path(end, :);
        end
    end

    for i = 1:nRob
        recalculatePath = false;
        rk=0;
        dataRecalc = struct('pos',[],'t',[]);
        for j=i+1:nRob
            if (norm(positions(i,:) - positions(j,:)) < safety_dist) && (t > (recalcT_matrix(i,j)+3))
                pathsRobots{j} = addStopPoint(pathsRobots{j},t); %%4 points on the path
                recalculatePath = true;
                rk = rk + 1;
                dataRecalc.pos = [dataRecalc.pos; prevPos(i,:) ;positions(j,:)];
                dataRecalc.t = [dataRecalc.t; t-1; t]; 
                dataRecalc = addAditionalcells(dataRecalc, pathsRobots{i}.path);
                recalcT_matrix(i,j) = t;
            end
        end
        if recalculatePath
            startPoint = pathsRobots{i}.path(1,:);
            endPoint = pathsRobots{i}.path(end,:);
            pathsRobots{i} = IACO_CollisionAvoidnace(dataRecalc,mapa,startPoint,endPoint);
        end
    end


end






end


function [pathRobotStruct]=addStopPoint(pathRobotStruct,t)
l = size(pathRobotStruct.path,1);
if t>= l
    return %%No es necesario parar el robot pq ya esta quieto
end
path1 = pathRobotStruct.path(1:t,:);
path2 = ones(2,1) * pathRobotStruct.path(t,:);
path3 = pathRobotStruct.path(t:end,:);
pathRobotStruct.path = [path1;path2;path3];

end

function [RobotStruct] = IACO_CollisionAvoidnace(dataRecalc,mapa,startNode,endPoint)
    numAnts=10;
    maxIter=50;
    alpha=1;
    beta=8;
    rho=0.1;
    bestPaths= struct('cost', {},'path',{} ,'rot_cost', {}); %Nos quedaremos con los tres mejores caminos
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
    RobotStruct = bestPaths(1);
end


function [matriz_vecinos]=celdas_accesibles(mapaObs,ant,time,dataRecalc)
    [n,m]=size(mapaObs);
    matriz_vecinos=[];
    
    for i=max(1,ant.y-1):min(n,ant.y+1)
        for j=max(1,ant.x-1):min(m,ant.x+1)
            %colisionFlag = ((time == dataRecalc.t) && (dataRecalc.pos(1)==i) && (dataRecalc.pos(2)==j));%Condicion para 
            colisionFlag = obtainColisionFlag(dataRecalc, [i,j], time);
            if mapaObs(i,j)==1 && (ant.y~=i || ant.x~=j) && ~colisionFlag %Condicion espacio libre y no se queda atrapada en el mismo nodo
                matriz_vecinos=[matriz_vecinos;[i,j]];
                %%Se almacena las coordenadas del espacio libre 
            end
        end
    end
    
end

