function [bestPaths] = optimizeBestPaths(bestPaths, mapa)
    % Optimización geométrica del camino (line-of-sight)
    for l = 1:length(bestPaths)
        optimized_path = bestPaths(l).path;
        cost_path = 0;
        rot_cost = 0;
        i = 1;

        while i < length(optimized_path) - 1
            j = i + 2;
            while j < length(optimized_path)
                if lineOfSight(optimized_path(i,:), optimized_path(j,:), mapa)
                    optimized_path(i+1:j-1, :) = [];
                    j = i + 2;
                else
                    j = j + 1;
                end
            end
            i = i + 1;
        end
        bestPaths(l).opt_path = optimized_path;

        % Calcular el coste en distancia
        for k = 1:length(optimized_path)-1
            cost_path = cost_path + norm(optimized_path(k,:) - optimized_path(k+1,:));
        end
        bestPaths(l).cost = cost_path;
        
        % Calcular el coste en rotaciones
        oldTheta=atan2(bestPaths(l).opt_path(end,2)-bestPaths(l).opt_path(1,2),bestPaths(l).opt_path(end,1)-bestPaths(l).opt_path(1,1));%orientacion inicial

        for k = 2:length(optimized_path)
            newTheta=atan2(optimized_path(k,2)-optimized_path(k-1,2),optimized_path(k,1)-optimized_path(i-1,1));
            rot_cost=rot_cost+abs(oldTheta-newTheta);
            oldTheta=newTheta;    
        end
        bestPaths(l).rot_cost = rot_cost/length(bestPaths(l).opt_path);
    end

    % Reordenar los caminos por coste y luego por número de rotaciones
    [~, idx] = sortrows([[bestPaths.cost]', [bestPaths.rot_cost]']);
    bestPaths = bestPaths(idx);
    bestPaths=updateCosts(bestPaths);
end

function isClear = lineOfSight(p1, p2, mapa)
    % Verifica si hay línea de visión directa entre dos celdas
    [n, m] = size(mapa);
    isClear = true;
    for dx = -0.6:0.6:0.6
        for dy = -0.6:0.6:0.6
            p2_x = min(max(round(p2(2) + dx), 1), m);
            p2_y = min(max(round(p2(1) + dy), 1), n);
            if p2_x == p1(2) && p2_y == p1(1)
                continue;
            end
            [x, y] = bresenham(p1(2), p1(1), p2_x, p2_y);
            ind = sub2ind(size(mapa), y, x);
            if ~all(mapa(ind) == 1)
                isClear = false;
                return;
            end
        end
    end
end
