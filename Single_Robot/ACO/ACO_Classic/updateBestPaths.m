%Funcion que evalua los caminos descubiertos de las 10 hormigas de cada
%iteracion y en caso de ser mejores los almacena

function [bestPaths] = updateBestPaths(ants, bestPaths)
    % Evaluar cada hormiga
    for i = 1:length(ants)
        pathUnico = true;

        % Verificar si ya existe el camino
        for j = 1:length(bestPaths)
            if ~isempty(bestPaths(j).path) && isequal(bestPaths(j).path, ants(i).path)
                pathUnico = false;
                break;
            end
        end

        % Si el camino es único, evaluamos si lo añadimos
        if pathUnico
            nuevo = struct('path', ants(i).path, 'cost', ants(i).cost, 'rot_cost', ants(i).rot_cost);
            
            if length(bestPaths) < 3
                bestPaths(end+1) = nuevo;
            else
                % Buscar el peor (mayor coste, y en caso de empate, mayor rot_cost)
                [~, idxMax] = max([bestPaths.cost] + 1e-3 * [bestPaths.rot_cost]);
                if (ants(i).cost < bestPaths(idxMax).cost) || ...
                   ((ants(i).cost == bestPaths(idxMax).cost) && ...
                    (ants(i).rot_cost < bestPaths(idxMax).rot_cost))
                    bestPaths(idxMax) = nuevo;
                end
            end
        end
    end

    % Ordenar los caminos por coste (y rot_cost en caso de empate)
    if ~isempty(bestPaths)
        [~, idxOrden] = sortrows([[bestPaths.cost]' [bestPaths.rot_cost]'], [1 2]);
        bestPaths = bestPaths(idxOrden);
    end
end