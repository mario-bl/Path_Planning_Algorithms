function [bestPaths] = updateBestPaths(ants, bestPaths)
    for i = 1:length(ants)
        pathUnico = true;

        % Verificar si ya existe ese camino
        for j = 1:length(bestPaths)
            if ~isempty(bestPaths(j).path) && isequal(bestPaths(j).path, ants(i).path)
                pathUnico = false;
                break;
            end
        end

        if pathUnico
            nuevo = struct('opt_path', NaN, ...
                           'cost', ants(i).cost, ...
                           'path', ants(i).path, ...
                           'rot_cost', ants(i).rot_cost);

            if length(bestPaths) < 3
                bestPaths(end+1)=nuevo;
            else
                % Evaluar si mejora el peor camino
                [~, idxMax] = max([bestPaths.cost] + 1e-3 * [bestPaths.rot_cost]);
                if (ants(i).cost < bestPaths(idxMax).cost) || ...
                   ((ants(i).cost == bestPaths(idxMax).cost) && ...
                    (ants(i).rot_cost < bestPaths(idxMax).rot_cost))
                    bestPaths(idxMax) = nuevo;
                end
            end
        end
    end

    % Ordenar bestPaths por coste y rotación
    if ~isempty(bestPaths)
        % Filtrar entradas no vacías
        idx_validos = ~cellfun(@isempty, {bestPaths.path});
        validos = bestPaths(idx_validos);

        % Ordenar por coste y rot_cost
        [~, orden] = sortrows([[validos.cost]' [validos.rot_cost]'], [1 2]);
        bestPaths(idx_validos) = validos(orden);
    end
end