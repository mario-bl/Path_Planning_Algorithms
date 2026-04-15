function applyCollisionAvoidance(robots, mapa, safety_dist)
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
            for j = i+1:nRobots
                % Comprobación robusta antes de acceder a .log_stop(end)

                if norm(positions(i,:) - positions(j,:)) < safety_dist && ...
                   (isempty(robots(i).log_stop) && isempty(robots(j).log_stop) || ...
                    (~isempty(robots(i).log_stop) && robots(i).log_stop(end) + 150 < t) || ...
                    (~isempty(robots(j).log_stop) && robots(j).log_stop(end) + 150 < t))
                                    % Conflicto detectado → el robot con menor prioridad se detiene
                    % if flag_matrix(i,j) == 0 && flag_matrix(j,i) == 0
                        if i < j
                            % El robot i espera
                            
                            robots(i).smoothedPath = [robots(i).smoothedPath(1:t-1,:); ...
                                                      robots(i).smoothedPath(t-1,:); ...
                                                      robots(i).smoothedPath(t:end,:)];
                            robots(i).log_stop=[robots(i).log_stop;t];%ALmacenamos la fila en la que hay que parar el robot
                            flag_matrix(i,j) = 1;
                            flag_matrix(j,i) = 2;
                        else
                            robots(j).smoothedPath = [robots(j).smoothedPath(1:t-1,:); ...
                                                      robots(j).smoothedPath(t-1,:); ...
                                                      robots(j).smoothedPath(t:end,:)];
                            robots(j).log_stop=[robots(j).log_stop;t];
                            flag_matrix(j,i) = 1;
                            flag_matrix(i,j) = 2;
                        end
                    
                end
            end
        end
    end
end
