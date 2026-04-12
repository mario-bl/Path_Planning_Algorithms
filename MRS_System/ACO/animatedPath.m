function animatedPath(robots, mapa)
    figure;
    imagesc(mapa);
    colormap(gray);
    axis equal;
    hold on;

    numRobots = length(robots);
    colors = lines(numRobots); % Paleta de colores distinta por robot

    % Inicialización de gráficos: líneas y puntos
    h = gobjects(numRobots, 1);  % Líneas de trayectoria
    p = gobjects(numRobots, 1);  % Posición actual del robot

    for j = 1:numRobots
        if ~isempty(robots(j).smoothedPath)
            h(j) = plot(robots(j).smoothedPath(1,2), robots(j).smoothedPath(1,1), ...
                        '-', 'Color', colors(j,:), 'LineWidth', 1.5);
            p(j) = plot(robots(j).smoothedPath(1,2), robots(j).smoothedPath(1,1), ...
                        'o', 'MarkerSize', 8, 'MarkerFaceColor', colors(j,:), 'MarkerEdgeColor', 'k');
        end
    end

    % Determinar longitud máxima de las trayectorias suavizadas
    max_points = max(arrayfun(@(r) size(r.smoothedPath, 1), robots));

    % Animación principal
    for i = 2:max_points
        for j = 1:numRobots
            path = robots(j).smoothedPath;
            if i <= size(path, 1)
                % Actualizar trayectoria parcial
                set(h(j), 'XData', path(1:i,2), 'YData', path(1:i,1));
                set(p(j), 'XData', path(i,2), 'YData', path(i,1));
            end
        end
        pause(0.005);
        drawnow;
    end

    title('Animación del seguimiento de trayectorias suavizadas (ACO)');
end