function plotbestPaths(bestPaths,mapa,it)
    [n,m]=size(mapa);
    %colores caminos
    colors={'r','g','b'};
    figure;
    title(sprintf('ACO - %d iteraciones',it))
    for i=1:length(colors)
        subplot(1,3,i)
        %dibujamos mapa de osbactulos
        imagesc(mapa);
        colormap("gray");
        hold on;
        %axis equal;
        %trazamos el camino
        path=bestPaths(i).path;
        plot(path(:,2),path(:,1),'-','Color',colors{i},'LineWidth',2);
        %%Mallado del mapa pero desplazado 

        % Dibujar las líneas de la cuadrícula desplazadas
        for k = 0:n  % Líneas horizontales
            plot([0, m], [k, k] + 0.5, 'k', 'LineWidth', 0.5);
        end
         % Líneas verticales
        for j = 0:m 
            plot([j, j] + 0.5, [0, n], 'k', 'LineWidth', 1);
        end
        plot(path(end,2), path(end,1), '*m', 'MarkerSize', 15, 'LineWidth', 2);
        plot(path(1,2), path(1,1),'om', 'MarkerSize', 15, 'LineWidth', 2);
        %Titulo con el coste del camino
        title(sprintf('ACO-Camino %d, coste= %.2f',i,bestPaths(i).cost));
        xlabel('X (dm)')
        ylabel('Y (dm)')
        fprintf('Camino %d, Longitud= %.3f, Curvatura promedia: %.2f\n',i,bestPaths(i).cost,bestPaths(i).rot_cost)
    end
    %Update del mapa de feromonas 

end