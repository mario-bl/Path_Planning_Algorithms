function plotMRSpaths(Robots,mapa)
    [n,m]=size(mapa);
    colors={'r','g','b'};
    figure 
    imagesc(mapa);
    colormap("gray");
    hold on;
    for r=1:length(Robots)
        camino=Robots(r).real_path;
        smoothPath=Robots(r).smoothedPath;
        plot(camino(:,2),camino(:,1),'k--','LineWidth',0.5);
        plot(smoothPath(:,2),smoothPath(:,1),'Color',colors{r},'LineWidth',0.5)
        
    
    
    
    end
    % for k = 0:n  % Líneas horizontales
    %             plot([0, m], [k, k] + 0.5, 'k', 'LineWidth', 0.5);
    % end
    %      % Líneas verticales
    % for j = 0:m 
    %     plot([j, j] + 0.5, [0, n], 'k', 'LineWidth', 1);
    % end

end
