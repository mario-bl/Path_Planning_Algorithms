% ytEN este archivo se intenta generar un mapa similar al del articulo de Yang
%et al, futuras revisiones de este codigo intentaran implementar 
%la heuristica y la actualziacion de feromonas mejorada
%path=ants(1).path;
%hold on
%plot(path(:,1),path(:,2),'-','Color','b','LineWidth',2);
% Dimensiones del mapa
tic;
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

%Metodo para mejorar la inicializacion de feromonas del mapa
pherom=inic_map_feromonas(mapa);
cmap = colormap('parula');
cmap(1, :) = [0 0 0];
pherom_graf=pherom;
pherom_graf(~mapa)=min(pherom_graf(:))-1; %representacion feromonas inicializadas


[nRows,nCols]=size(mapa);
startNode=[18,2];%Nodo inicial
goalNode=[11,20];%Nodo objetivo
numAnts=10;
maxIter=50; %se bajan las iteraciones de 150 a 50, para ver que tan buena es la optimizacion
%%%MUY IMPORTANTE AJUSTAR ESTOS VALORES PARA CONVERGENCIA DEL ALGORITMO
alpha=1;%Importancia feromonas
beta=8;%Importancia heuristica
rho=0.1;%Tasa evaporacion

heuristic=fun_heuristica(mapa,goalNode);%Matriz con huristica creada al punto final
bestPaths=struct('opt_path',{},'cost',{},'path',{},'rot_cost',{}); %Nos quedaremos con los tres mejores caminos10000
bestCost=Inf;


ants=Ant.empty(numAnts,0);
for i=1:numAnts
    ants(i)=Ant(startNode); %Inicializacion de clase
end


for t=1:maxIter
   %fprintf('Iteracion nº: %d \n', t);
    for i=1:numAnts
        %fprintf('Hormiga nº: %d \n',i);
        ants(i).reset(startNode);
        while ~isequal(goalNode,[ants(i).x,ants(i).y])%Condicion para alcanzar nodo objetivo
            celdasVecinas=celdas_accesibles(mapa,ants(i));%Obtencion de vector con celdas accesibles disponibles desde el nodo actual 
            if isempty(celdasVecinas) 
                fprintf('Hormiga %d atrapada en (%d, %d)', ants(i).x, ants(i).y);
                continue; % Intentar en la siguiente iteración
            end
            probabilidades=computeProbabilities(celdasVecinas,ants(i),pherom,heuristic,alpha,beta);
            nextNode=selectNextNode(probabilidades,celdasVecinas,[ants(i).x,ants(i).y]);
            ants(i).move(nextNode);             
        end
    end
    pherom=updatePherom(pherom,ants,rho); %Actualizacion de feromonas tras alcanzarse final del camino
    for i=1:length(ants)
        optimizePath(ants(i),mapa);%proceso de optimizacion del camino a revisar
        rotationCostCalc(ants(i));
    end
    bestPaths=updateBestPaths(ants,bestPaths);
end
tiempo_transcurrido=toc;
fprintf('Tiempo transcurrido para realizar los calculos del algoritmo: %.2f\n',tiempo_transcurrido);    
%Representacion de mapas y recorridos 
figure;
imagesc(pherom_graf);
colormap(cmap);
axis equal;
title('Mapa inicial de feromonas');
xlabel('X')
ylabel('Y')
%Funcion que imprime los 3 mejores caminos sin optimizar
plotbestPaths(bestPaths,mapa);
% Opt_bestPaths=struct('path',{},'cost',{});
% for k=1:length(bestPaths)
%     [opt_path,opt_cost]=optimizePath(bestPaths(k).path,mapa);
%     Opt_bestPaths=[Opt_bestPaths,struct('path',{opt_path},'cost',{opt_cost})];
% 
% 
% end


%Imprimimos el mapa de feromonas actualizado
obstaculos=(mapa==0);%Matriz logica donde hay obstaculos
pherom(~mapa)=min(pherom(:))-1;%Seleccionamos aquellas posiciones que corresponden a un obstaculo real y bajamos un nivel a -1

figure;

%cmap = colormap('hot'); % Obtener el colormap estándar
%cmap(1, :) = [0 1 1];   % Color cian para los obstáculos (RGB: [0,1,1])
%colormap(cmap);

imagesc(pherom);
colormap(cmap);
colorbar
axis equal;
title('Mapa final de feromonas');
xlabel('X')
ylabel('Y')




%%Funciones AUXILIARES
%Caluclo Vecinos proximos sin restriccion de retorno y pudiendo hacer
%movimientos diagonales
function [matriz_vecinos]=celdas_accesibles(mapaObs,ant)
    [n,m]=size(mapaObs);
    matriz_vecinos=[];
    
    for i=max(1,ant.x-1):min(n,ant.x+1)
        for j=max(1,ant.y-1):min(m,ant.y+1)
            if mapaObs(i,j)==1 && (ant.x~=i || ant.y~=j) %Condicion espacio libre y no se queda atrapada en el mismo nodo
                matriz_vecinos=[matriz_vecinos;[i,j]];
                %%Se almacena las coordenadas del espacio libre 
            end
        end
    end
    
end



%Obtencion vecino siguiente

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
