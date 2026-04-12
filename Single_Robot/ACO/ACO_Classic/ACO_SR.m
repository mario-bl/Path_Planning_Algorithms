%ALGORITMO ACO CLÁSICO SIN MEJORAS
%******************DEFINICION DE MAPA DE TRABAJO ***************
%***************************************************************
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

%Ahora hay que aplicar la inicializacion de las feromonas al mapa
pherom=inic_map_feromonas(mapa);
%Para posterior representacion
pherom_graf=pherom;
pherom_graf(~mapa)=min(pherom_graf(:))-1; 
[nRows,nCols]=size(mapa);
%***************************************************************


%*******POSICION DE SALIDA DEL ROBOT******
%****************************************
startNode=[20,1];%Nodo inicial
goalNode=[1,20];%Nodo objetivo
%****************************************

numAnts=10;
maxIter=150;
alpha=1;%Importancia feromonas
beta=8;%Importancia heuristica
rho=0.1;%Tasa evaporacion
heuristic=fun_heuristica(mapa,goalNode);%%Matriz con huristica creada al punto final
%Se van a realizar 100 iteraciones con poblaciones de 10 hormigas
%Nos quedaremos con los tres mejores caminos
bestPaths=struct('path',{},'cost',{},'rot_cost',{});
bestCost=Inf;
ants=Ant.empty(numAnts,0);

for i=1:numAnts
    ants(i)=Ant(startNode); %constructor de clase
end
%Bucle prinicpal
log_iter=zeros(1,maxIter);
for t=1:maxIter
    n_it=0;  
    for i=1:numAnts
        ants(i).reset(startNode);
        while ~isequal(goalNode,[ants(i).x,ants(i).y])%Condicion para alcanzar nodo objetivo
            celdasVecinas=celdas_accesibles(mapa,ants(i));%Obtencion de vector con celdas accesibles disponibles desde el nodo actual
            if isempty(celdasVecinas)
                fprintf('Hormiga %d atrapada en (%d, %d)', ants(i).x, ants(i).y);
                continue; 
            end
            probabilidades=computeProbabilities(celdasVecinas,ants(i),pherom,heuristic,alpha,beta);
            nextNode=selectNextNode(probabilidades,celdasVecinas,[ants(i).x,ants(i).y]);
            ants(i).move(nextNode);  
            n_it=n_it+1;
        end
    end
    n_it=n_it/numAnts; %promedio de cada iteracion
    log_iter(t)=n_it;
    pherom=updatePherom(pherom,ants,rho);
    for i=1:length(ants)
        rotationCostCalc(ants(i));
    end

    bestPaths=updateBestPaths(ants,bestPaths);
end
bestPaths=updateCosts(bestPaths);
tiempo_transcurrido=toc;
fprintf('Tiempo transcurrido para realizar los calculos del algoritmo: %.2f\n',tiempo_transcurrido); 
fprintf('Promedio iteraciones de cada hormiga:%.2f\n',mean(log_iter));

%Representacion fancy de obstaculos
cmap = colormap('parula');
cmap(1, :) = [0 0 0];    % Color negro para los obstáculos (RGB: [0,0,0])
imagesc(pherom_graf);
colormap(cmap);
title('Mapa inicial de feromonas');
xlabel('X (dm)')
ylabel('Y (dm)')
axis equal
%Imprimimos el mapa de feromonas actualizado
obstaculos=(mapa==0);%Matriz logica donde hay obstaculos
pherom(~mapa)=min(pherom(:))-1;%Seleccionamos aquellas posiciones que corresponden a un obstaculo real y bajamos un nivel a -1

figure;
imagesc(pherom);
colormap(cmap);
colorbar
title('Mapa final de feromonas');
xlabel('X (dm)')
ylabel('Y (dm)')
axis equal
plotbestPaths(bestPaths,mapa,maxIter);

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
