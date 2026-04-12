function [probabilities]=computeProbabilities(neighbors,hormiga, pheromones, heuristics, alpha, beta)
    k=size(neighbors,1);%Seleccion del numero de filas==numero de vecinos
    if k==0
        fprintf('En la celda (%d, %d) no se tienen vecinos accesibles',hormiga.y,hormiga.x)
        probabilities=[];
        return;%Salida en caso de que no se tengan vecinos accesibles
    end
    
    probabilities=zeros(k,1); %Vector que almacene las probabilidades 

    %Bucle para el calculo de la probabilidad de cada vecino
    pTotal=0;
    for i=1:k
        node=neighbors(i,:);%Almacenamos el nodo del vecino que se quiere evaluar

        % Verificamos que los valores no sean NaN ni ceros para evitar errores
        pheromone_value = pheromones(node(1), node(2));
        heuristic_value = heuristics(node(1), node(2));
        
        % Evitar NaN o valores cero
        if isnan(pheromone_value) || isnan(heuristic_value)
            fprintf('Advertencia: NaN detectado en pheromones o heuristics en el nodo (%d, %d).\n', node(1), node(2));
            continue;  % Saltamos el nodo si hay NaN
        end
        
        if isnan(pheromone_value) || isnan(heuristic_value) || pheromone_value == 0 || heuristic_value == 0
            fprintf('Advertencia: Valor invalido  en el nodo (%d, %d).\n', node(1), node(2));
            continue;  % Saltamos el nodo si hay cero
        end

        % Calculamos la probabilidad para este vecino
        probabilities(i) = (pheromone_value ^ alpha) * (heuristic_value ^ beta);
        pTotal=pTotal+probabilities(i);%Vamos sumando la probabilidad total en cada iteracion  
    end
   
    if pTotal==0 %caso que la suma total sea nula, probabilidades uniformes
        probabilities=ones(k,1)/k;
    end
    
    probabilities=probabilities/pTotal;
    %Devuelve un vector columna con la probabilida de cada vecino

end