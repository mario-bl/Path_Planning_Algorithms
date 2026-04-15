%Funcion que se encarga de seleccionar uno de los vecinos accesibles en
%base a su probabilidad
function [nextNode]=selectNextNode(probabilities,neighbors,previousNode)
    % Comprobar si no se ha asignado un nodo, asignar el nodo de origen y
    % que itere 
    if isempty(probabilities)
        nextNode = previousNode;  % Asignar el primer vecino como valor predeterminado
        return;
    end
    r=rand;%valor aleatorio a comparar
    probabilities=probabilities/sum(probabilities);
    prob_acum=0; %prob acumulada para la seleccion lo inicializamos 
    for i=1:size(neighbors,1)
        prob_acum=prob_acum+probabilities(i);
        if prob_acum>=r
            nextNode=neighbors(i,:);
            return;
        end
    end
    %En caso de que no se seleccione ninguno se devuelve el primero
    nextNode=neighbors(1,:);

end