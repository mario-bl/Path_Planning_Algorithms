function isValid = checkRobPosition(startNode, goalNode, mapa)
    isValid = true;

    % Verificar nodo de inicio
    if mapa(startNode(1), startNode(2)) == 0
        fprintf('Error: la posición inicial [%d, %d] está ubicada en un obstáculo.\n', startNode(1), startNode(2));
        isValid = false;
    end

    % Verificar nodo objetivo
    if mapa(goalNode(1), goalNode(2)) == 0
        fprintf('Error: la posición objetivo [%d, %d] está ubicada en un obstáculo.\n', goalNode(1), goalNode(2));
        isValid = false;
    end
end