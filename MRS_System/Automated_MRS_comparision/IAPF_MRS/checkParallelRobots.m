function [flag] = checkParallelRobots(rob1, rob2,l)
    % Función que verifica si dos robots siguen trayectorias paralelas.
    tol = 1e-2; % Tolerancia para comparación de pendientes
    
    % Número de puntos en la trayectoria
    n1 = size(rob1.path, 1);
    n2 = size(rob2.path, 1);
    
    % Asegurar que hay suficientes puntos para calcular 3 pendientes
    if n1 < 3 || n2 < 3
        flag = false;
        return;
    end
    
    % Inicializar pendientes
    m1 = zeros(1, 3);
    m2 = zeros(1, 3);
    
    % Calcular pendientes de los últimos 3 segmentos
    for i = 0:2
        idx1 = n1 - i;
        idx2 = n1 - (i + 1);
        idx3 = n2 - i;
        idx4 = n2 - (i + 1);
    
        dx1 = rob1.path(idx1, 1) - rob1.path(idx2, 1);
        dy1 = rob1.path(idx1, 2) - rob1.path(idx2, 2);
    
        dx2 = rob2.path(idx3, 1) - rob2.path(idx4, 1);
        dy2 = rob2.path(idx3, 2) - rob2.path(idx4, 2);
    
        % Evitar división por cero
        if dx1 == 0
            m1(i + 1) = Inf;
        else
            m1(i + 1) = dy1 / dx1;
        end
    
        if dx2 == 0
            m2(i + 1) = Inf;
        else
            m2(i + 1) = dy2 / dx2;
        end
    end
    
    % Verificar si las pendientes son aproximadamente iguales
    if all(abs(m1 - m2) < tol) %&& (norm(rob1.path(end,:)-rob2.path(max(n1-3,1),:))>1.5*l) && (norm(rob2.path(end,:)-rob2.path(max(n2-3,1),:))>1.5*l)
        flag = true;
    else
        flag = false;
    end
end




% function [flag]=checkParallelRobots(rob1,rob2) %Funcion que verifica si ambos estan realizando un movimiento en paralelo 
% m1=zeros(1,3);
% m2=zeros(1,3);
%     for i=0:2
%         m1(i+1)=(rob1.path(max(end-i,1),2)-rob1.path(max(end-(i+1),1),2))/(rob1.path(max(end-i,1),1)-rob1.path(max(end-(i+1),1),1));
%         m2(i+1)=(rob2.path(max(end-i,1),2)-rob2.path(max(end-(i+1),1),2))/(rob2.path(max(end-i,1),1)-rob2.path(max(end-(i+1),1),1));
%     end
% 
%     if(isequal(m1,m2)) %Ambas pendientes son iguales, robots van en paralelo--> insertaremos objeto virtual
%         flag=true;
%     else
%         flag=false;
%     end
% 
% end