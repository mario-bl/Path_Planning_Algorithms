function optimizePath(ant,mapa)
    % Funcion para optimiar el camino basandonos en Geometri optimization
    %usamos metodo linea de vision (line-of-sight check)
    optimized_path=ant.path;
    cost_path=0;
    i=1;
    while i <length(optimized_path)-1
        j=i+2;
        while j<length(optimized_path) %bucle que busca desde el punto i un punto accesible
            if lineOfSight(optimized_path(i,:),optimized_path(j,:),mapa)
                optimized_path(i+1:j-1,:)=[];
                %eliminamos del camino todas las filas (coordenadas) de
                %celdas que no son necesarias recorrer
                j=i+2;%reinicio del indice cuando se elminan filas
            else
                j=j+1;%Probamos siguiente punto
            end
        end
        i=i+1;
    end

    for k=1:length(optimized_path)-1
        cost_path=cost_path+norm(optimized_path(k,:)-optimized_path(k+1,:));
    end
    ant.opt_path=optimized_path;
    ant.cost=cost_path;

end

function isClear=lineOfSight(p1,p2,mapa) 
    %Funcion para verificar si la linea entre los dos puntos no tiene
    %obstaculo alguno
    [n, m] = size(mapa);  % Dimensiones del mapa
    isClear=true;
    for dx=-0.6:0.6:0.6
        for dy=-0.6:0.6:0.6
            % Ajustar los valores para evitar salir de los límites
            p2_x = min(max(round(p2(2) + dx), 1), m);
            p2_y = min(max(round(p2(1) + dy), 1), n);
            
            % Evitar que p2 coincida con p1
            if p2_x == p1(2) && p2_y == p1(1)
                continue; % Saltamos esta iteración
            end

            [x,y]=bresenham(p1(2),p1(1),p2_x,p2_y);
            ind=sub2ind(size(mapa),y,x);%conversion a indices de matriz-->permite indexacion multiple 
            if ~all(mapa(ind)==1) %condicion de falso--> se atraviesa un obstaculo
                isClear=false;
                break
            end
        end
    end
end



