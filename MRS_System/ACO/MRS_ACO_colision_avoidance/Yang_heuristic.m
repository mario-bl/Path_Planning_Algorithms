
function [mapa_heuristic]=Yang_heuristic(mapa,goalNode,startNode,ant)
    [n,m]=size(mapa);
    mapa_heuristic=zeros(n,m); %Inicializamos a cero
    actualNode=ant.path(end,:);
    for i=max(1,ant.y-1):min(n,ant.y+1)
        for j=max(1,ant.x-1):min(m,ant.x+1)
            if mapa(i,j)==1 && (ant.y~=i || ant.x~=j) %Condicion espacio libre y no se queda atrapada en el mismo nodo
                d_sj=norm(startNode-[i,j]);
                d_ij=norm(actualNode-[i,j]);
                d_jE=norm(goalNode-[i,j]);
                jDirection=atan2(i-ant.y,j-ant.x);
                E_j=E_turn(ant.direction,jDirection);%Se calcula el factor de supresion de giros
               
                mapa_heuristic(i,j)=d_sj*E_j/(d_ij+d_jE); %Usamos la heurística de Yang
                if startNode==[i,j]
                    mapa_heuristic(i,j)=0.01;
                end
            end
        end
    end
        
end

function [E]=E_turn(actualDirection,nextDirection)
    u1=1;
    Lg=1;
    if (actualDirection==nextDirection)
        E=u1/Lg;
    else
        E=u1/(Lg*sqrt(2));
    end
end