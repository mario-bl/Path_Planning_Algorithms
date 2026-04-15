%%PRUEBA DE PROGRAMACION ORIENTADA A OBJETOS
%%Creacion de clase hormiga
classdef Ant < handle

    properties
        x%fila
        y%colmna
        path%camino que sigue la hormiga
        opt_path;
        cost%coste
        rot_cost
        direction%para la heuristica de Yang, necesitamos esta metrica
    end

    methods
        %Constructor de la clase
        function obj=Ant(startNode,goalNode)
            obj.y=startNode(1);%fila=y
            obj.x=startNode(2);%columna=x
            obj.path=startNode;
            obj.opt_path;
            obj.cost=0;
            obj.rot_cost=0;
            obj.direction=atan2(goalNode(1)-startNode(1),goalNode(2)-startNode(2));%
        end
            
        %Funcion para mover la hormiga de un nodo a otro y actualizar
        %camino
        function move(obj,newNode)
            %Funcion de costo super simple que solo aumente con distancia
            %recorrrida
            obj.cost=obj.cost+sqrt((newNode(1)-obj.y)^2+(newNode(2)-obj.x)^2);
            obj.direction=atan2(newNode(1)-obj.y,newNode(2)-obj.x); %Actualizamos la direccion del movimiento
            obj.x=newNode(2);
            obj.y=newNode(1);
            obj.path=[obj.path;newNode];  
            
                      
        end
        
        %Reiniciar la hormiga
        function reset(obj,startNode,goalNode)
            obj.x=startNode(2);%columna=x
            obj.y=startNode(1);%fila=y
            obj.path=startNode;
            obj.cost=0;
            obj.direction=atan2(goalNode(1)-startNode(1),goalNode(2)-startNode(2));%

        end

    end
end