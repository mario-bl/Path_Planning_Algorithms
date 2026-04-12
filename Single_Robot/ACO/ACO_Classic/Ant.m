%%PRUEBA DE PROGRAMACION ORIENTADA A OBJETOS
%%Creacion de clase hormiga
classdef Ant < handle %sean clases por referencia y no copia-->realizar cambios en las instancias

    properties
        x%fila
        y%colmna
        path%camino que sigue la hormiga
        opt_path%camino optimizado
        cost%coste
        rot_cost
    end

    methods
        %Constructor de la clase
        function obj=Ant(startNode)
            obj.x=startNode(1);
            obj.y=startNode(2);
            obj.path=[startNode(1),startNode(2)];
            obj.opt_path;
            obj.cost=0;
            obj.rot_cost=0;
        end
            
        %Funcion para mover la hormiga de un nodo a otro y actualizar
        %camino
        function move(obj,newNode)
            %Funcion de costo super simple que solo aumente con distancia
            %recorrrida
            obj.cost=obj.cost+sqrt((newNode(1)-obj.x)^2+(newNode(2)-obj.y)^2);
            obj.x=newNode(1);
            obj.y=newNode(2);
            obj.path=[obj.path;newNode];  
                      
        end
        
        %Reiniciar la hormiga
        function reset(obj,startNode)
            obj.x=startNode(1);
            obj.y=startNode(2);
            obj.path=startNode;
            obj.cost=0;

        end

    end
end