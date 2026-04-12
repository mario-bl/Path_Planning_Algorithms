%Clase para cada robot

classdef Robot<handle %%hereda de la clase handle--> son clases que trabajan por eferencia y no copia

    properties
        x;
        y;
        v_x;
        v_y;       
        %%alpha;%orientacion del robot no necesaria de momento
        path;
        theta;%angulo resultante de la fuerza 
        cost_l;
        cost_w;%%Por ver como implementarlo
        goal;
    end

    methods

        function obj=Robot(x0,y0,Target,v_max)
            obj.goal=Target;
            obj.x=x0;
            obj.y=y0;
            obj.theta=atan((Target(1,2)-y0)/(Target(1,1)-x0));%inicializamos el angulo en direccion al punto final 
            obj.v_x=v_max*cos(obj.theta);
            obj.v_y=v_max*sin(obj.theta);
            obj.cost_l=0;
            obj.cost_w=0;%%se va sumando la velocidad angular que ha requerido el robot
            obj.path=[x0,y0];
        end

        function nextPosition(obj,Force,dT,l,f,v_max) %Factor de jitter que acorta el paso
            xOld=obj.x;
            yOld=obj.y;
            fx=Force(1,1);
            fy=Force(1,2);
            
            a_max = 0.6; % Ajustar este parámetro
            ax = (fx * dT);
            ay = (fy * dT);
            if norm([ax, ay]) > a_max
            ax = (ax / norm([ax, ay])) * a_max;
            ay = (ay / norm([ax, ay])) * a_max;
            end
            obj.v_x = obj.v_x + ax;
            obj.v_y = obj.v_y + ay;
           
            v_mag = norm([obj.v_x, obj.v_y]);
            
            if v_mag > v_max
                obj.v_x = (obj.v_x / v_mag) * v_max;
                obj.v_y = (obj.v_y / v_mag) * v_max;
            end 

            %Calculo del nuevo angulo resultante de las fuerzas para
            %evaluar Jitter
            newTheta=atan2(fy,fx);
            deltaTheta=mod(newTheta-obj.theta+pi,2*pi)-pi;
            if (1.4)<abs(deltaTheta) && abs(deltaTheta)<pi % Se cumple la condicion de Jitter
                %DYNAMIC STEP ADJUSTMENT
                obj.x=obj.x+f*l*cos(obj.theta+0.5*deltaTheta);
                obj.y=obj.y+f*l*sin(obj.theta+0.5*deltaTheta);

            else %Caso en el que no haya un cambio muy brusco de fuerzas
                obj.x = obj.x + sign(obj.v_x) * min(abs(obj.v_x * dT), l);
                obj.y = obj.y + sign(obj.v_y) * min(abs(obj.v_y * dT), l);
            
            end

            %%Vamos a pensar como salir de los minimos locales, para ello se
            %%establece una condicion en la que 

            % obj.cost_w=obj.cost_w+abs(newTheta-obj.theta)/dT;%Actualizamos el cambio de angulo entre iteraciones
            % 
            % obj.theta=newTheta;
            % obj.cost_l=obj.cost_l+sqrt((obj.x-xOld)^2+(obj.y-yOld)^2);%actualizamos el coste
            obj.theta=newTheta;
            obj.path=[obj.path;[obj.x,obj.y]];

        end

        function update_costs(obj)
            obj.cost_l=0;
            obj.cost_w=0;
            oldTheta=atan2(obj.path(end,2)-obj.path(1,2),obj.path(end,1)-obj.path(1,1));
            for i=1:(length(obj.path)-1)
                d=norm(obj.path(i+1,:)-obj.path(i,:));
                obj.cost_l=obj.cost_l+d;
                newTheta=atan2(obj.path(i+1,2)-obj.path(i,2),obj.path(i+1,1)-obj.path(i,1));
                dtheta=abs(angleDiff(newTheta,oldTheta));
                obj.cost_w=obj.cost_w+(dtheta/d);
                oldTheta=newTheta;   
            end
            obj.cost_w=obj.cost_w/(length(obj.path)-1); %Promedio de curvatura

        end

    end
end

            
        




