function [flag]=continueAPF_algorithm(Robots,r_goal)
    flag=false; %presuponemos que no y que se han alcanzado los puntos finales
    n=size(Robots,2);
    for i=1:n
        if norm(Robots(i).path(end,:)-Robots(i).goal)>r_goal %Condicion de no alcance de punto objetivo
            flag=true; %Se han de coontinuar los calculos
            return;
        end
    end
end