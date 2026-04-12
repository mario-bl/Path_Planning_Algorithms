function [bestPaths]=updateCosts(bestPaths)
    %Ojo para este caso primera columna (1) == coordenada_x y (2) == coordenada_y 
    for j=1:length(bestPaths)
        cost_l=0;
        cost_w=0;
        path=bestPaths(j).path;
        oldTheta=atan2(path(end,2)-path(1,2),path(end,1)-path(1,1));
        for i=1:(length(path)-1)
            d=norm(path(i+1,:)-path(i,:));
            cost_l=cost_l+d;
            newTheta=atan2(path(i+1,2)-path(i,2),path(i+1,1)-path(i,1));
            dtheta=abs(angleDiff(newTheta,oldTheta));
            cost_w=cost_w+(dtheta/d);
            oldTheta=newTheta;
        end
        cost_w=cost_w/(length(path)-1);%Promedio de curvatura
        bestPaths(j).cost=cost_l;
        bestPaths(j).rot_cost=cost_w;
    end
end