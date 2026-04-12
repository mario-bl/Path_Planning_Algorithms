function [obstacles,flag]=checkLocalMinima(rob,obstacles,l,d_inf,flag,k_rep_v) %%Basado en Cooperative UAV_APF
    
    n=length(obstacles); %Numero de obstaculos
    m=size(rob.path,1);%Numero de puntos del camino
    if ((m>4)&&(norm(rob.path(end,:)-rob.path(max(m-2,1),:))<(1.1*l)) && flag)
        %Se añade un obstaculo virtual pequeño con un factor aleatorio
        %respecto a la linea que orienta robot y goal
        theta=atan2(rob.y-rob.goal(2),rob.x-rob.goal(1))+(rand-0.5)*60*pi/180;%angulo aleatorio +30 -30
        obstacles(n+1)=struct('x', rob.x+0.5*d_inf*cos(theta), 'y', rob.y+0.5*d_inf*sin(theta), 'r', 0.4,'colour','green','k_rep',k_rep_v);%En verde para que se vea que es virtual
        flag=false;
    end

    if (norm(rob.path(end,:)-rob.path(max(m-3,1),:))>1.5*l)
        flag=true;%%Cuando ya se alejado activamos la bandera de nueva
    end



end