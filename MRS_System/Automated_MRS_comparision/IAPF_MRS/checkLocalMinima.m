function checkLocalMinima(rob,l,d_inf,k_rep_v) %%Basado en Cooperative UAV_APF
    
    n=length(rob.obs); %Numero de obstaculos
    m=size(rob.path,1);%Numero de puntos del camino
    if ((norm(rob.path(end,:)-rob.path(max(m-2,1),:))<(1.1*l)) && rob.virtualFlag && (rob.it_Virtual==0))
        %Se añade un obstaculo virtual pequeño con un factor aleatorio
        %respecto a la linea que orienta robot y goal
        %fprintf("El robot ha encontrado un minimo local en (%.2f,%.2f) \n",rob.x,rob.y);
        %fprintf("Se añadera un objeto virtual para salir del minimo \n");
        theta=atan2(rob.y-rob.goal(2),rob.x-rob.goal(1))+(rand-0.5)*60*pi/180;%angulo aleatorio +30 -30
        rob.obs(n+1)=struct('x', rob.x+0.1*d_inf*cos(theta), 'y', rob.y+0.1*d_inf*sin(theta), 'r', 0.2,'colour','green','k_rep',k_rep_v);%En verde para que se vea que es virtual
        rob.virtualFlag=false;
        rob.it_Virtual=size(rob.path,1);


    elseif ((norm(rob.path(end,:)-rob.path(max(m-3,1),:))>1.5*l) && (size(rob.path,1)>=(rob.it_Virtual+50)))
        rob.virtualFlag=true;%%Cuando ya se alejado activamos la bandera de nueva
        rob.it_Virtual=0;
    end



end