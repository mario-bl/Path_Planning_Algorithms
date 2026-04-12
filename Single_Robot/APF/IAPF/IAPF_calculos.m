function [F]=IAPF_calculos(rob,obs,goal,cts)
    d_max=7;
    xGoal=goal(1,1);
    yGoal=goal(1,2);
    F_att_x=cts.k_att*(xGoal-rob.x);
    F_att_y=cts.k_att*(yGoal-rob.y);
    %Inicializamos fuerzas repulsivas
    F_rep_x=0;
    F_rep_y=0;
    %Revisamos si algun obstaculo esta en radio de accion

    for i=1:length(obs)
        %Distancia del robot a los obstaculos
        d_obs=sqrt((obs(i).x-rob.x)^2+(obs(i).y-rob.y)^2)-obs(i).r; %Le restamos el radio del obstaculo

        if(d_obs<cts.d_inf) %Distancia al obstaculo con el engorde esta dentro de la influencia
            %Primer termino repulsivo
            F_rep_x=F_rep_x+obs(i).k_rep*(1/d_obs-1/cts.d_inf)/(d_obs^3)*min((rob.x-xGoal)^2+(rob.y-yGoal)^2,d_max)*(rob.x-obs(i).x); 
            F_rep_y=F_rep_y+obs(i).k_rep*(1/d_obs-1/cts.d_inf)/(d_obs^3)*min((rob.x-xGoal)^2+(rob.y-yGoal)^2,d_max)*(rob.y-obs(i).y); 
            %Segundo termino repulsivo falta un signo negativo
            F_rep_x=F_rep_x-obs(i).k_rep*(1/d_obs-1/cts.d_inf)^2*(rob.x-xGoal);
            F_rep_y=F_rep_y-obs(i).k_rep*(1/d_obs-1/cts.d_inf)^2*(rob.y-yGoal);
        end
        

    end
    
    Fx=F_att_x+F_rep_x;
    Fy=F_att_y+F_rep_y;
    %Output de la funcion
    F=[Fx,Fy];
end